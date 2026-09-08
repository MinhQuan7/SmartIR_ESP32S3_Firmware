#include "ac_protocol_probe.h"

#include "ac_ir_frame.h"
#include "config.h"

#include <IRac.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <math.h>
#include <string.h>

namespace {

struct ProbeScratch
{
  uint16_t rawA[AC_IR_MAX_RAW_LEN + 1] = {};
  uint16_t rawB[AC_IR_MAX_RAW_LEN + 1] = {};
  uint16_t rawSingle[AC_IR_MAX_RAW_LEN + 1] = {};
  decode_results resA = {};
  decode_results resB = {};
  decode_results resSingle = {};
};

ProbeScratch sProbeScratch;

bool tryDecodeFrame(const AcIrFrame &frame, decode_type_t vendor, const stdAc::state_t *prev,
                    stdAc::state_t *outState, int16_t *outModel)
{
  if (!outState || vendor == UNKNOWN)
    return false;
  if (!IRac::isProtocolSupported(vendor))
    return false;
  // decodeToState() reinterprets frame.state[] without re-validating checksum, so a
  // forced vendor is only valid when IRrecv natively decoded this frame as `vendor`.
  if (frame.decodeType != vendor)
    return false;

  sProbeScratch.resSingle = {};
  acIrFrameToDecodeResults(frame, vendor, &sProbeScratch.resSingle, sProbeScratch.rawSingle,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  if (!IRAcUtils::decodeToState(&sProbeScratch.resSingle, outState, prev))
    return false;
  if (sProbeScratch.resSingle.decode_type != vendor)
    return false;
  if (outModel)
    *outModel = outState->model;
  return true;
}

/**
 * Library-native path: IRAcUtils uses decode_type captured by IRrecv (no forced vendor).
 */
bool tryLibraryAutoPair(const AcIrFrame &frameA, const AcIrFrame &frameB,
                        stdAc::state_t *stateA, stdAc::state_t *stateB,
                        decode_type_t *outVendor, int16_t *outModel)
{
  if (!stateA || !stateB || !outVendor)
    return false;

  sProbeScratch.resA = {};
  sProbeScratch.resB = {};
  acIrFrameToDecodeResults(frameA, UNKNOWN, &sProbeScratch.resA, sProbeScratch.rawA,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  acIrFrameToDecodeResults(frameB, UNKNOWN, &sProbeScratch.resB, sProbeScratch.rawB,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));

  if (!IRAcUtils::decodeToState(&sProbeScratch.resA, stateA, nullptr))
    return false;
  if (!IRAcUtils::decodeToState(&sProbeScratch.resB, stateB, stateA))
    return false;

  const decode_type_t vendor = sProbeScratch.resA.decode_type;
  if (vendor == UNKNOWN || vendor != sProbeScratch.resB.decode_type)
    return false;
  if (!IRac::isProtocolSupported(vendor))
    return false;

  *outVendor = vendor;
  if (outModel)
    *outModel = stateA->model;
  return true;
}

/** Native IRrecv decode path for one captured frame (no forced vendor). */
bool tryLibraryNativeDecode(const AcIrFrame &frame, const stdAc::state_t *prev,
                            stdAc::state_t *outState)
{
  if (!outState)
    return false;

  sProbeScratch.resSingle = {};
  acIrFrameToDecodeResults(frame, UNKNOWN, &sProbeScratch.resSingle, sProbeScratch.rawSingle,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  return IRAcUtils::decodeToState(&sProbeScratch.resSingle, outState, prev);
}

bool nativeAllFourMatch(decode_type_t vendor, const AcIrFrame &frameOn,
                        const AcIrFrame &frameOff, const AcIrFrame &frameTempUp,
                        const AcIrFrame &frameTempDown)
{
  return vendor != UNKNOWN && frameOn.decodeType == vendor && frameOff.decodeType == vendor &&
         frameTempUp.decodeType == vendor && frameTempDown.decodeType == vendor;
}

bool tryDecodeTempFrame(const AcIrFrame &frame, const stdAc::state_t &baseOn,
                        const stdAc::state_t *prevLinked, stdAc::state_t *outState)
{
  if (!outState)
    return false;
  if (prevLinked && tryLibraryNativeDecode(frame, prevLinked, outState))
    return true;
  if (tryLibraryNativeDecode(frame, &baseOn, outState))
    return true;
  return tryLibraryNativeDecode(frame, nullptr, outState);
}

bool tryDecodeSequence(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                       const AcIrFrame *frameTempUp, const AcIrFrame *frameTempDown,
                       decode_type_t vendor, stdAc::state_t *stateOn, stdAc::state_t *stateOff,
                       stdAc::state_t *stateTempUp, stdAc::state_t *stateTempDown,
                       int16_t *outModel)
{
  if (!stateOn || !stateOff || !stateTempUp || !stateTempDown)
    return false;
  if (!tryDecodeFrame(frameOn, vendor, nullptr, stateOn, outModel))
    return false;
  if (!tryDecodeFrame(frameOff, vendor, stateOn, stateOff, nullptr))
    return false;
  if (!frameTempUp || !frameTempDown)
    return false;

  if (nativeAllFourMatch(vendor, frameOn, frameOff, *frameTempUp, *frameTempDown))
  {
    if (tryDecodeTempFrame(*frameTempUp, *stateOn, nullptr, stateTempUp) &&
        tryDecodeTempFrame(*frameTempDown, *stateOn, stateTempUp, stateTempDown))
      return true;
  }

  if (!tryDecodeFrame(*frameTempUp, vendor, stateOn, stateTempUp, nullptr))
    return false;
  return tryDecodeFrame(*frameTempDown, vendor, stateTempUp, stateTempDown, nullptr);
}

bool validatePowerPair(const stdAc::state_t &a, const stdAc::state_t &b)
{
  return a.power != b.power;
}

bool validateTempRise(const stdAc::state_t &baseState, const stdAc::state_t &upState)
{
  if (!baseState.power || !upState.power)
    return false;
  return fabsf(upState.degrees - baseState.degrees) >= 0.9f &&
         fabsf(upState.degrees - baseState.degrees) <= 1.1f;
}

bool validateTempDrop(const stdAc::state_t &upState, const stdAc::state_t &downState)
{
  if (!upState.power || !downState.power)
    return false;
  return fabsf(upState.degrees - downState.degrees) >= 0.9f &&
         fabsf(upState.degrees - downState.degrees) <= 1.1f;
}

bool validateTempRoundTrip(const stdAc::state_t &baseState, const stdAc::state_t &downState)
{
  if (!baseState.power || !downState.power)
    return false;
  return fabsf(baseState.degrees - downState.degrees) <= 0.25f;
}

bool validateSaneState(const stdAc::state_t &state)
{
  if (!state.power)
    return true;
  // 15C is a legit minimum on Sharp/AC remotes (kSharpAcMinTemp).
  return state.degrees >= 15.0f && state.degrees <= 33.0f;
}

bool validateOnOffOrientation(const stdAc::state_t &stateA, const stdAc::state_t &stateB)
{
  return stateA.power && !stateB.power;
}

bool isUntrustedProtocol(decode_type_t vendor)
{
  return acProtocolIsUntrusted(vendor);
}

bool nativeDecodeMatches(decode_type_t vendor, const AcIrFrame &frameA, const AcIrFrame &frameB)
{
  return frameA.decodeType == vendor && frameB.decodeType == vendor && vendor != UNKNOWN;
}

bool validateStableCoolingState(const stdAc::state_t &baseState, const stdAc::state_t &otherState)
{
  if (!baseState.power || !otherState.power)
    return false;
  if (baseState.mode != otherState.mode)
    return false;
  if (baseState.fanspeed != otherState.fanspeed)
    return false;
  return true;
}

uint8_t scoreFourStepProbe(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                           decode_type_t vendor, bool libraryAuto, const stdAc::state_t &stateOn,
                           const stdAc::state_t &stateOff, const stdAc::state_t &stateTempUp,
                           const stdAc::state_t &stateTempDown)
{
  if (!validateSaneState(stateOn) || !validateSaneState(stateOff) ||
      !validateSaneState(stateTempUp) || !validateSaneState(stateTempDown))
    return 0;
  if (!validatePowerPair(stateOn, stateOff) || !validateOnOffOrientation(stateOn, stateOff))
    return 0;
  if (!validateTempRise(stateOn, stateTempUp))
    return 0;
  if (!validateTempDrop(stateTempUp, stateTempDown))
    return 0;

  uint8_t score = 45;
  score += 15;
  if (validateTempRoundTrip(stateOn, stateTempDown))
    score += 12;
  if (validateStableCoolingState(stateOn, stateTempUp) &&
      validateStableCoolingState(stateOn, stateTempDown))
    score += 12;
  if (stateOn.model >= 0)
    score += 8;
  if (libraryAuto)
    score += 12;
  else if (nativeDecodeMatches(vendor, frameOn, frameOff))
    score += 6;
  if (acIrFrameSimilarity(frameOn, frameOff) >= 50)
    score += 6;
  if (isUntrustedProtocol(vendor))
    score = (score > 14) ? (uint8_t)(score - 14) : 0;

  return score;
}

struct ProbeBest
{
  uint8_t score = 0;
  decode_type_t vendor = UNKNOWN;
  int16_t model = -1;
  bool libraryAuto = false;
  stdAc::state_t stateOn = {};
  stdAc::state_t stateOff = {};
  stdAc::state_t stateTempUp = {};
  stdAc::state_t stateTempDown = {};
  bool hasTempEvidence = false;
};

void considerCandidate(ProbeBest *best, uint8_t *runnerUpScore, uint8_t score,
                       decode_type_t vendor, bool libraryAuto, const stdAc::state_t &stateOn,
                       const stdAc::state_t &stateOff, const stdAc::state_t *stateTempUp,
                       const stdAc::state_t *stateTempDown, int16_t model)
{
  if (!best || score == 0)
    return;

  if (score > best->score)
  {
    if (runnerUpScore)
      *runnerUpScore = best->score;
    best->score = score;
    best->vendor = vendor;
    best->model = model;
    best->libraryAuto = libraryAuto;
    best->stateOn = stateOn;
    best->stateOff = stateOff;
    best->hasTempEvidence = stateTempUp && stateTempDown;
    best->stateTempUp = stateTempUp ? *stateTempUp : stdAc::state_t{};
    best->stateTempDown = stateTempDown ? *stateTempDown : stdAc::state_t{};
    return;
  }

  if (score < best->score)
  {
    if (runnerUpScore && score > *runnerUpScore)
      *runnerUpScore = score;
    return;
  }

  if (libraryAuto && !best->libraryAuto)
  {
    best->vendor = vendor;
    best->model = model;
    best->libraryAuto = true;
    best->stateOn = stateOn;
    best->stateOff = stateOff;
    best->hasTempEvidence = stateTempUp && stateTempDown;
    best->stateTempUp = stateTempUp ? *stateTempUp : stdAc::state_t{};
    best->stateTempDown = stateTempDown ? *stateTempDown : stdAc::state_t{};
    return;
  }

  if (libraryAuto == best->libraryAuto && vendor < best->vendor)
  {
    best->vendor = vendor;
    best->model = model;
    best->stateOn = stateOn;
    best->stateOff = stateOff;
    best->hasTempEvidence = stateTempUp && stateTempDown;
    best->stateTempUp = stateTempUp ? *stateTempUp : stdAc::state_t{};
    best->stateTempDown = stateTempDown ? *stateTempDown : stdAc::state_t{};
  }
}

void scanAllIracProtocols(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                          const AcIrFrame *frameTempUp, const AcIrFrame *frameTempDown,
                          ProbeBest *best, uint8_t *runnerUpScore)
{
  for (int raw = 1; raw <= (int)kLastDecodeType; raw++)
  {
    const decode_type_t vendor = (decode_type_t)raw;
    if (!IRac::isProtocolSupported(vendor))
      continue;

    stdAc::state_t stateOn = {};
    stdAc::state_t stateOff = {};
    stdAc::state_t stateTempUpLocal = {};
    stdAc::state_t stateTempDownLocal = {};
    int16_t model = -1;
    if (!tryDecodeSequence(frameOn, frameOff, frameTempUp, frameTempDown, vendor, &stateOn,
                           &stateOff, &stateTempUpLocal, &stateTempDownLocal, &model))
      continue;
    const uint8_t score = scoreFourStepProbe(frameOn, frameOff, vendor, false, stateOn, stateOff,
                                             stateTempUpLocal, stateTempDownLocal);
    considerCandidate(best, runnerUpScore, score, vendor, false, stateOn, stateOff,
                      &stateTempUpLocal, &stateTempDownLocal, model);
  }
}

bool nativeCaptureWeak(const AcIrFrame &frameA, const AcIrFrame &frameB)
{
  if (frameA.decodeType == UNKNOWN || frameB.decodeType == UNKNOWN)
    return true;
  return acProtocolIsUntrusted(frameA.decodeType) || acProtocolIsUntrusted(frameB.decodeType);
}

bool acceptSemanticProbe(const ProbeBest &best, uint8_t runnerUpScore, decode_type_t autoVendor,
                         const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                         const AcIrFrame *frameTempUp, const AcIrFrame *frameTempDown)
{
  if (best.vendor == UNKNOWN || best.score < AC_PROBE_MIN_CONFIDENCE)
    return false;

  const bool weakNative = nativeCaptureWeak(frameOn, frameOff);
  const uint8_t minScore =
      best.libraryAuto ? AC_PROBE_MIN_CONFIDENCE : AC_PROBE_MIN_CONFIDENCE_SCAN;
  if (best.score < minScore)
    return false;

  const bool nativeConsensus =
      frameTempUp && frameTempDown &&
      nativeAllFourMatch(best.vendor, frameOn, frameOff, *frameTempUp, *frameTempDown);

  if (!best.hasTempEvidence)
  {
    if (!(best.libraryAuto && nativeConsensus))
      return false;
  }

  if (!best.libraryAuto && best.model < 0 &&
      !(acProtocolIsUntrusted(best.vendor) && best.score >= AC_PROBE_STRONG_OVERRIDE_CONF))
    return false;

  if (weakNative && !best.libraryAuto)
    return false;

  if (autoVendor != UNKNOWN && autoVendor != best.vendor && weakNative)
    return false;

  if (runnerUpScore > 0 &&
      best.score < (uint8_t)(runnerUpScore + AC_PROBE_WINNER_MARGIN))
    return false;

  if (acProtocolIsUntrusted(best.vendor))
  {
    const bool nativeFour =
        frameTempUp && frameTempDown &&
        nativeAllFourMatch(best.vendor, frameOn, frameOff, *frameTempUp, *frameTempDown);
    if (best.score < AC_PROBE_STRONG_OVERRIDE_CONF && !(best.libraryAuto && nativeFour))
      return false;
  }

  return true;
}

bool probeTempTrusted(const ProbeBest &best, const AcIrFrame &frameOn,
                      const AcIrFrame &frameOff, const AcIrFrame *frameTempUp,
                      const AcIrFrame *frameTempDown)
{
  if (!best.hasTempEvidence)
    return false;
  if (validateTempRise(best.stateOn, best.stateTempUp) &&
      validateTempDrop(best.stateTempUp, best.stateTempDown))
    return true;
  if (best.libraryAuto && frameTempUp && frameTempDown &&
      nativeAllFourMatch(best.vendor, frameOn, frameOff, *frameTempUp, *frameTempDown) &&
      validateTempRoundTrip(best.stateOn, best.stateTempDown))
    return true;
  if (best.libraryAuto && best.model >= 0 && !acProtocolIsUntrusted(best.vendor))
    return true;
  if (nativeDecodeMatches(best.vendor, frameOn, frameOff) && best.model >= 0)
    return true;
  return false;
}

uint8_t scoreNativeConsensusProbe(decode_type_t vendor, bool libraryAuto,
                                  const stdAc::state_t &stateOn, const stdAc::state_t &stateOff,
                                  const stdAc::state_t *stateTempUp,
                                  const stdAc::state_t *stateTempDown)
{
  if (!validateSaneState(stateOn) || !validateSaneState(stateOff))
    return 0;
  if (!validatePowerPair(stateOn, stateOff) || !validateOnOffOrientation(stateOn, stateOff))
    return 0;

  uint8_t score = 68;
  if (libraryAuto)
    score += 10;
  if (stateOn.model >= 0)
    score += 6;

  if (stateTempUp && stateTempDown)
  {
    if (!validateSaneState(*stateTempUp) || !validateSaneState(*stateTempDown))
      return score;

    if (validateTempRise(stateOn, *stateTempUp))
      score += 12;
    if (validateTempDrop(*stateTempUp, *stateTempDown))
      score += 12;
    if (validateTempRoundTrip(stateOn, *stateTempDown))
      score += 8;
    if (validateStableCoolingState(stateOn, *stateTempUp) &&
        validateStableCoolingState(stateOn, *stateTempDown))
      score += 6;
  }

  if (isUntrustedProtocol(vendor))
    score = (score > 14) ? (uint8_t)(score - 14) : 0;

  return score;
}

void considerNativeConsensus(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                             const AcIrFrame &frameTempUp, const AcIrFrame &frameTempDown,
                             decode_type_t autoVendor, const stdAc::state_t &autoOn,
                             const stdAc::state_t &autoOff, int16_t autoModel, ProbeBest *best,
                             uint8_t *runnerUpScore)
{
  if (!best || autoVendor == UNKNOWN)
    return;
  if (!nativeAllFourMatch(autoVendor, frameOn, frameOff, frameTempUp, frameTempDown))
    return;
  if (!validatePowerPair(autoOn, autoOff) || !validateOnOffOrientation(autoOn, autoOff))
    return;

  stdAc::state_t tempUp = {};
  stdAc::state_t tempDown = {};
  const bool hasUp = tryDecodeTempFrame(frameTempUp, autoOn, nullptr, &tempUp);
  const bool hasDown =
      hasUp ? tryDecodeTempFrame(frameTempDown, autoOn, &tempUp, &tempDown)
            : tryDecodeTempFrame(frameTempDown, autoOn, nullptr, &tempDown);

  const stdAc::state_t *upPtr = hasUp ? &tempUp : nullptr;
  const stdAc::state_t *downPtr = hasDown ? &tempDown : nullptr;
  const uint8_t score =
      scoreNativeConsensusProbe(autoVendor, true, autoOn, autoOff, upPtr, downPtr);
  considerCandidate(best, runnerUpScore, score, autoVendor, true, autoOn, autoOff, upPtr, downPtr,
                    autoModel);

  if (ENABLE_DEBUG && score >= AC_PROBE_MIN_CONFIDENCE)
  {
    Serial.print(F("[PROBE] native consensus vendor="));
    Serial.print(acProtocolName(autoVendor));
    Serial.print(F(" conf="));
    Serial.print(score);
    Serial.print(F("% tempDecoded="));
    Serial.println((hasUp && hasDown) ? 1 : 0);
  }
}

} // namespace

bool acProtocolIsValidVendor(decode_type_t vendor)
{
  const int raw = static_cast<int>(vendor);
  if (vendor == UNKNOWN || raw < 0 || raw > static_cast<int>(kLastDecodeType))
    return false;
  return IRac::isProtocolSupported(vendor);
}

decode_type_t acProtocolConsensusFromFrames(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                                              const AcIrFrame &frameTempUp,
                                              const AcIrFrame &frameTempDown,
                                              uint8_t *outVoteCount)
{
  const decode_type_t types[4] = {frameOn.decodeType, frameOff.decodeType, frameTempUp.decodeType,
                                  frameTempDown.decodeType};
  decode_type_t bestVendor = UNKNOWN;
  uint8_t bestCount = 0;
  uint8_t validCount = 0;

  for (int i = 0; i < 4; i++)
  {
    const decode_type_t candidate = types[i];
    if (!acProtocolIsValidVendor(candidate))
      continue;

    validCount++;
    uint8_t count = 0;
    for (int j = 0; j < 4; j++)
    {
      if (types[j] == candidate)
        count++;
    }
    if (count > bestCount)
    {
      bestCount = count;
      bestVendor = candidate;
    }
  }

  if (outVoteCount)
    *outVoteCount = bestCount;

  if (bestCount >= 3)
    return bestVendor;
  if (bestCount >= 2)
    return bestVendor;
  // ponytail: single tagged frame when IRrecv marks rest UNKNOWN (common on MIRAGE)
  if (bestCount == 1 && validCount == 1)
    return bestVendor;
  return UNKNOWN;
}

bool acProtocolConsensusSelfCheck()
{
  AcIrFrame on = {};
  AcIrFrame off = {};
  AcIrFrame up = {};
  AcIrFrame down = {};
  on.decodeType = MIRAGE;
  off.decodeType = MIRAGE;
  up.decodeType = MIRAGE;
  down.decodeType = FUJITSU_AC;
  uint8_t votes = 0;
  if (acProtocolConsensusFromFrames(on, off, up, down, &votes) != MIRAGE || votes != 3)
    return false;

  down.decodeType = MIRAGE;
  if (acProtocolConsensusFromFrames(on, off, up, down, &votes) != MIRAGE || votes != 4)
    return false;

  off.decodeType = UNKNOWN;
  up.decodeType = UNKNOWN;
  down.decodeType = UNKNOWN;
  if (acProtocolConsensusFromFrames(on, off, up, down, &votes) != UNKNOWN)
    return false;

  off.decodeType = MIRAGE;
  if (acProtocolConsensusFromFrames(on, off, up, down, &votes) != MIRAGE || votes != 1)
    return false;

  return true;
}

bool acProtocolIsUntrusted(decode_type_t vendor)
{
  switch (vendor)
  {
  case MIRAGE:
  case AIWA_RC_T501:
  case GICABLE:
  case MULTIBRACKETS:
    return true;
  default:
    return false;
  }
}

const char *acProtocolName(decode_type_t vendor)
{
  static char nameBuf[48];
  const int vendorRaw = static_cast<int>(vendor);
  if (vendorRaw < 0 || vendorRaw > static_cast<int>(kLastDecodeType))
  {
    snprintf(nameBuf, sizeof(nameBuf), "INVALID_%d", vendorRaw);
    return nameBuf;
  }

  const String name = typeToString(vendor, false);
  if (name.length() == 0)
  {
    snprintf(nameBuf, sizeof(nameBuf), "TYPE_%d", vendorRaw);
    return nameBuf;
  }

  name.toCharArray(nameBuf, sizeof(nameBuf));
  return nameBuf;
}

AcProbeResult acProtocolProbe(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                              const AcIrFrame *frameTempUp, const AcIrFrame *frameTempDown)
{
  AcProbeResult result = {};
  if (!frameTempUp || !frameTempDown)
    return result;

  const uint8_t rawSim = acIrFrameSimilarity(frameOn, frameOff);
  if (rawSim < 40)
  {
    result.confidence = rawSim / 2;
    return result;
  }

  ProbeBest best = {};
  uint8_t runnerUpScore = 0;

  stdAc::state_t autoA = {};
  stdAc::state_t autoB = {};
  decode_type_t autoVendor = UNKNOWN;
  int16_t autoModel = -1;
  if (tryLibraryAutoPair(frameOn, frameOff, &autoA, &autoB, &autoVendor, &autoModel))
  {
    if (frameTempUp && frameTempDown)
    {
      stdAc::state_t autoUp = {};
      stdAc::state_t autoDown = {};
      int16_t seqModel = autoModel;
      const bool hasUp = tryDecodeTempFrame(*frameTempUp, autoA, nullptr, &autoUp);
      const bool hasDown =
          hasUp ? tryDecodeTempFrame(*frameTempDown, autoA, &autoUp, &autoDown)
                : tryDecodeTempFrame(*frameTempDown, autoA, nullptr, &autoDown);
      if (hasUp && hasDown)
      {
        const uint8_t score = scoreFourStepProbe(frameOn, frameOff, autoVendor, true, autoA, autoB,
                                                 autoUp, autoDown);
        considerCandidate(&best, &runnerUpScore, score, autoVendor, true, autoA, autoB, &autoUp,
                          &autoDown, seqModel);
      }
      considerNativeConsensus(frameOn, frameOff, *frameTempUp, *frameTempDown, autoVendor, autoA,
                              autoB, autoModel, &best, &runnerUpScore);
    }
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PROBE] library auto vendor="));
      Serial.println(acProtocolName(autoVendor));
    }
  }

  scanAllIracProtocols(frameOn, frameOff, frameTempUp, frameTempDown, &best, &runnerUpScore);

  if (acceptSemanticProbe(best, runnerUpScore, autoVendor, frameOn, frameOff, frameTempUp,
                          frameTempDown))
  {
    result.ok = true;
    result.mode = AcControlMode::kSemantic;
    result.vendor = best.vendor;
    result.model = best.model;
    result.confidence = best.score;
    result.runnerUpConfidence = runnerUpScore;
    result.stateOn = best.stateOn;
    result.stateOff = best.stateOff;
    result.hasSemantic = true;
    result.probeFromScan = !best.libraryAuto;
    result.tempTrusted =
        probeTempTrusted(best, frameOn, frameOff, frameTempUp, frameTempDown);

    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PROBE] selected "));
      Serial.print(acProtocolName(best.vendor));
      Serial.print(F(" model="));
      Serial.print(best.model);
      Serial.print(F(" conf="));
      Serial.print(best.score);
      Serial.print(F("% runnerUp="));
      Serial.print(runnerUpScore);
      Serial.print(F("% src="));
      Serial.print(best.libraryAuto ? F("IRrecv+IRAcUtils") : F("IRac-scan"));
      Serial.print(F(" tempTrusted="));
      Serial.println(result.tempTrusted ? 1 : 0);
    }
    return result;
  }

  if (best.score > 0 && ENABLE_DEBUG)
  {
    Serial.print(F("[PROBE] reject semantic vendor="));
    Serial.print(acProtocolName(best.vendor));
    Serial.print(F(" score="));
    Serial.print(best.score);
    Serial.print(F(" runnerUp="));
    Serial.print(runnerUpScore);
    Serial.print(F(" auto="));
    Serial.println(acProtocolName(autoVendor));
  }

  const uint16_t diffCells = acIrFrameDiffCells(frameOn, frameOff, AC_IR_MATCH_TOLERANCE_US);
  if (rawSim >= 55 && diffCells >= 4 && diffCells <= frameOn.rawLen / 2)
  {
    result.ok = true;
    result.mode = AcControlMode::kRawFallback;
    const int conf = (int)rawSim + (int)(diffCells / 4);
    result.confidence = (uint8_t)(conf > 90 ? 90 : conf);
    result.hasSemantic = false;

    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PROBE] raw fallback conf="));
      Serial.print(result.confidence);
      Serial.print(F("% bestSemantic="));
      Serial.println(best.score);
    }
    return result;
  }

  result.confidence = best.score;
  if (ENABLE_DEBUG)
  {
    Serial.print(F("[PROBE] failed bestScore="));
    Serial.print(best.score);
    Serial.print(F(" vendor="));
    Serial.println(acProtocolName(best.vendor));
  }
  return result;
}
