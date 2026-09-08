#include "ac_semantic_driver.h"

#include "ac_ir_frame.h"
#include "ac_protocol_probe.h"
#include "config.h"
#include "ir_feedback.h"
#include "ir_transport.h"

#include <IRac.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <math.h>

AcSemanticDriver acSemanticDriver;

namespace {

void clearBlockedInfo(AcBlockedRxInfo *info)
{
  if (!info)
    return;
  *info = {};
}

bool validateSaneState(const stdAc::state_t &state)
{
  if (!state.power)
    return true;
  return state.degrees >= 16.0f && state.degrees <= 32.0f;
}

bool isDefaultLookingActiveState(const stdAc::state_t &state)
{
  return state.power && state.degrees == 16.0f && state.mode == stdAc::opmode_t::kAuto &&
         state.fanspeed == stdAc::fanspeed_t::kAuto;
}

bool tryDecodeSingle(const AcIrFrame &frame, decode_type_t vendor, const stdAc::state_t *prev,
                     stdAc::state_t *outState)
{
  if (!outState || vendor == UNKNOWN || !IRac::isProtocolSupported(vendor))
    return false;

  uint16_t rawBuf[AC_IR_MAX_RAW_LEN + 1];
  decode_results results = {};
  acIrFrameToDecodeResults(frame, vendor, &results, rawBuf, (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  return IRAcUtils::decodeToState(&results, outState, prev) &&
         results.decode_type == vendor;
}

uint8_t scoreRxState(decode_type_t vendor, const stdAc::state_t &state, const AcIrFrame &frame,
                     const AcDeviceProfile &profile, uint8_t anchorScore)
{
  if (!validateSaneState(state))
    return 0;

  uint8_t score = 25;
  if (vendor == profile.vendor)
    score += 25;
  if (frame.decodeType == vendor)
    score += 20;
  if (state.model >= 0)
    score += 8;
  if (acProtocolIsUntrusted(vendor))
    score = (score > 15) ? (uint8_t)(score - 15) : 0;

  if (anchorScore > 0)
    score = (uint8_t)(score + (anchorScore / 4));

  if (profile.hasLastState)
  {
    if (state.power == profile.lastState.power)
      score += 6;
    if (state.power && fabsf(state.degrees - profile.lastState.degrees) >= 0.5f)
      score += 18;
    else if (!state.power && !profile.lastState.power)
      score += 4;

    if (state.power && profile.lastState.power && isDefaultLookingActiveState(state) &&
        anchorScore < AC_RX_RAW_ANCHOR_MIN)
    {
      if (fabsf(state.degrees - profile.lastState.degrees) > 1.0f ||
          state.mode != profile.lastState.mode || state.fanspeed != profile.lastState.fanspeed)
        score = (score > 20) ? (uint8_t)(score - 20) : 0;
    }
  }

  return score;
}

void considerRxCandidate(uint8_t score, decode_type_t vendor, const stdAc::state_t &state,
                         uint8_t minRx, uint8_t *bestScore, decode_type_t *bestVendor,
                         stdAc::state_t *bestState, uint8_t *runnerUpScore,
                         decode_type_t *runnerUpVendor)
{
  if (score < minRx || !bestScore || !bestVendor || !bestState)
    return;

  if (score > *bestScore)
  {
    if (runnerUpScore && runnerUpVendor && *bestScore >= minRx && *bestVendor != UNKNOWN &&
        *bestVendor != vendor)
    {
      *runnerUpScore = *bestScore;
      *runnerUpVendor = *bestVendor;
    }
    *bestScore = score;
    *bestVendor = vendor;
    *bestState = state;
    return;
  }

  if (runnerUpScore && runnerUpVendor && score >= minRx && vendor != *bestVendor &&
      score > *runnerUpScore)
  {
    *runnerUpScore = score;
    *runnerUpVendor = vendor;
  }
}

void collectRxCandidates(const AcIrFrame &frame, const AcDeviceProfile &profile,
                         const stdAc::state_t *prev, uint8_t anchorScore, uint8_t minRx,
                         decode_type_t lockedVendor, uint8_t *bestScore, decode_type_t *bestVendor,
                         stdAc::state_t *bestState, uint8_t *runnerUpScore,
                         decode_type_t *runnerUpVendor)
{
  uint16_t rawBuf[AC_IR_MAX_RAW_LEN + 1];
  decode_results results = {};
  acIrFrameToDecodeResults(frame, UNKNOWN, &results, rawBuf,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  stdAc::state_t nativeState = {};
  if (IRAcUtils::decodeToState(&results, &nativeState, prev))
  {
    const decode_type_t vendor = results.decode_type;
    if (lockedVendor == UNKNOWN || vendor == lockedVendor)
    {
      const uint8_t score = scoreRxState(vendor, nativeState, frame, profile, anchorScore);
      considerRxCandidate(score, vendor, nativeState, minRx, bestScore, bestVendor, bestState,
                          runnerUpScore, runnerUpVendor);
    }
  }

  if (lockedVendor != UNKNOWN)
  {
    stdAc::state_t state = {};
    if (tryDecodeSingle(frame, lockedVendor, prev, &state))
    {
      const uint8_t score = scoreRxState(lockedVendor, state, frame, profile, anchorScore);
      considerRxCandidate(score, lockedVendor, state, minRx, bestScore, bestVendor, bestState,
                          runnerUpScore, runnerUpVendor);
    }
    return;
  }

  for (int raw = 1; raw <= (int)kLastDecodeType; raw++)
  {
    const decode_type_t vendor = (decode_type_t)raw;
    if (!IRac::isProtocolSupported(vendor))
      continue;

    stdAc::state_t state = {};
    if (!tryDecodeSingle(frame, vendor, prev, &state))
      continue;

    const uint8_t score = scoreRxState(vendor, state, frame, profile, anchorScore);
    considerRxCandidate(score, vendor, state, minRx, bestScore, bestVendor, bestState,
                      runnerUpScore, runnerUpVendor);
  }
}

} // namespace

void AcSemanticDriver::begin(uint16_t txPin)
{
  (void)txPin;
  ready_ = true;
}

bool AcSemanticDriver::sendState(const AcDeviceProfile &profile, const stdAc::state_t &desired)
{
  if (!ready_ || profile.mode != AcControlMode::kSemantic || profile.vendor == UNKNOWN)
    return false;

  const stdAc::state_t *prev = profile.hasLastState ? &profile.lastState : nullptr;
  irTransport.disableRx();
  const bool ok = ac_.sendAc(desired, prev);
  if (ok)
  {
    irTransport.muteRxAfterTx(AC_IR_TX_RX_MUTE_MS);
    irFeedback.onCommandTx();
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[TX] IRac "));
      Serial.print(acProtocolName(profile.vendor));
      Serial.print(F(" model="));
      Serial.print(profile.model);
      Serial.print(F(" power="));
      Serial.print(desired.power ? 1 : 0);
      Serial.print(F(" temp="));
      Serial.println(desired.degrees);
    }
  }
  else
  {
    irTransport.enableRx();
  }
  return ok;
}

bool AcSemanticDriver::decodeFrame(const AcIrFrame &frame, const AcDeviceProfile &profile,
                                   stdAc::state_t *outState)
{
  if (!outState || profile.vendor == UNKNOWN)
    return false;

  uint16_t rawBuf[AC_IR_MAX_RAW_LEN + 1];
  decode_results results = {};
  const stdAc::state_t *prev = profile.hasLastState ? &profile.lastState : nullptr;

  acIrFrameToDecodeResults(frame, UNKNOWN, &results, rawBuf,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  if (IRAcUtils::decodeToState(&results, outState, prev) &&
      results.decode_type == profile.vendor)
    return true;
  if (IRAcUtils::decodeToState(&results, outState, nullptr) &&
      results.decode_type == profile.vendor)
    return true;

  acIrFrameToDecodeResults(frame, profile.vendor, &results, rawBuf,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  if (IRAcUtils::decodeToState(&results, outState, prev) &&
      results.decode_type == profile.vendor)
    return true;
  return IRAcUtils::decodeToState(&results, outState, nullptr) &&
         results.decode_type == profile.vendor;
}

bool AcSemanticDriver::decodeFrameVendorStateless(const AcIrFrame &frame, decode_type_t vendor,
                                                  stdAc::state_t *outState)
{
  if (!outState || vendor == UNKNOWN)
    return false;

  uint16_t rawBuf[AC_IR_MAX_RAW_LEN + 1];
  decode_results results = {};
  acIrFrameToDecodeResults(frame, vendor, &results, rawBuf,
                           (uint16_t)(AC_IR_MAX_RAW_LEN + 1));
  return IRAcUtils::decodeToState(&results, outState, nullptr) &&
         results.decode_type == vendor;
}

bool AcSemanticDriver::decodeFrameLocked(const AcIrFrame &frame, const AcDeviceProfile &profile,
                                         stdAc::state_t *outState, uint8_t anchorScore,
                                         uint8_t *outScore)
{
  if (!outState || profile.vendor == UNKNOWN)
    return false;
  if (!decodeFrame(frame, profile, outState))
    return false;

  const uint8_t score = scoreRxState(profile.vendor, *outState, frame, profile, anchorScore);
  if (outScore)
    *outScore = score;
  // Tag lock: successful forced decode for paired vendor is enough.
  return true;
}

bool AcSemanticDriver::passesRxPlausibility(const stdAc::state_t &state,
                                            const AcDeviceProfile &profile,
                                            uint8_t anchorScore) const
{
  (void)profile;
  (void)anchorScore;
  return validateSaneState(state);
}

bool AcSemanticDriver::decodeFrameBest(const AcIrFrame &frame, const AcDeviceProfile &profile,
                                       uint8_t anchorScore, stdAc::state_t *outState,
                                       decode_type_t *outVendor, AcBlockedRxInfo *outBlockedInfo,
                                       uint8_t *outBestScore, uint8_t *outRunnerUpScore,
                                       AcRxRejectReason *outReject)
{
  if (!outState)
    return false;
  clearBlockedInfo(outBlockedInfo);
  if (outReject)
    *outReject = AcRxRejectReason::kNone;

  const stdAc::state_t *prev = profile.hasLastState ? &profile.lastState : nullptr;
  const uint8_t minRx = profile.mode == AcControlMode::kRawFallback ? AC_RX_DECODE_MIN_SCORE_RAW
                                                                    : AC_RX_DECODE_MIN_SCORE;
  uint8_t bestScore = 0;
  uint8_t runnerUpScore = 0;
  decode_type_t bestVendor = UNKNOWN;
  decode_type_t runnerUpVendor = UNKNOWN;
  stdAc::state_t bestState = {};

  const decode_type_t lockedVendor = profile.vendor;
  collectRxCandidates(frame, profile, prev, anchorScore, minRx, lockedVendor, &bestScore,
                      &bestVendor, &bestState, &runnerUpScore, &runnerUpVendor);

  if (outBestScore)
    *outBestScore = bestScore;
  if (outRunnerUpScore)
    *outRunnerUpScore = runnerUpScore;

  if (bestScore < minRx)
  {
    if (outReject)
      *outReject = AcRxRejectReason::kLowScore;
    return false;
  }

  if (runnerUpVendor != UNKNOWN && bestVendor != UNKNOWN && runnerUpVendor != bestVendor &&
      runnerUpScore > 0 && bestScore < (uint8_t)(runnerUpScore + AC_RX_WINNER_MARGIN))
  {
    if (outReject)
      *outReject = AcRxRejectReason::kWinnerMargin;
    return false;
  }

#if AC_RX_ENFORCE_PAIRED_VENDOR
  if (profile.vendor != UNKNOWN && bestVendor != UNKNOWN && bestVendor != profile.vendor)
  {
    if (outBlockedInfo)
    {
      outBlockedInfo->blocked = true;
      outBlockedInfo->expectedVendor = profile.vendor;
      outBlockedInfo->blockedVendor = bestVendor;
      outBlockedInfo->frameType = frame.decodeType;
      outBlockedInfo->blockedState = bestState;
      outBlockedInfo->hasBlockedState = true;
    }
    if (outReject)
      *outReject = AcRxRejectReason::kVendorMismatch;
    return false;
  }
#endif

  if (!passesRxPlausibility(bestState, profile, anchorScore))
  {
    if (outReject)
      *outReject = AcRxRejectReason::kPlausibility;
    return false;
  }

  *outState = bestState;
  if (outVendor)
    *outVendor = bestVendor;
  return true;
}
