#include "ac_hybrid_driver.h"

#include "ac_raw_store.h"
#include "ac_semantic_driver.h"
#include "ac_protocol_probe.h"
#include "board_pins.h"
#include "config.h"
#include "ir_transport.h"

#include <string.h>

AcHybridDriver acHybridDriver;

namespace {

void logBlockedRxFrame(const AcBlockedRxInfo &info)
{
  if (!ENABLE_DEBUG || !info.blocked)
    return;

  Serial.println(F("########### BLOCKED RX TAG ###########"));
  Serial.println(F("[RX] blocked frame"));
  Serial.print(F("reason=vendor_mismatch expectedTag="));
  Serial.print(acProtocolName(info.expectedVendor));
  Serial.print(F(" blockedTag="));
  Serial.println(acProtocolName(info.blockedVendor));
  Serial.print(F("frameType="));
  Serial.println(acProtocolName(info.frameType));
  if (info.hasBlockedState)
  {
    Serial.print(F("data power="));
    Serial.print(info.blockedState.power ? 1 : 0);
    Serial.print(F(" temp="));
    Serial.print(info.blockedState.degrees);
    Serial.print(F(" mode="));
    Serial.print((int)info.blockedState.mode);
    Serial.print(F(" fan="));
    Serial.println((int)info.blockedState.fanspeed);
  }
  Serial.println(F("########### BLOCKED RX TAG ###########"));
}

void logTxPower(bool on, uint16_t samples)
{
  if (!ENABLE_DEBUG)
    return;
  Serial.print(F("[TX] IR raw "));
  Serial.print(on ? F("pw_on") : F("pw_of"));
  Serial.print(F(" samples="));
  Serial.println(samples);
}

void logRxReject(AcRxRejectReason reason, uint8_t anchor, uint8_t bestScore, uint8_t runnerUp)
{
  if (!ENABLE_DEBUG || reason == AcRxRejectReason::kNone)
    return;

  Serial.print(F("[RX] semantic reject reason="));
  switch (reason)
  {
  case AcRxRejectReason::kLowScore:
    Serial.print(F("low_score"));
    break;
  case AcRxRejectReason::kVendorMismatch:
    Serial.print(F("vendor_mismatch"));
    break;
  case AcRxRejectReason::kWinnerMargin:
    Serial.print(F("winner_margin"));
    break;
  case AcRxRejectReason::kPlausibility:
    Serial.print(F("plausibility"));
    break;
  default:
    Serial.print(F("unknown"));
    break;
  }
  Serial.print(F(" anchor="));
  Serial.print(anchor);
  Serial.print(F(" best="));
  Serial.print(bestScore);
  Serial.print(F(" runnerUp="));
  Serial.println(runnerUp);
}

bool applyDecodedRemoteState(const AcIrFrame &frame, const AcDeviceProfile &profile,
                             decode_type_t rxVendor, const stdAc::state_t &state,
                             AcClimateState *outClimate, const char **outRawSlot);

bool trySemanticRemoteDecode(const AcIrFrame &frame, const AcDeviceProfile &profile, uint8_t anchor,
                             AcClimateState *outClimate, const char **outRawSlot)
{
  stdAc::state_t state = {};
  uint8_t lockedScore = 0;

  if (profile.vendor != UNKNOWN)
  {
    if ((acSemanticDriver.decodeFrameLocked(frame, profile, &state, anchor, &lockedScore) ||
         acSemanticDriver.decodeFrameVendorStateless(frame, profile.vendor, &state)) &&
        acSemanticDriver.passesRxPlausibility(state, profile, anchor) &&
        applyDecodedRemoteState(frame, profile, profile.vendor, state, outClimate, outRawSlot))
    {
      if (ENABLE_DEBUG)
      {
        Serial.print(F("[RX] locked tag="));
        Serial.print(acProtocolName(profile.vendor));
        Serial.print(F(" score="));
        Serial.print(lockedScore);
        Serial.print(F(" anchor="));
        Serial.println(anchor);
      }
      return true;
    }
    return false;
  }

  decode_type_t rxVendor = UNKNOWN;
  AcBlockedRxInfo blockedInfo = {};
  uint8_t bestScore = 0;
  uint8_t runnerUpScore = 0;
  AcRxRejectReason reject = AcRxRejectReason::kNone;
  if (acSemanticDriver.decodeFrameBest(frame, profile, anchor, &state, &rxVendor, &blockedInfo,
                                       &bestScore, &runnerUpScore, &reject))
  {
    if (applyDecodedRemoteState(frame, profile, rxVendor, state, outClimate, outRawSlot))
    {
      if (ENABLE_DEBUG)
      {
        Serial.print(F("[RX] scan vendor score="));
        Serial.print(bestScore);
        Serial.print(F(" anchor="));
        Serial.println(anchor);
      }
      return true;
    }
  }

  if (blockedInfo.blocked)
  {
    logBlockedRxFrame(blockedInfo);
    return false;
  }

  logRxReject(reject, anchor, bestScore, runnerUpScore);
  return false;
}

void applySlotClimate(const char *slot)
{
  if (!slot)
    return;
  if (strcmp(slot, "pw_on") == 0)
    acStateModel.setPower(true);
  else if (strcmp(slot, "pw_of") == 0)
    acStateModel.setPower(false);
  else if (strncmp(slot, "cool_", 5) == 0)
  {
    acStateModel.setPower(true);
    acStateModel.setTempC((float)atoi(slot + 5));
    acStateModel.setMode(stdAc::opmode_t::kCool);
  }
}

bool matchPairedPower(const AcIrFrame &frame, const AcDeviceProfile &profile, bool *outPower,
                      uint8_t *outScore, bool *outOnFamily)
{
  if (!outPower)
    return false;

  uint8_t scoreOn = 0;
  uint8_t scoreOff = 0;
  if (profile.hasFrameOn)
    scoreOn = acIrFrameSyncScore(frame, profile.frameOn);
  if (profile.hasFrameOff)
    scoreOff = acIrFrameSyncScore(frame, profile.frameOff);

  const bool onStrong = scoreOn >= AC_IR_SYNC_MIN_SCORE;
  const bool offStrong = scoreOff >= AC_IR_SYNC_MIN_SCORE;
  const bool onWeak = scoreOn >= 52 && scoreOn > scoreOff + 4;
  const bool onMed = scoreOn >= 52;
  const bool offMed = scoreOff >= 52;

  if (outOnFamily)
    *outOnFamily = onStrong || onWeak || (onMed && offMed);

  if (!onStrong && !offStrong && !onWeak)
  {
    if (onMed && offMed)
    {
      const int delta = (int)scoreOn - (int)scoreOff;
      if (delta >= -AC_IR_POWER_MATCH_MIN_DELTA && delta <= AC_IR_POWER_MATCH_MIN_DELTA)
      {
        *outPower = true;
        if (outScore)
          *outScore = scoreOn > scoreOff ? scoreOn : scoreOff;
        return true;
      }
    }

    if (ENABLE_DEBUG)
    {
      Serial.print(F("[RX] sync scores on="));
      Serial.print(scoreOn);
      Serial.print(F(" off="));
      Serial.println(scoreOff);
    }
    return false;
  }

  if (onStrong && offStrong)
  {
    const int delta = (int)scoreOn - (int)scoreOff;
    if (delta >= -AC_IR_POWER_MATCH_MIN_DELTA && delta <= AC_IR_POWER_MATCH_MIN_DELTA)
      return false;
    *outPower = scoreOn > scoreOff;
    if (outScore)
      *outScore = scoreOn > scoreOff ? scoreOn : scoreOff;
    return true;
  }

  if (offStrong && scoreOff >= scoreOn + AC_IR_POWER_MATCH_MIN_DELTA)
  {
    *outPower = false;
    if (outScore)
      *outScore = scoreOff;
    return true;
  }

  if (onStrong || onWeak)
  {
    *outPower = true;
    if (outScore)
      *outScore = scoreOn;
    return true;
  }

  *outPower = false;
  if (outScore)
    *outScore = scoreOff;
  return true;
}

void passiveLearnTempSlot(const AcIrFrame &frame, const stdAc::state_t &state)
{
  if (!state.power || state.degrees < AC_TEMP_MIN || state.degrees > AC_TEMP_MAX)
    return;

  char slot[16];
  snprintf(slot, sizeof(slot), "cool_%d", (int)state.degrees);
  if (acRawStoreExists(slot))
    return;

  if (acRawStoreSave(slot, frame) && ENABLE_DEBUG)
  {
    Serial.print(F("[LEARN] auto "));
    Serial.println(slot);
  }
}

bool applyDecodedRemoteState(const AcIrFrame &frame, const AcDeviceProfile &profile,
                             decode_type_t rxVendor, const stdAc::state_t &state,
                             AcClimateState *outClimate, const char **outRawSlot)
{
  AcClimateState climate = {};
  if (!acClimateFromDecodedState(state, &climate))
    return false;

  if (!acProfileTempTrusted(profile))
    passiveLearnTempSlot(frame, state);

  AcDeviceProfile mutableProfile = profile;
  mutableProfile.lastState = state;
  mutableProfile.hasLastState = true;
  acProfileStore.save(mutableProfile);
  if (outClimate)
    *outClimate = climate;
  if (outRawSlot)
    *outRawSlot = nullptr;
  if (ENABLE_DEBUG)
  {
    Serial.print(F("[RX] IRac "));
    Serial.print(acProtocolName(rxVendor));
    Serial.print(F(" power="));
    Serial.print(state.power ? 1 : 0);
    Serial.print(F(" temp="));
    Serial.print(state.degrees);
    Serial.print(F(" mode="));
    Serial.print((int)state.mode);
    Serial.print(F(" fan="));
    Serial.print((int)state.fanspeed);
    Serial.print(F(" swingv="));
    Serial.println((int)state.swingv);
  }
  return true;
}

void reprobeStoredProfile(AcDeviceProfile *profile)
{
  if (!profile || !profile->hasFrameOn || !profile->hasFrameOff)
    return;

  const AcIrFrame *tempUp = profile->hasFrameTempUp ? &profile->frameTempUp : nullptr;
  const AcIrFrame *tempDown = profile->hasFrameTempDown ? &profile->frameTempDown : nullptr;
  const AcProbeResult probe =
      acProtocolProbe(profile->frameOn, profile->frameOff, tempUp, tempDown);
  if (!probe.ok)
    return;

  if (probe.mode == AcControlMode::kSemantic)
  {
    profile->mode = AcControlMode::kSemantic;
    profile->vendor = probe.vendor;
    profile->model = probe.model;
    profile->probeConfidence = probe.confidence;
    profile->lastState = probe.stateOn;
    profile->hasLastState = true;
    profile->probeFromScan = probe.probeFromScan;
    profile->tempTrusted = probe.tempTrusted;
    acProfileStore.save(*profile);
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PROFILE] reprobe -> "));
      Serial.println(acProtocolName(profile->vendor));
    }
    return;
  }

  if (probe.mode == AcControlMode::kRawFallback)
  {
    profile->mode = AcControlMode::kRawFallback;
    profile->probeConfidence = probe.confidence;
    profile->hasLastState = false;
    acProfileStore.save(*profile);
    acRawStoreSave("pw_on", profile->frameOn);
    acRawStoreSave("pw_of", profile->frameOff);
    if (ENABLE_DEBUG)
      Serial.println(F("[PROFILE] reprobe -> raw fallback"));
  }
}

} // namespace

void AcHybridDriver::begin()
{
  acProfileStore.ensureLoaded();
  AcDeviceProfile &profile = acProfileStore.mutableCached();
  if (profile.mode == AcControlMode::kRawFallback && profile.vendor == UNKNOWN &&
      profile.hasFrameOn && profile.hasFrameOff && profile.hasFrameTempUp &&
      profile.hasFrameTempDown)
  {
    profile.vendor = acProtocolConsensusFromFrames(profile.frameOn, profile.frameOff,
                                                   profile.frameTempUp, profile.frameTempDown,
                                                   nullptr);
    if (profile.vendor != UNKNOWN)
      acProfileStore.save(profile);
  }
  if (profile.mode == AcControlMode::kSemantic && profile.probeFromScan && !profile.tempTrusted)
    reprobeStoredProfile(&profile);
  else if (profile.mode == AcControlMode::kSemantic && acProtocolIsUntrusted(profile.vendor))
    reprobeStoredProfile(&profile);
  profileLoaded_ = acProfileStore.isPaired();
  acSemanticDriver.begin(IR_TX_PIN);

  if (profile.hasLastState)
    acStateModel.fromStdAc(profile.lastState);
  else if (profile.hasFrameOff)
    acStateModel.setPower(false);
  else if (profile.hasFrameOn)
    acStateModel.setPower(true);
}

bool AcHybridDriver::sendSemantic(const stdAc::state_t &desired)
{
  AcDeviceProfile profile = acProfileStore.cached();
  if (!acProfileSemanticTrusted(profile))
    return false;

  if (!acSemanticDriver.sendState(profile, desired))
    return false;

  profile.lastState = desired;
  profile.hasLastState = true;
  acProfileStore.save(profile);
  acStateModel.fromStdAc(desired);
  return true;
}

bool AcHybridDriver::sendRawFromProfile(bool powerOn)
{
  const AcDeviceProfile &profile = acProfileStore.cached();
  const AcIrFrame *frame = powerOn ? &profile.frameOn : &profile.frameOff;
  const bool hasFrame = powerOn ? profile.hasFrameOn : profile.hasFrameOff;
  if (!hasFrame || !frame || frame->rawLen == 0)
    return false;

  irTransport.sendRaw(frame->raw, frame->rawLen);
  acStateModel.setPower(powerOn);
  logTxPower(powerOn, frame->rawLen);
  return true;
}

bool AcHybridDriver::sendRawSlot(const char *name)
{
  AcIrFrame frame = {};
  if (acRawStoreLoad(name, &frame))
  {
    irTransport.sendRaw(frame.raw, frame.rawLen);
    applySlotClimate(name);
    return true;
  }

  const AcDeviceProfile &profile = acProfileStore.cached();
  if (strcmp(name, "pw_on") == 0 && profile.hasFrameOn)
  {
    irTransport.sendRaw(profile.frameOn.raw, profile.frameOn.rawLen);
    acStateModel.setPower(true);
    logTxPower(true, profile.frameOn.rawLen);
    return true;
  }
  if (strcmp(name, "pw_of") == 0 && profile.hasFrameOff)
  {
    irTransport.sendRaw(profile.frameOff.raw, profile.frameOff.rawLen);
    acStateModel.setPower(false);
    logTxPower(false, profile.frameOff.rawLen);
    return true;
  }
  return false;
}

bool AcHybridDriver::sendPower(bool on)
{
  const AcDeviceProfile &profile = acProfileStore.cached();

  if (acProfileSemanticTrusted(profile) && acProfileTempTrusted(profile))
  {
    AcClimateState target = acStateModel.climate();
    target.power = on;
    if (applyClimate(target))
      return true;
    if (ENABLE_DEBUG)
      Serial.println(F("[TX] semantic power failed — trying raw pair frames"));
  }

  if (on && profile.hasFrameOn)
    return sendRawFromProfile(true);
  if (!on && profile.hasFrameOff)
    return sendRawFromProfile(false);

  if (acProfileSemanticTrusted(profile))
  {
    AcClimateState target = acStateModel.climate();
    target.power = on;
    AcStateModel model;
    model.setMode(target.mode);
    model.setTempC(target.tempC);
    model.setFan(target.fan);
    model.setSwingV(target.swingV);
    model.setPower(on);
    if (sendSemantic(model.toStdAc(profile)))
      return true;
  }

  return false;
}

bool AcHybridDriver::applyClimate(const AcClimateState &target)
{
  const AcDeviceProfile &profile = acProfileStore.cached();
  const AcClimateState desired = target;

  if (acProfileSemanticTrusted(profile) && acProfileTempTrusted(profile))
  {
    AcStateModel model;
    model.setMode(desired.mode);
    model.setTempC(desired.tempC);
    model.setFan(desired.fan);
    model.setSwingV(desired.swingV);
    model.setPower(desired.power);
    if (sendSemantic(model.toStdAc(profile)))
      return true;
    if (ENABLE_DEBUG)
      Serial.println(F("[TX] semantic climate failed — raw fallback"));
  }

  if (!desired.power && profile.hasFrameOff)
    return sendRawFromProfile(false);

  if (desired.power)
  {
    char slot[16];
    snprintf(slot, sizeof(slot), "cool_%d", (int)desired.tempC);
    if (acRawStoreExists(slot) && sendRawSlot(slot))
    {
      acStateModel.setPower(true);
      acStateModel.setTempC(desired.tempC);
      acStateModel.setMode(desired.mode);
      return true;
    }
  }

  if (!desired.power)
    return sendRawFromProfile(false);

  char slot[16];
  snprintf(slot, sizeof(slot), "cool_%d", (int)desired.tempC);
  if (sendRawSlot(slot))
  {
    acStateModel.setPower(true);
    acStateModel.setTempC(desired.tempC);
    acStateModel.setMode(desired.mode);
    return true;
  }

  if (desired.power && profile.hasFrameOn)
    return sendRawFromProfile(true);

  if (ENABLE_DEBUG)
    Serial.println(F("[TX] climate FAIL — use remote to teach temp slots or re-run pair"));
  return false;
}

bool AcHybridDriver::sendSlotRaw(const char *slotName)
{
  return sendRawSlot(slotName);
}

bool AcHybridDriver::learnRawSlot(const char *name, const AcIrFrame &frame)
{
  return acRawStoreSave(name, frame);
}

bool AcHybridDriver::pollRemoteSync(AcClimateState *outClimate, const char **outRawSlot)
{
  if (!acProfileStore.isPaired())
    return false;

  AcIrFrame frame = {};
  if (!irTransport.pollCapture(&frame))
    return false;

  const AcDeviceProfile &profile = acProfileStore.cached();

  const uint8_t anchor = acIrFrameBestAnchorScore(frame, profile);
  const bool anchorOk = anchor >= AC_RX_RAW_ANCHOR_MIN;
  const bool qualityOk = acIrFramePassesQualityGate(frame);
  const bool trySemantic = qualityOk &&
                           (profile.mode == AcControlMode::kSemantic || anchorOk ||
                            profile.vendor != UNKNOWN);

  if (trySemantic && trySemanticRemoteDecode(frame, profile, anchor, outClimate, outRawSlot))
    return true;

  if (!qualityOk && ENABLE_DEBUG)
  {
    Serial.print(F("[RX] short frame len="));
    Serial.print(frame.rawLen);
    Serial.println(F(" — raw path only"));
  }
  else if (!anchorOk && ENABLE_DEBUG && profile.mode == AcControlMode::kRawFallback &&
           profile.vendor == UNKNOWN)
  {
    Serial.print(F("[RX] skip semantic anchor="));
    Serial.println(anchor);
  }

  bool powerOn = false;
  uint8_t powerScore = 0;
  bool onFamily = false;
  if (matchPairedPower(frame, profile, &powerOn, &powerScore, &onFamily))
  {
    if (qualityOk && onFamily && powerOn &&
        (profile.vendor != UNKNOWN || anchorOk) &&
        trySemanticRemoteDecode(frame, profile, anchor, outClimate, outRawSlot))
      return true;

    acStateModel.setPower(powerOn);
    if (onFamily && powerOn && !acRawStoreExists("pw_on"))
      acRawStoreSave("pw_on", frame);

    if (outRawSlot)
      *outRawSlot = powerOn ? "pw_on" : "pw_of";
    if (outClimate)
      *outClimate = acStateModel.climate();
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[RX] raw power "));
      Serial.print(powerOn ? F("ON") : F("OFF"));
      Serial.print(F(" score="));
      Serial.println(powerScore);
    }
    return true;
  }

  uint8_t score = 0;
  const char *slot = acRawStoreMatchBestSync(frame, &score);
  if (slot)
  {
    applySlotClimate(slot);
    if (outRawSlot)
      *outRawSlot = slot;
    if (outClimate)
      *outClimate = acStateModel.climate();
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[RX] raw slot "));
      Serial.print(slot);
      Serial.print(F(" score="));
      Serial.println(score);
    }
    return true;
  }

  if (ENABLE_DEBUG && profile.vendor == UNKNOWN)
  {
    Serial.print(F("[RX] no match type="));
    Serial.print(acProtocolName(frame.decodeType));
    Serial.print(F(" anchor="));
    Serial.println(anchor);
  }
  return false;
}
