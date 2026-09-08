#include "ac_onboarding.h"

#include "ac_hybrid_driver.h"
#include "ac_mqtt_handler.h"
#include "ac_profile.h"
#include "ac_protocol_probe.h"
#include "ac_raw_store.h"
#include "ac_state_model.h"
#include "config.h"
#include "ir_transport.h"

AcOnboarding acOnboarding;

void AcOnboarding::begin() {}

void AcOnboarding::setPhase(AcOnboardPhase phase)
{
  phase_ = phase;
  phaseStartedMs_ = millis();
}

bool AcOnboarding::start()
{
  if (phase_ == AcOnboardPhase::kPairWarmup || phase_ == AcOnboardPhase::kWaitPowerOn ||
      phase_ == AcOnboardPhase::kWaitPowerOff || phase_ == AcOnboardPhase::kWaitTempUp ||
      phase_ == AcOnboardPhase::kWaitTempDown || phase_ == AcOnboardPhase::kProbing)
    return false;

  if (phase_ == AcOnboardPhase::kFailed)
    phase_ = AcOnboardPhase::kIdle;

  flow_ = AcPairingFlow::kPowerOnOffTempUpTempDown;
  hasFrameOn_ = false;
  hasFrameOff_ = false;
  hasFrameTempUp_ = false;
  hasFrameTempDown_ = false;
  probePending_ = false;
  postPairAction_ = PostPairAction::kNone;
  frameOn_ = {};
  frameOff_ = {};
  frameTempUp_ = {};
  frameTempDown_ = {};
  hasPendingFrame_ = false;
  stepCollectUntilMs_ = 0;
  stepGuardUntilMs_ = 0;
  irTransport.disableRx();
  pairWarmupUntilMs_ = millis() + AC_PAIR_SETTLE_MS;
  setPhase(AcOnboardPhase::kPairWarmup);

  if (ENABLE_DEBUG)
  {
    Serial.print(F("[PAIR] IR settle "));
    Serial.print(AC_PAIR_SETTLE_MS);
    Serial.println(F("ms — do not press remote yet"));
  }
  return true;
}

void AcOnboarding::cancel()
{
  phase_ = AcOnboardPhase::kIdle;
  pairWarmupUntilMs_ = 0;
  hasPendingFrame_ = false;
  stepCollectUntilMs_ = 0;
  stepGuardUntilMs_ = 0;
  hasFrameOn_ = false;
  hasFrameOff_ = false;
  hasFrameTempUp_ = false;
  hasFrameTempDown_ = false;
  postPairAction_ = PostPairAction::kNone;
  if (ENABLE_DEBUG)
    Serial.println(F("[PAIR] cancelled"));
}

bool AcOnboarding::isTimedOut() const
{
  return (millis() - phaseStartedMs_) > AC_PROBE_CAPTURE_TIMEOUT_MS;
}

bool AcOnboarding::frameLooksBetter(const AcIrFrame &candidate, const AcIrFrame &current)
{
  const bool candTagged = acProtocolIsValidVendor(candidate.decodeType);
  const bool curTagged = acProtocolIsValidVendor(current.decodeType);
  if (candTagged != curTagged)
    return candTagged;
  return candidate.rawLen > current.rawLen;
}

/**
 * One remote keypress often lands as several captures: the real full-state frame
 * plus truncated tail fragments (AGC dropouts / burst splits). Collect the whole
 * burst for AC_PAIR_STEP_COLLECT_MS, keep only the best frame, then guard the next
 * step against late fragments. This is what previously corrupted the POWER OFF
 * step (a TEMP DOWN tail fragment was consumed as the OFF frame).
 */
void AcOnboarding::pollStepCapture()
{
  if (hasPendingFrame_ && (int32_t)(millis() - stepCollectUntilMs_) >= 0)
  {
    hasPendingFrame_ = false;
    stepGuardUntilMs_ = millis() + AC_PAIR_STEP_GUARD_MS;
    onFrameCaptured(pendingFrame_);
    irTransport.flushRxBuffer();
    return;
  }

  AcIrFrame frame = {};
  if (!irTransport.pollCapture(&frame))
    return;

  if (frame.repeat)
  {
    if (ENABLE_DEBUG)
      Serial.println(F("[PAIR] ignore repeat frame"));
    return;
  }

  if (!hasPendingFrame_)
  {
    if (stepGuardUntilMs_ != 0 && (int32_t)(millis() - stepGuardUntilMs_) < 0)
    {
      if (ENABLE_DEBUG)
        Serial.println(F("[PAIR] drop tail fragment (step guard)"));
      return;
    }
    pendingFrame_ = frame;
    hasPendingFrame_ = true;
    stepCollectUntilMs_ = millis() + AC_PAIR_STEP_COLLECT_MS;
    return;
  }

  if (frameLooksBetter(frame, pendingFrame_))
  {
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PAIR] burst upgrade type="));
      Serial.print(acProtocolName(frame.decodeType));
      Serial.print(F(" len="));
      Serial.println(frame.rawLen);
    }
    pendingFrame_ = frame;
  }
}

void AcOnboarding::onFrameCaptured(const AcIrFrame &frame)
{
  if (phase_ == AcOnboardPhase::kWaitPowerOn)
  {
    frameOn_ = frame;
    hasFrameOn_ = true;
    setPhase(AcOnboardPhase::kWaitTempUp);
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PAIR] captured POWER ON, type="));
      Serial.println(acProtocolName(frame.decodeType));
      Serial.println(F("[PAIR] Step 2/4: press TEMP UP once"));
    }
    return;
  }

  if (phase_ == AcOnboardPhase::kWaitTempUp)
  {
    frameTempUp_ = frame;
    hasFrameTempUp_ = true;
    setPhase(AcOnboardPhase::kWaitTempDown);
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PAIR] captured TEMP UP, type="));
      Serial.println(acProtocolName(frame.decodeType));
      Serial.println(F("[PAIR] Step 3/4: press TEMP DOWN once"));
    }
    return;
  }

  if (phase_ == AcOnboardPhase::kWaitTempDown)
  {
    frameTempDown_ = frame;
    hasFrameTempDown_ = true;
    setPhase(AcOnboardPhase::kWaitPowerOff);
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PAIR] captured TEMP DOWN, type="));
      Serial.println(acProtocolName(frame.decodeType));
      Serial.println(F("[PAIR] Step 4/4: press POWER OFF"));
    }
    return;
  }

  if (phase_ == AcOnboardPhase::kWaitPowerOff)
  {
    frameOff_ = frame;
    hasFrameOff_ = true;
    setPhase(AcOnboardPhase::kProbing);
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PAIR] captured POWER OFF, type="));
      Serial.println(acProtocolName(frame.decodeType));
    }
    probePending_ = true;
  }
}

void AcOnboarding::tickPostPair()
{
  switch (postPairAction_)
  {
  case PostPairAction::kSaveRawSlots:
    if (!postPairSemantic_)
    {
      acRawStoreSave("pw_on", frameOn_);
      if (hasFrameOff_)
        acRawStoreSave("pw_of", frameOff_);
    }
    postPairAction_ = PostPairAction::kReloadDriver;
    return;

  case PostPairAction::kReloadDriver:
    acHybridDriver.begin();
    postPairAction_ = PostPairAction::kFinalize;
    return;

  case PostPairAction::kFinalize:
    irTransport.flushRxBuffer(); // drop late fragments of the final OFF press
    if (postPairSemantic_)
      acStateModel.fromStdAc(postPairStateOn_);
    else
    {
      acStateModel.setPower(false);
      acStateModel.setTempC(AC_TEMP_DEFAULT);
    }
    acMqttPublishFullState();

    if (ENABLE_DEBUG)
    {
      if (postPairSemantic_)
      {
        Serial.print(F("[PAIR] detected "));
        Serial.print(acProtocolName(postPairVendor_));
        Serial.print(F(" model="));
        Serial.print(postPairModel_);
        Serial.print(F(" conf="));
        Serial.print(postPairConf_);
        if (postPairTempTrusted_)
          Serial.println(F("% — full IRac control"));
        else
          Serial.println(F("% — semantic power OK; temperature not fully trusted"));
      }
      else
      {
        Serial.print(F("[PAIR] raw fallback OK conf="));
        Serial.print(postPairConf_);
        Serial.println(F("% — raw fallback active"));
      }
    }

    postPairAction_ = PostPairAction::kNone;
    setPhase(AcOnboardPhase::kDone);
    return;

  default:
    postPairAction_ = PostPairAction::kNone;
    return;
  }
}

void AcOnboarding::runProbe()
{
  if (!hasFrameOn_)
  {
    setPhase(AcOnboardPhase::kFailed);
    return;
  }

  if (!hasFrameOff_ || !hasFrameTempUp_ || !hasFrameTempDown_)
  {
    setPhase(AcOnboardPhase::kFailed);
    return;
  }

  const AcIrFrame *probeTempUp = hasFrameTempUp_ ? &frameTempUp_ : nullptr;
  const AcIrFrame *probeTempDown = hasFrameTempDown_ ? &frameTempDown_ : nullptr;
  const AcProbeResult probe =
      acProtocolProbe(frameOn_, frameOff_, probeTempUp, probeTempDown);

  AcDeviceProfile &profile = acProfileStore.mutableCached();
  profile = {};
  profile.pairingFlow = flow_;
  profile.frameOn = frameOn_;
  profile.frameOff = frameOff_;
  profile.frameTempUp = frameTempUp_;
  profile.frameTempDown = frameTempDown_;
  profile.hasFrameOn = true;
  profile.hasFrameOff = true;
  profile.hasFrameTempUp = true;
  profile.hasFrameTempDown = true;

  if (probe.ok && probe.mode == AcControlMode::kSemantic)
  {
    profile.mode = AcControlMode::kSemantic;
    profile.vendor = probe.vendor;
    profile.model = probe.model;
    profile.probeConfidence = probe.confidence;
    profile.lastState = probe.stateOn;
    profile.hasLastState = true;
    profile.probeFromScan = probe.probeFromScan;
    profile.tempTrusted = probe.tempTrusted;

    if (!acProfileStore.save(profile) && ENABLE_DEBUG)
      Serial.println(F("[PAIR] warn: NVS save failed — profile kept in RAM"));
    else if (ENABLE_DEBUG)
      Serial.println(F("[PAIR] profile saved to NVS"));

    postPairSemantic_ = true;
    postPairStateOn_ = probe.stateOn;
    postPairVendor_ = profile.vendor;
    postPairModel_ = profile.model;
    postPairConf_ = profile.probeConfidence;
    postPairTempTrusted_ = profile.tempTrusted;
    postPairAction_ = PostPairAction::kReloadDriver;
    return;
  }

  if (probe.ok && probe.mode == AcControlMode::kRawFallback)
  {
    profile.mode = AcControlMode::kRawFallback;
    uint8_t voteCount = 0;
    profile.vendor =
        acProtocolConsensusFromFrames(frameOn_, frameOff_, frameTempUp_, frameTempDown_, &voteCount);
    profile.probeConfidence = probe.confidence;
    profile.hasLastState = false;
    profile.probeFromScan = false;
    profile.tempTrusted = false;
    if (!acProfileStore.save(profile) && ENABLE_DEBUG)
      Serial.println(F("[PAIR] warn: NVS save failed — profile kept in RAM"));
    else if (ENABLE_DEBUG)
      Serial.println(F("[PAIR] profile saved to NVS"));

    postPairSemantic_ = false;
    postPairConf_ = profile.probeConfidence;
    postPairAction_ = PostPairAction::kSaveRawSlots;

    if (ENABLE_DEBUG)
    {
      Serial.print(F("[PAIR] raw fallback conf="));
      Serial.print(profile.probeConfidence);
      Serial.print(F("% vendorHint="));
      Serial.println(acProtocolName(profile.vendor));
    }
    return;
  }

  if (ENABLE_DEBUG)
  {
    Serial.print(F("[PAIR] failed bestConf="));
    Serial.println(probe.confidence);
  }
  setPhase(AcOnboardPhase::kFailed);
}

void AcOnboarding::poll()
{
  if (postPairAction_ != PostPairAction::kNone)
  {
    tickPostPair();
    return;
  }

  if (probePending_)
  {
    probePending_ = false;
    runProbe();
    return;
  }

  if (phase_ == AcOnboardPhase::kPairWarmup)
  {
    if ((int32_t)(millis() - pairWarmupUntilMs_) < 0)
      return;

    irTransport.enableRx();
    irTransport.flushRxBuffer();
    setPhase(AcOnboardPhase::kWaitPowerOn);
    if (ENABLE_DEBUG)
    {
      Serial.println(F("[PAIR] listen ready"));
      Serial.println(F("[PAIR] Step 1/4: point remote and press POWER ON"));
      Serial.println(F("[PAIR] Flow: ON -> TEMP UP -> TEMP DOWN -> OFF"));
    }
    return;
  }

  if (phase_ == AcOnboardPhase::kIdle || phase_ == AcOnboardPhase::kDone ||
      phase_ == AcOnboardPhase::kFailed)
    return;

  if (isTimedOut())
  {
    if (ENABLE_DEBUG)
      Serial.println(F("[PAIR] timeout"));
    setPhase(AcOnboardPhase::kFailed);
    return;
  }

  if (phase_ != AcOnboardPhase::kWaitPowerOn && phase_ != AcOnboardPhase::kWaitPowerOff &&
      phase_ != AcOnboardPhase::kWaitTempUp && phase_ != AcOnboardPhase::kWaitTempDown)
    return;

  pollStepCapture();
}
