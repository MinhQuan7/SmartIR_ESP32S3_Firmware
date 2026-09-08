#pragma once

#ifndef IOT_IR_AC_HYBRID_ONBOARDING_H
#define IOT_IR_AC_HYBRID_ONBOARDING_H

#include <Arduino.h>

#include "ac_profile.h"

enum class AcOnboardPhase : uint8_t
{
  kIdle = 0,
  kPairWarmup,
  kWaitPowerOn,
  kWaitTempUp,
  kWaitTempDown,
  kWaitPowerOff,
  kProbing,
  kDone,
  kFailed,
};

class AcOnboarding
{
public:
  void begin();

  /** Default high-accuracy pairing: ON -> TEMP UP -> TEMP DOWN -> OFF. */
  bool start();
  void cancel();

  /** Call every loop while pairing. */
  void poll();

  bool isActive() const { return phase_ != AcOnboardPhase::kIdle && phase_ != AcOnboardPhase::kDone; }
  AcOnboardPhase phase() const { return phase_; }

private:
  void setPhase(AcOnboardPhase phase);
  void onFrameCaptured(const AcIrFrame &frame);
  void pollStepCapture();
  static bool frameLooksBetter(const AcIrFrame &candidate, const AcIrFrame &current);
  void runProbe();
  void tickPostPair();
  bool isTimedOut() const;

  enum class PostPairAction : uint8_t
  {
    kNone = 0,
    kSaveRawSlots,
    kReloadDriver,
    kFinalize,
  };

  AcOnboardPhase phase_ = AcOnboardPhase::kIdle;
  AcPairingFlow flow_ = AcPairingFlow::kPowerOnOffTempUpTempDown;
  uint32_t phaseStartedMs_ = 0;
  uint32_t pairWarmupUntilMs_ = 0;
  AcIrFrame pendingFrame_ = {};
  bool hasPendingFrame_ = false;
  uint32_t stepCollectUntilMs_ = 0;
  uint32_t stepGuardUntilMs_ = 0;
  AcIrFrame frameOn_ = {};
  AcIrFrame frameOff_ = {};
  AcIrFrame frameTempUp_ = {};
  AcIrFrame frameTempDown_ = {};
  bool hasFrameOn_ = false;
  bool hasFrameOff_ = false;
  bool hasFrameTempUp_ = false;
  bool hasFrameTempDown_ = false;
  bool probePending_ = false;
  PostPairAction postPairAction_ = PostPairAction::kNone;
  bool postPairSemantic_ = false;
  stdAc::state_t postPairStateOn_ = {};
  decode_type_t postPairVendor_ = UNKNOWN;
  int16_t postPairModel_ = -1;
  uint8_t postPairConf_ = 0;
  bool postPairTempTrusted_ = false;
};

extern AcOnboarding acOnboarding;

#endif
