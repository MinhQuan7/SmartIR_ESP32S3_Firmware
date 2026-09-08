#pragma once

#ifndef IOT_IR_AC_HYBRID_PROFILE_H
#define IOT_IR_AC_HYBRID_PROFILE_H

#include <Arduino.h>

#include <IRremoteESP8266.h>
#include <IRac.h>

#include "ac_ir_frame.h"
#include "config.h"

/** How we control this installed AC unit. */
enum class AcControlMode : uint8_t
{
  kNone = 0,
  kSemantic = 1,
  kRawFallback = 2,
};

/** Pairing flow used during onboarding. */
enum class AcPairingFlow : uint8_t
{
  kPowerOnOffTempUpTempDown = 0,
};

struct AcDeviceProfile
{
  AcControlMode mode = AcControlMode::kNone;
  AcPairingFlow pairingFlow = AcPairingFlow::kPowerOnOffTempUpTempDown;
  decode_type_t vendor = UNKNOWN;
  int16_t model = -1;
  uint8_t probeConfidence = 0;
  stdAc::state_t lastState = {};
  bool hasLastState = false;
  AcIrFrame frameOn = {};
  AcIrFrame frameOff = {};
  AcIrFrame frameTempUp = {};
  AcIrFrame frameTempDown = {};
  bool hasFrameOn = false;
  bool hasFrameOff = false;
  bool hasFrameTempUp = false;
  bool hasFrameTempDown = false;
  bool probeFromScan = false;
  bool tempTrusted = false;
};

class AcProfileStore
{
public:
  bool load(AcDeviceProfile *out);
  /** Load NVS into cached_ once; avoids a second AcDeviceProfile on the stack. */
  bool ensureLoaded();
  bool save(const AcDeviceProfile &profile);
  void clear();

  bool isPaired() const { return cached_.mode != AcControlMode::kNone; }
  const AcDeviceProfile &cached() const { return cached_; }
  AcDeviceProfile &mutableCached() { return cached_; }

private:
  bool loadFrame(const char *key, AcIrFrame *out);
  bool saveFrame(const char *key, const AcIrFrame &frame);

  AcDeviceProfile cached_ = {};
  bool loaded_ = false;
};

extern AcProfileStore acProfileStore;

/** True when pair probe selected a semantic AC protocol (IRac TX/RX). */
bool acProfileSemanticTrusted(const AcDeviceProfile &profile);

/** True when probe validated temperature encoding for 4-step pairing. */
bool acProfileTempTrusted(const AcDeviceProfile &profile);

#endif
