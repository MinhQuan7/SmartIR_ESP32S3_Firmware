#pragma once

#ifndef IOT_IR_AC_HYBRID_STATE_MODEL_H
#define IOT_IR_AC_HYBRID_STATE_MODEL_H

#include <Arduino.h>

#include <IRac.h>

#include "ac_profile.h"

#include "config.h"

struct AcClimateState
{
  bool power = false;
  stdAc::opmode_t mode = stdAc::opmode_t::kCool;
  float tempC = AC_TEMP_DEFAULT;
  stdAc::fanspeed_t fan = stdAc::fanspeed_t::kAuto;
  stdAc::swingv_t swingV = stdAc::swingv_t::kOff;
};

class AcStateModel
{
public:
  const AcClimateState &climate() const { return climate_; }

  void fromStdAc(const stdAc::state_t &state);
  stdAc::state_t toStdAc(const AcDeviceProfile &profile) const;

  void setPower(bool on);
  void setMode(stdAc::opmode_t mode);
  void setTempC(float temp);
  void setFan(stdAc::fanspeed_t fan);
  void setSwingV(stdAc::swingv_t swing);

private:
  AcClimateState climate_ = {};
};

extern AcStateModel acStateModel;

/** Apply semantic decode result to climate model if sane. */
bool acClimateFromDecodedState(const stdAc::state_t &state, AcClimateState *out);

#endif
