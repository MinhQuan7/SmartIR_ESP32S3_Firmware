#include "ac_state_model.h"

#include "ac_profile.h"
#include <IRac.h>

AcStateModel acStateModel;

void AcStateModel::fromStdAc(const stdAc::state_t &state)
{
  climate_.power = state.power;
  climate_.mode = state.mode;
  climate_.tempC = state.celsius ? state.degrees : (state.degrees - 32.0f) * 5.0f / 9.0f;
  climate_.fan = state.fanspeed;
  climate_.swingV = state.swingv;
}

stdAc::state_t AcStateModel::toStdAc(const AcDeviceProfile &profile) const
{
  stdAc::state_t state = {};
  if (profile.hasLastState)
    state = profile.lastState;
  else if (profile.vendor != UNKNOWN)
    IRac::initState(&state, profile.vendor, profile.model, climate_.power, climate_.mode,
                    climate_.tempC, true, climate_.fan, climate_.swingV, stdAc::swingh_t::kOff,
                    false, false, false, false, false, false, false, -1, -1);
  else
    IRac::initState(&state);

  state.power = climate_.power;
  state.mode = climate_.mode;
  state.celsius = true;
  state.degrees = climate_.tempC;
  state.fanspeed = climate_.fan;
  state.swingv = climate_.swingV;
  if (profile.model >= 0)
    state.model = profile.model;
  return state;
}

void AcStateModel::setPower(bool on)
{
  climate_.power = on;
}

void AcStateModel::setMode(stdAc::opmode_t mode)
{
  climate_.mode = mode;
}

void AcStateModel::setTempC(float temp)
{
  if (temp >= AC_TEMP_MIN && temp <= AC_TEMP_MAX)
    climate_.tempC = temp;
}

void AcStateModel::setFan(stdAc::fanspeed_t fan)
{
  climate_.fan = fan;
}

void AcStateModel::setSwingV(stdAc::swingv_t swing)
{
  climate_.swingV = swing;
}

bool acClimateFromDecodedState(const stdAc::state_t &state, AcClimateState *out)
{
  if (!out)
    return false;
  if (state.power && (state.degrees < 16.0f || state.degrees > 32.0f))
    return false;
  acStateModel.fromStdAc(state);
  *out = acStateModel.climate();
  return true;
}
