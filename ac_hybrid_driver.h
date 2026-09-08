#pragma once

#ifndef IOT_IR_AC_HYBRID_DRIVER_H
#define IOT_IR_AC_HYBRID_DRIVER_H

#include <Arduino.h>

#include "ac_profile.h"
#include "ac_state_model.h"

class AcHybridDriver
{
public:
  void begin();

  bool applyClimate(const AcClimateState &target);
  bool sendPower(bool on);
  bool sendSlotRaw(const char *slotName);

  /** Passive RX: decode to climate state or raw slot name. */
  bool pollRemoteSync(AcClimateState *outClimate, const char **outRawSlot);

  bool learnRawSlot(const char *name, const AcIrFrame &frame);

  const AcDeviceProfile &profile() const { return acProfileStore.cached(); }

private:
  bool sendSemantic(const stdAc::state_t &desired);
  bool sendRawFromProfile(bool powerOn);
  bool sendRawSlot(const char *name);

  bool profileLoaded_ = false;
};

extern AcHybridDriver acHybridDriver;

#endif
