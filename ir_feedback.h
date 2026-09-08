#pragma once

#ifndef IOT_IR_AC_HYBRID_FEEDBACK_H
#define IOT_IR_AC_HYBRID_FEEDBACK_H

#include <Arduino.h>

class IrFeedback
{
public:
  void begin();
  void poll();
  void onCommandTx();

private:
  void setCmdLed(bool on);
  void setBuzzer(bool on);
  void advanceCmdBlink(uint32_t nowMs);

  bool cmdBlinkActive_ = false;
  uint8_t cmdBlinkPhase_ = 0;
  uint32_t cmdBlinkNextMs_ = 0;
  uint32_t buzzerOffMs_ = 0;
};

extern IrFeedback irFeedback;

#endif
