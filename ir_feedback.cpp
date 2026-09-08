#include "ir_feedback.h"

#include "board_pins.h"
#include "config.h"

IrFeedback irFeedback;

namespace {

constexpr uint8_t kCmdBlinkPhases = 4;

} // namespace

void IrFeedback::setCmdLed(bool on)
{
#if FEEDBACK_ENABLE
  digitalWrite(FEEDBACK_CMD_LED_PIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

void IrFeedback::setBuzzer(bool on)
{
#if FEEDBACK_ENABLE
  digitalWrite(FEEDBACK_BUZZER_PIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

void IrFeedback::begin()
{
#if FEEDBACK_ENABLE
  pinMode(FEEDBACK_CMD_LED_PIN, OUTPUT);
  pinMode(FEEDBACK_BUZZER_PIN, OUTPUT);
  setCmdLed(false);
  setBuzzer(false);
#endif
}

void IrFeedback::onCommandTx()
{
#if !FEEDBACK_ENABLE
  return;
#endif

  const uint32_t now = millis();
  cmdBlinkActive_ = true;
  cmdBlinkPhase_ = 0;
  cmdBlinkNextMs_ = now + FEEDBACK_CMD_BLINK_MS;
  setCmdLed(true);
  setBuzzer(true);
  buzzerOffMs_ = now + FEEDBACK_BUZZER_MS;
}

void IrFeedback::advanceCmdBlink(uint32_t nowMs)
{
  if (!cmdBlinkActive_)
    return;
  if (nowMs < cmdBlinkNextMs_)
    return;

  cmdBlinkPhase_++;
  if (cmdBlinkPhase_ >= kCmdBlinkPhases)
  {
    cmdBlinkActive_ = false;
    setCmdLed(false);
    return;
  }

  setCmdLed((cmdBlinkPhase_ % 2) == 0);
  cmdBlinkNextMs_ = nowMs + FEEDBACK_CMD_BLINK_MS;
}

void IrFeedback::poll()
{
#if !FEEDBACK_ENABLE
  return;
#endif

  const uint32_t now = millis();
  if (buzzerOffMs_ != 0 && now >= buzzerOffMs_)
  {
    setBuzzer(false);
    buzzerOffMs_ = 0;
  }
  advanceCmdBlink(now);
}
