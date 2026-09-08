#include "ir_transport.h"

#include "board_pins.h"
#include "config.h"
#include "ir_feedback.h"

#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

IrTransport irTransport;

namespace {

IRrecv sRecv(IR_RX_PIN, AC_IR_CAPTURE_BUF, AC_IR_TIMEOUT_MS, true);
IRsend sSend(IR_TX_PIN);
decode_results sResults;

} // namespace

void IrTransport::begin()
{
  sSend.begin();
  sRecv.setUnknownThreshold(AC_IR_MIN_UNKNOWN);
  sRecv.setTolerance(AC_IR_RX_TOLERANCE_PCT);
}

void IrTransport::enableRx()
{
  if (!rxEnabled_)
  {
    sRecv.enableIRIn();
    rxEnabled_ = true;
  }
}

void IrTransport::disableRx()
{
  if (rxEnabled_)
  {
    sRecv.disableIRIn();
    rxEnabled_ = false;
  }
}

void IrTransport::setEchoGuardMs(uint32_t ms)
{
  const uint32_t until = millis() + ms;
  if (until > ignoreRxUntilMs_)
    ignoreRxUntilMs_ = until;
}

void IrTransport::flushRxBuffer()
{
  if (!rxEnabled_)
    enableRx();

  uint8_t drained = 0;
  while (sRecv.decode(&sResults))
  {
    sRecv.resume();
    if (++drained >= 8)
      break;
  }

  if (ENABLE_DEBUG && drained > 0)
  {
    Serial.print(F("[RX] flushed "));
    Serial.print(drained);
    Serial.println(F(" stale frame(s) after TX mute"));
  }
}

void IrTransport::muteRxAfterTx(uint32_t ms)
{
  disableRx();

  const uint32_t guardMs = ms > 0 ? ms : AC_IR_TX_RX_MUTE_MS;
  const uint32_t until = millis() + guardMs;
  if (until > ignoreRxUntilMs_)
    ignoreRxUntilMs_ = until;
  if (until > rxReenableAtMs_)
    rxReenableAtMs_ = until;

  if (ENABLE_DEBUG)
  {
    Serial.print(F("[RX] muted "));
    Serial.print(guardMs);
    Serial.println(F("ms after local TX"));
  }
}

bool IrTransport::isRxMuted() const
{
  if (rxReenableAtMs_ != 0 && (int32_t)(millis() - rxReenableAtMs_) < 0)
    return true;
  return millis() < ignoreRxUntilMs_;
}

void IrTransport::poll()
{
  if (rxReenableAtMs_ == 0)
    return;
  if ((int32_t)(millis() - rxReenableAtMs_) < 0)
    return;

  rxReenableAtMs_ = 0;
  flushRxBuffer();
}

bool IrTransport::pollCapture(AcIrFrame *out)
{
  if (!rxEnabled_ || isRxMuted())
    return false;

  if (!sRecv.decode(&sResults))
    return false;

  if (out)
    acIrFrameFromDecode(sResults, out);

  sRecv.resume();
  return true;
}

void IrTransport::sendRaw(const uint16_t *raw, uint16_t len)
{
  if (!raw || len == 0)
    return;

  disableRx();
  sSend.sendRaw(raw, len, AC_IR_CARRIER_KHZ);
  muteRxAfterTx(AC_IR_TX_RX_MUTE_MS);
  irFeedback.onCommandTx();

  if (ENABLE_DEBUG)
  {
    Serial.print(F("[TX] IR sent carrier="));
    Serial.print(AC_IR_CARRIER_KHZ);
    Serial.print(F("kHz samples="));
    Serial.println(len);
  }
}
