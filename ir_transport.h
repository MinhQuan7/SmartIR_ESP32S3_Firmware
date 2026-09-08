#pragma once

#ifndef IOT_IR_AC_HYBRID_TRANSPORT_H
#define IOT_IR_AC_HYBRID_TRANSPORT_H

#include <Arduino.h>

#include "ac_ir_frame.h"
#include "config.h"

class IrTransport
{
public:
  void begin();
  void enableRx();
  void disableRx();

  /** Non-blocking capture. Returns true when a frame is ready. */
  bool pollCapture(AcIrFrame *out);

  void sendRaw(const uint16_t *raw, uint16_t len);

  /** Disable RX and schedule re-enable after guard (call poll() from loop). */
  void muteRxAfterTx(uint32_t ms);
  void poll();

  void setEchoGuardMs(uint32_t ms);

  /** Drop decoded frames still in the IRrecv queue (after warmup / TX mute). */
  void flushRxBuffer();

  bool isRxEnabled() const { return rxEnabled_; }
  bool isRxMuted() const;
  uint32_t ignoreRxUntilMs() const { return ignoreRxUntilMs_; }

private:
  bool rxEnabled_ = false;
  uint32_t ignoreRxUntilMs_ = 0;
  uint32_t rxReenableAtMs_ = 0;
};

extern IrTransport irTransport;

#endif
