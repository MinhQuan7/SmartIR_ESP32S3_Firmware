#pragma once

#ifndef IOT_IR_AC_HYBRID_SEMANTIC_DRIVER_H
#define IOT_IR_AC_HYBRID_SEMANTIC_DRIVER_H

#include <Arduino.h>

#include <IRac.h>

#include "ac_profile.h"
#include "ac_ir_frame.h"
#include "board_pins.h"

struct AcBlockedRxInfo
{
  bool blocked = false;
  decode_type_t expectedVendor = UNKNOWN;
  decode_type_t blockedVendor = UNKNOWN;
  decode_type_t frameType = UNKNOWN;
  stdAc::state_t blockedState = {};
  bool hasBlockedState = false;
};

enum class AcRxRejectReason : uint8_t
{
  kNone = 0,
  kLowScore,
  kVendorMismatch,
  kWinnerMargin,
  kPlausibility,
};

class AcSemanticDriver
{
public:
  void begin(uint16_t txPin);

  bool sendState(const AcDeviceProfile &profile, const stdAc::state_t &desired);
  bool decodeFrame(const AcIrFrame &frame, const AcDeviceProfile &profile,
                   stdAc::state_t *outState);
  /** Locked paired vendor only; returns score when outScore set. */
  bool decodeFrameLocked(const AcIrFrame &frame, const AcDeviceProfile &profile,
                         stdAc::state_t *outState, uint8_t anchorScore, uint8_t *outScore = nullptr);
  /** Best sane climate decode; honors vendor lock, winner margin, anchor scoring. */
  bool decodeFrameBest(const AcIrFrame &frame, const AcDeviceProfile &profile, uint8_t anchorScore,
                       stdAc::state_t *outState, decode_type_t *outVendor = nullptr,
                       AcBlockedRxInfo *outBlockedInfo = nullptr, uint8_t *outBestScore = nullptr,
                       uint8_t *outRunnerUpScore = nullptr, AcRxRejectReason *outReject = nullptr);
  bool passesRxPlausibility(const stdAc::state_t &state, const AcDeviceProfile &profile,
                            uint8_t anchorScore) const;

private:
  IRac ac_{IR_TX_PIN, false, true};
  bool ready_ = false;
};

extern AcSemanticDriver acSemanticDriver;

#endif
