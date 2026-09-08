#pragma once

#ifndef IOT_IR_AC_HYBRID_FRAME_H
#define IOT_IR_AC_HYBRID_FRAME_H

#include <Arduino.h>

#include <IRrecv.h>
#include <IRremoteESP8266.h>

#include "config.h"

struct AcDeviceProfile;

/** One captured IR frame (timings + decoder state for IRAcUtils). */
struct AcIrFrame
{
  uint16_t raw[AC_IR_MAX_RAW_LEN];
  uint16_t rawLen = 0;
  uint64_t code = 0;
  uint16_t bits = 0;
  decode_type_t decodeType = UNKNOWN;
  uint8_t state[kStateSizeMax] = {};
  uint16_t stateLen = 0;
  bool repeat = false;
};

inline uint16_t acIrMinU16(uint16_t a, uint16_t b)
{
  return a < b ? a : b;
}

void acIrFrameFromDecode(const decode_results &results, AcIrFrame *out);

/**
 * Rebuild decode_results for IRAcUtils. Caller supplies rawBuf (min rawLen+1).
 * rawbuf in decode_results is a pointer in IRremoteESP8266 v2.8+.
 */
void acIrFrameToDecodeResults(const AcIrFrame &frame, decode_type_t forcedType,
                              decode_results *out, uint16_t *rawBuf, uint16_t rawBufCap);

uint8_t acIrFrameSimilarity(const AcIrFrame &a, const AcIrFrame &b);
/** Fuzzy match for passive RX: prefix-weighted + wider timing tolerance. */
uint8_t acIrFrameSyncScore(const AcIrFrame &a, const AcIrFrame &b);
uint16_t acIrFrameDiffCells(const AcIrFrame &a, const AcIrFrame &b, uint16_t toleranceUs);

/** Best prefix-weighted sync vs paired ON / temp frames (RX anchor gate). */
uint8_t acIrFrameBestAnchorScore(const AcIrFrame &frame, const AcDeviceProfile &profile);

bool acIrFramePassesQualityGate(const AcIrFrame &frame);

#endif
