#pragma once

#ifndef IOT_IR_AC_HYBRID_RAW_STORE_H
#define IOT_IR_AC_HYBRID_RAW_STORE_H

#include <Arduino.h>

#include "ac_ir_frame.h"
#include "config.h"

bool acRawStoreSave(const char *name, const AcIrFrame &frame);
bool acRawStoreLoad(const char *name, AcIrFrame *out);
bool acRawStoreExists(const char *name);

/** Best fuzzy match against all NVS raw slots. Returns slot name or nullptr. */
const char *acRawStoreMatchBest(const AcIrFrame &frame, uint8_t *outScore);
const char *acRawStoreMatchBestSync(const AcIrFrame &frame, uint8_t *outScore);

#endif
