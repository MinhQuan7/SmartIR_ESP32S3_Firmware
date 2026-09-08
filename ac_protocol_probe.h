#pragma once

#ifndef IOT_IR_AC_HYBRID_PROTOCOL_PROBE_H
#define IOT_IR_AC_HYBRID_PROTOCOL_PROBE_H

#include <Arduino.h>

#include <IRremoteESP8266.h>
#include <IRac.h>

#include "ac_ir_frame.h"
#include "ac_profile.h"

struct AcProbeResult
{
  bool ok = false;
  AcControlMode mode = AcControlMode::kNone;
  decode_type_t vendor = UNKNOWN;
  int16_t model = -1;
  uint8_t confidence = 0;
  stdAc::state_t stateOn = {};
  stdAc::state_t stateOff = {};
  bool hasSemantic = false;
  bool probeFromScan = false;
  bool tempTrusted = false;
  uint8_t runnerUpConfidence = 0;
};

/**
 * Sensibo-style inference from captured pairing frames.
 * 1) IRAcUtils auto-decode (native IRrecv protocol id)
 * 2) Full scan: every decode_type where IRac::isProtocolSupported()
 */
AcProbeResult acProtocolProbe(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                              const AcIrFrame *frameTempUp, const AcIrFrame *frameTempDown);

const char *acProtocolName(decode_type_t vendor);

/** Protocols that must not win probe from misleading native decode. */
bool acProtocolIsUntrusted(decode_type_t vendor);

/**
 * Majority decode_type from pairing captures (>=3 of 4). UNKNOWN if no consensus.
 * outVoteCount optional (0-4).
 */
decode_type_t acProtocolConsensusFromFrames(const AcIrFrame &frameOn, const AcIrFrame &frameOff,
                                              const AcIrFrame &frameTempUp,
                                              const AcIrFrame &frameTempDown,
                                              uint8_t *outVoteCount = nullptr);

/** True when IRrecv reported a supported protocol id on capture. */
bool acProtocolIsValidVendor(decode_type_t vendor);

/** Returns true when built-in consensus checks pass (serial selfcheck). */
bool acProtocolConsensusSelfCheck();

#endif
