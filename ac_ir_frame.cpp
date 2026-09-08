#include "ac_ir_frame.h"

#include <IRutils.h>

#include "ac_profile.h"
#include "config.h"
#include "ac_protocol_probe.h"

#include <string.h>

void acIrFrameFromDecode(const decode_results &results, AcIrFrame *out)
{
  if (!out)
    return;

  out->code = results.value;
  out->bits = results.bits;
  out->decodeType = results.decode_type;
  if (!acProtocolIsValidVendor(out->decodeType))
    out->decodeType = UNKNOWN;
  out->repeat = results.repeat;
  out->rawLen = acIrMinU16((uint16_t)(results.rawlen - 1), (uint16_t)AC_IR_MAX_RAW_LEN);

  if (results.rawbuf)
  {
    for (uint16_t i = 0; i < out->rawLen; i++)
      out->raw[i] = results.rawbuf[i + 1] * kRawTick;
  }

  const uint16_t stateBytes = (uint16_t)((results.bits + 7U) / 8U);
  out->stateLen = acIrMinU16(stateBytes, kStateSizeMax);
  if (out->stateLen > 0)
    memcpy(out->state, results.state, out->stateLen);
}

uint8_t acIrFrameSimilarity(const AcIrFrame &a, const AcIrFrame &b)
{
  if (a.rawLen < 8 || b.rawLen < 8)
    return 0;

  const uint16_t n = acIrMinU16(a.rawLen, b.rawLen);
  uint16_t matched = 0;
  for (uint16_t i = 0; i < n; i++)
  {
    const int diff = (int)a.raw[i] - (int)b.raw[i];
    if (diff >= -AC_IR_MATCH_TOLERANCE_US && diff <= AC_IR_MATCH_TOLERANCE_US)
      matched++;
  }
  return (uint8_t)((matched * 100U) / n);
}

uint8_t acIrFrameSyncScore(const AcIrFrame &a, const AcIrFrame &b)
{
  if (a.rawLen < 8 || b.rawLen < 8)
    return 0;

  const uint16_t n = acIrMinU16(a.rawLen, b.rawLen);
  const uint16_t prefixN = acIrMinU16(n, (uint16_t)AC_IR_SYNC_PREFIX_CELLS);
  uint16_t prefixMatched = 0;
  uint16_t fullMatched = 0;

  for (uint16_t i = 0; i < n; i++)
  {
    const int diff = (int)a.raw[i] - (int)b.raw[i];
    if (diff >= -(int)AC_IR_SYNC_TOLERANCE_US && diff <= (int)AC_IR_SYNC_TOLERANCE_US)
    {
      fullMatched++;
      if (i < prefixN)
        prefixMatched++;
    }
  }

  const uint8_t fullScore = (uint8_t)((fullMatched * 100U) / n);
  const uint8_t prefixScore =
      prefixN > 0 ? (uint8_t)((prefixMatched * 100U) / prefixN) : fullScore;
  return fullScore > prefixScore ? fullScore : prefixScore;
}

uint16_t acIrFrameDiffCells(const AcIrFrame &a, const AcIrFrame &b, uint16_t toleranceUs)
{
  if (a.rawLen < 8 || b.rawLen < 8)
    return 0;

  const uint16_t n = acIrMinU16(a.rawLen, b.rawLen);
  uint16_t diff = 0;
  for (uint16_t i = 0; i < n; i++)
  {
    const int delta = (int)a.raw[i] - (int)b.raw[i];
    if (delta < -(int)toleranceUs || delta > (int)toleranceUs)
      diff++;
  }
  return diff;
}

uint8_t acIrFrameBestAnchorScore(const AcIrFrame &frame, const AcDeviceProfile &profile)
{
  uint8_t best = 0;
  if (profile.hasFrameOn)
  {
    const uint8_t s = acIrFrameSyncScore(frame, profile.frameOn);
    if (s > best)
      best = s;
  }
  if (profile.hasFrameTempUp)
  {
    const uint8_t s = acIrFrameSyncScore(frame, profile.frameTempUp);
    if (s > best)
      best = s;
  }
  if (profile.hasFrameTempDown)
  {
    const uint8_t s = acIrFrameSyncScore(frame, profile.frameTempDown);
    if (s > best)
      best = s;
  }
  return best;
}

bool acIrFramePassesQualityGate(const AcIrFrame &frame)
{
  return frame.rawLen >= AC_IR_MIN_AC_RAW_LEN;
}

void acIrFrameToDecodeResults(const AcIrFrame &frame, decode_type_t forcedType,
                              decode_results *out, uint16_t *rawBuf, uint16_t rawBufCap)
{
  if (!out || !rawBuf || rawBufCap < 2)
    return;

  *out = {};
  out->decode_type = forcedType != UNKNOWN ? forcedType : frame.decodeType;
  out->value = frame.code;
  out->bits = frame.bits;
  out->repeat = frame.repeat;

  const uint16_t need = acIrMinU16((uint16_t)(frame.rawLen + 1), rawBufCap);
  rawBuf[0] = 0;
  for (uint16_t i = 0; i + 1 < need; i++)
    rawBuf[i + 1] = frame.raw[i] / kRawTick;

  out->rawbuf = reinterpret_cast<atomic_uint16_t *>(rawBuf);
  out->rawlen = need;

  if (frame.stateLen > 0)
  {
    const uint16_t n = acIrMinU16(frame.stateLen, kStateSizeMax);
    memcpy(out->state, frame.state, n);
  }
}
