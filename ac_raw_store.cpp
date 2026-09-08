#include "ac_raw_store.h"

#include <Preferences.h>

#include <string.h>

namespace {

Preferences sPrefs;
bool sReady = false;

bool beginPrefs()
{
  if (!sReady)
    sReady = sPrefs.begin(AC_RAW_NVS_NS, false);
  return sReady;
}

void slotRawKey(const char *name, char *out, size_t outLen)
{
  snprintf(out, outLen, "r_%s", name);
}

void slotLenKey(const char *name, char *out, size_t outLen)
{
  snprintf(out, outLen, "l_%s", name);
}

} // namespace

bool acRawStoreSave(const char *name, const AcIrFrame &frame)
{
  if (!name || !name[0] || frame.rawLen == 0 || !beginPrefs())
    return false;

  char rawKey[24];
  char lenKey[24];
  slotRawKey(name, rawKey, sizeof(rawKey));
  slotLenKey(name, lenKey, sizeof(lenKey));

  sPrefs.putUShort(lenKey, frame.rawLen);
  sPrefs.putBytes(rawKey, frame.raw, frame.rawLen * sizeof(uint16_t));

  for (uint8_t i = 0; i < AC_IR_MAX_RAW_SLOTS; i++)
  {
    char nameKey[12];
    snprintf(nameKey, sizeof(nameKey), "n%02u", i);
    char existing[AC_IR_MAX_SLOT_NAME + 1];
    sPrefs.getString(nameKey, existing, sizeof(existing));
    if (strcmp(existing, name) == 0)
      return true;
    if (existing[0] == '\0')
    {
      sPrefs.putString(nameKey, name);
      return true;
    }
  }
  return true;
}

bool acRawStoreLoad(const char *name, AcIrFrame *out)
{
  if (!name || !out || !beginPrefs())
    return false;

  char lenKey[24];
  slotLenKey(name, lenKey, sizeof(lenKey));
  const uint16_t len = sPrefs.getUShort(lenKey, 0);
  if (len == 0 || len > AC_IR_MAX_RAW_LEN)
    return false;

  char rawKey[24];
  slotRawKey(name, rawKey, sizeof(rawKey));
  out->rawLen = len;
  sPrefs.getBytes(rawKey, out->raw, len * sizeof(uint16_t));
  return true;
}

bool acRawStoreExists(const char *name)
{
  if (!name || !beginPrefs())
    return false;

  char lenKey[24];
  slotLenKey(name, lenKey, sizeof(lenKey));
  return sPrefs.getUShort(lenKey, 0) > 0;
}

const char *acRawStoreMatchBestScore(const AcIrFrame &frame, uint8_t minScore, bool useSync,
                                     uint8_t *outScore)
{
  static char bestName[AC_IR_MAX_SLOT_NAME + 1];
  bestName[0] = '\0';
  uint8_t best = 0;

  if (!beginPrefs())
    return nullptr;

  for (uint8_t i = 0; i < AC_IR_MAX_RAW_SLOTS; i++)
  {
    char name[AC_IR_MAX_SLOT_NAME + 1];
    snprintf(name, sizeof(name), "s%02u", i);
    char nameKey[12];
    snprintf(nameKey, sizeof(nameKey), "n%02u", i);
    sPrefs.getString(nameKey, name, sizeof(name));
    if (name[0] == '\0')
      continue;

    AcIrFrame slot = {};
    if (!acRawStoreLoad(name, &slot))
      continue;

    const uint8_t score =
        useSync ? acIrFrameSyncScore(frame, slot) : acIrFrameSimilarity(frame, slot);
    if (score > best)
    {
      best = score;
      strncpy(bestName, name, sizeof(bestName) - 1);
      bestName[sizeof(bestName) - 1] = '\0';
    }
  }

  if (outScore)
    *outScore = best;
  return (best >= minScore) ? bestName : nullptr;
}

const char *acRawStoreMatchBest(const AcIrFrame &frame, uint8_t *outScore)
{
  return acRawStoreMatchBestScore(frame, AC_IR_MATCH_MIN_SCORE, false, outScore);
}

const char *acRawStoreMatchBestSync(const AcIrFrame &frame, uint8_t *outScore)
{
  return acRawStoreMatchBestScore(frame, AC_IR_SYNC_MIN_SCORE, true, outScore);
}
