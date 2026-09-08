#include "ac_profile.h"

#include <Preferences.h>

#include "ac_protocol_probe.h"

AcProfileStore acProfileStore;

namespace {

Preferences sPrefs;
bool sPrefsOpen = false;

bool prefsBegin()
{
  if (sPrefsOpen)
    return true;
  sPrefsOpen = sPrefs.begin(AC_PROFILE_NVS_NS, false);
  return sPrefsOpen;
}

void prefsEnd()
{
  if (sPrefsOpen)
  {
    sPrefs.end();
    sPrefsOpen = false;
  }
}

void stateToBytes(const stdAc::state_t &state, uint8_t *buf, size_t len)
{
  if (!buf || len < sizeof(stdAc::state_t))
    return;
  memcpy(buf, &state, sizeof(stdAc::state_t));
}

void bytesToState(const uint8_t *buf, size_t len, stdAc::state_t *state)
{
  if (!buf || !state || len < sizeof(stdAc::state_t))
    return;
  memcpy(state, buf, sizeof(stdAc::state_t));
}

} // namespace

bool AcProfileStore::loadFrame(const char *key, AcIrFrame *out)
{
  if (!out || !key || !prefsBegin())
    return false;

  const uint16_t len = sPrefs.getUShort(key, 0);
  if (len == 0 || len > AC_IR_MAX_RAW_LEN)
    return false;

  char rawKey[20];
  char codeKey[20];
  char bitsKey[20];
  char typeKey[20];
  char stateKey[20];
  char stateLenKey[20];
  snprintf(rawKey, sizeof(rawKey), "%s_r", key);
  snprintf(codeKey, sizeof(codeKey), "%s_c", key);
  snprintf(bitsKey, sizeof(bitsKey), "%s_b", key);
  snprintf(typeKey, sizeof(typeKey), "%s_t", key);
  snprintf(stateKey, sizeof(stateKey), "%s_s", key);
  snprintf(stateLenKey, sizeof(stateLenKey), "%s_sl", key);

  out->rawLen = len;
  sPrefs.getBytes(rawKey, out->raw, len * sizeof(uint16_t));
  out->code = sPrefs.getULong64(codeKey, 0);
  out->bits = sPrefs.getUShort(bitsKey, 0);
  out->decodeType = (decode_type_t)sPrefs.getInt(typeKey, UNKNOWN);
  out->stateLen = sPrefs.getUShort(stateLenKey, 0);
  if (out->stateLen > kStateSizeMax)
    out->stateLen = kStateSizeMax;
  if (out->stateLen > 0)
    sPrefs.getBytes(stateKey, out->state, out->stateLen);
  return true;
}

bool AcProfileStore::saveFrame(const char *key, const AcIrFrame &frame)
{
  if (!key || frame.rawLen == 0 || !prefsBegin())
    return false;

  char rawKey[20];
  char codeKey[20];
  char bitsKey[20];
  char typeKey[20];
  char stateKey[20];
  char stateLenKey[20];
  snprintf(rawKey, sizeof(rawKey), "%s_r", key);
  snprintf(codeKey, sizeof(codeKey), "%s_c", key);
  snprintf(bitsKey, sizeof(bitsKey), "%s_b", key);
  snprintf(typeKey, sizeof(typeKey), "%s_t", key);
  snprintf(stateKey, sizeof(stateKey), "%s_s", key);
  snprintf(stateLenKey, sizeof(stateLenKey), "%s_sl", key);

  sPrefs.putUShort(key, frame.rawLen);
  sPrefs.putBytes(rawKey, frame.raw, frame.rawLen * sizeof(uint16_t));
  sPrefs.putULong64(codeKey, frame.code);
  sPrefs.putUShort(bitsKey, frame.bits);
  sPrefs.putInt(typeKey, (int)frame.decodeType);
  sPrefs.putUShort(stateLenKey, frame.stateLen);
  if (frame.stateLen > 0)
    sPrefs.putBytes(stateKey, frame.state, frame.stateLen);
  return true;
}

bool AcProfileStore::ensureLoaded()
{
  if (loaded_)
    return true;
  return load(&cached_);
}

bool AcProfileStore::load(AcDeviceProfile *out)
{
  if (!out)
    return false;

  if (!prefsBegin())
  {
    *out = cached_;
    return cached_.mode != AcControlMode::kNone;
  }

  cached_.mode = (AcControlMode)sPrefs.getUChar("mode", (uint8_t)AcControlMode::kNone);
  cached_.pairingFlow = (AcPairingFlow)sPrefs.getUChar("flow", 0);
  cached_.vendor = (decode_type_t)sPrefs.getInt("vendor", UNKNOWN);
  cached_.model = sPrefs.getShort("model", -1);
  cached_.probeConfidence = sPrefs.getUChar("conf", 0);
  cached_.hasLastState = sPrefs.getBool("hasState", false);
  cached_.probeFromScan = sPrefs.getBool("scan", false);
  cached_.tempTrusted = sPrefs.getBool("tempOk", false);

  if (cached_.hasLastState)
  {
    uint8_t buf[sizeof(stdAc::state_t)];
    sPrefs.getBytes("state", buf, sizeof(buf));
    bytesToState(buf, sizeof(buf), &cached_.lastState);
  }

  cached_.hasFrameOn = loadFrame("fon", &cached_.frameOn);
  cached_.hasFrameOff = loadFrame("fof", &cached_.frameOff);
  cached_.hasFrameTempUp = loadFrame("ftu", &cached_.frameTempUp);
  cached_.hasFrameTempDown = loadFrame("ftd", &cached_.frameTempDown);
  loaded_ = true;
  *out = cached_;
  prefsEnd();
  return cached_.mode != AcControlMode::kNone;
}

bool AcProfileStore::save(const AcDeviceProfile &profile)
{
  cached_ = profile;
  loaded_ = true;

  if (!prefsBegin())
    return false;

  sPrefs.putUChar("mode", (uint8_t)profile.mode);
  sPrefs.putUChar("flow", (uint8_t)profile.pairingFlow);
  sPrefs.putInt("vendor", (int)profile.vendor);
  sPrefs.putShort("model", profile.model);
  sPrefs.putUChar("conf", profile.probeConfidence);
  sPrefs.putBool("hasState", profile.hasLastState);
  sPrefs.putBool("scan", profile.probeFromScan);
  sPrefs.putBool("tempOk", profile.tempTrusted);

  if (profile.hasLastState)
  {
    uint8_t buf[sizeof(stdAc::state_t)];
    stateToBytes(profile.lastState, buf, sizeof(buf));
    sPrefs.putBytes("state", buf, sizeof(buf));
  }

  if (profile.hasFrameOn)
    saveFrame("fon", profile.frameOn);
  if (profile.hasFrameOff)
    saveFrame("fof", profile.frameOff);
  if (profile.hasFrameTempUp)
    saveFrame("ftu", profile.frameTempUp);
  if (profile.hasFrameTempDown)
    saveFrame("ftd", profile.frameTempDown);

  prefsEnd();
  return true;
}

bool acProfileSemanticTrusted(const AcDeviceProfile &profile)
{
  if (profile.mode != AcControlMode::kSemantic || profile.vendor == UNKNOWN)
    return false;
  if (!profile.hasFrameOn || !profile.hasFrameOff)
    return false;
  if (profile.pairingFlow == AcPairingFlow::kPowerOnOffTempUpTempDown &&
      (!profile.hasFrameTempUp || !profile.hasFrameTempDown))
    return false;
  return profile.probeConfidence >= AC_PROBE_MIN_CONFIDENCE && profile.hasLastState;
}

bool acProfileTempTrusted(const AcDeviceProfile &profile)
{
  if (profile.mode != AcControlMode::kSemantic)
    return false;
  if (profile.tempTrusted)
    return true;
  if (profile.pairingFlow == AcPairingFlow::kPowerOnOffTempUpTempDown &&
      profile.hasFrameTempUp && profile.hasFrameTempDown &&
      profile.probeConfidence >= AC_PROBE_MIN_CONFIDENCE)
    return true;
  if (!profile.probeFromScan && profile.model >= 0 && !acProtocolIsUntrusted(profile.vendor))
    return true;
  return false;
}

void AcProfileStore::clear()
{
  cached_ = {};
  loaded_ = false;
  if (prefsBegin())
  {
    sPrefs.clear();
    prefsEnd();
  }
}
