#include "ac_mqtt_handler.h"

#include "ac_hybrid_driver.h"
#include "ac_onboarding.h"
#include "ac_profile.h"
#include "ac_state_model.h"
#include "config.h"
#include "ir_transport.h"

#include <dtg.h>
#include <math.h>
#include <string.h>

namespace {

DtgWiFi *sDtg = nullptr;
uint32_t sLastPublishMs = 0;
AcClimateState sLastPublished = {};
bool sHasLastPublished = false;

bool climateSame(const AcClimateState &a, const AcClimateState &b)
{
  return a.power == b.power && a.mode == b.mode && a.fan == b.fan &&
         fabsf(a.tempC - b.tempC) < 0.5f;
}

void publishCmdAck(const char *cmdId, const char *streamId, const char *status, float v)
{
  if (!sDtg || !cmdId || !cmdId[0])
    return;

  char payload[128];
  const int n = snprintf(payload, sizeof(payload),
                         "{\"id\":\"%s\",\"st\":\"%s\",\"stream\":\"%s\",\"v\":%.0f}", cmdId,
                         status, streamId ? streamId : "", static_cast<double>(v));
  if (n > 0 && n < (int)sizeof(payload))
    sDtg->publishTopicSuffix("up/cmd_ack", payload, false, 0);
}

bool parseEnvelope(const uint8_t *payload, unsigned int length, char *cmdIdOut,
                   size_t cmdIdLen, char *valueOut, size_t valueLen)
{
  if (!payload || length == 0 || !valueOut || valueLen == 0)
    return false;

  if (payload[0] != '{')
  {
    const size_t n = min(length, valueLen - 1);
    memcpy(valueOut, payload, n);
    valueOut[n] = '\0';
    return true;
  }

  char tmp[96];
  const size_t n = min(length, sizeof(tmp) - 1);
  memcpy(tmp, payload, n);
  tmp[n] = '\0';

  const char *idKey = strstr(tmp, "\"id\"");
  if (idKey && cmdIdOut && cmdIdLen > 0)
  {
    const char *colon = strchr(idKey, ':');
    if (colon)
    {
      const char *q1 = strchr(colon, '"');
      if (q1)
      {
        const char *q2 = strchr(q1 + 1, '"');
        if (q2 && (size_t)(q2 - q1 - 1) < cmdIdLen)
        {
          memcpy(cmdIdOut, q1 + 1, (size_t)(q2 - q1 - 1));
          cmdIdOut[q2 - q1 - 1] = '\0';
        }
      }
    }
  }

  const char *vKey = strstr(tmp, "\"v\"");
  if (vKey)
  {
    const char *colon = strchr(vKey, ':');
    if (colon)
    {
      colon++;
      while (*colon == ' ')
        colon++;
      if (*colon == '"')
      {
        colon++;
        const char *q2 = strchr(colon, '"');
        if (q2)
        {
          const size_t len = min((size_t)(q2 - colon), valueLen - 1);
          memcpy(valueOut, colon, len);
          valueOut[len] = '\0';
          return true;
        }
      }
      else
      {
        size_t i = 0;
        while (colon[i] && colon[i] != '}' && colon[i] != ',' && i < valueLen - 1)
        {
          valueOut[i] = colon[i];
          i++;
        }
        valueOut[i] = '\0';
        return true;
      }
    }
  }
  return false;
}

long parseLongValue(const char *s)
{
  if (!s || !s[0])
    return 0;
  return strtol(s, nullptr, 10);
}

bool strEq(const char *a, const char *b)
{
  return a && b && strcmp(a, b) == 0;
}

void publishScalar(const char *streamId, float v)
{
  if (sDtg)
    sDtg->publish(streamId, v, true);
}

stdAc::opmode_t modeFromVal(long val)
{
  switch (val)
  {
  case 0:
    return stdAc::opmode_t::kAuto;
  case 1:
    return stdAc::opmode_t::kCool;
  case 2:
    return stdAc::opmode_t::kDry;
  case 3:
    return stdAc::opmode_t::kFan;
  default:
    return stdAc::opmode_t::kCool;
  }
}

stdAc::fanspeed_t fanFromVal(long val)
{
  switch (val)
  {
  case 1:
    return stdAc::fanspeed_t::kMin;
  case 2:
    return stdAc::fanspeed_t::kMedium;
  case 3:
    return stdAc::fanspeed_t::kMax;
  default:
    return stdAc::fanspeed_t::kAuto;
  }
}

// Uplink inverses of modeFromVal/fanFromVal: collapse the 6-level stdAc enums
// onto the Hub 0-3 scale. Publishing the raw enum cast breaks the Hub reading
// (e.g. stdAc kMedium=3 shows as Hub "Max", stdAc kDry=3 shows as Hub "Fan").
long modeToVal(stdAc::opmode_t mode)
{
  switch (mode)
  {
  case stdAc::opmode_t::kCool:
    return 1;
  case stdAc::opmode_t::kDry:
    return 2;
  case stdAc::opmode_t::kFan:
    return 3;
  default: // kAuto, kOff, kHeat (Hub has no heat mode)
    return 0;
  }
}

long fanToVal(stdAc::fanspeed_t fan)
{
  switch (fan)
  {
  case stdAc::fanspeed_t::kMin:
  case stdAc::fanspeed_t::kLow:
    return 1;
  case stdAc::fanspeed_t::kMedium:
  case stdAc::fanspeed_t::kMediumHigh:
    return 2;
  case stdAc::fanspeed_t::kHigh:
  case stdAc::fanspeed_t::kMax:
    return 3;
  default: // kAuto
    return 0;
  }
}

} // namespace

void acMqttInit(DtgWiFi *dtg)
{
  sDtg = dtg;
}

void acMqttPublishFullState()
{
  const AcClimateState &st = acStateModel.climate();
  publishScalar("ac_power", st.power ? 1.0f : 0.0f);
  publishScalar("ac_mode", static_cast<float>(modeToVal(st.mode)));
  publishScalar("ac_temp", st.tempC);
  publishScalar("ac_fan", static_cast<float>(fanToVal(st.fan)));
  float profileMode = 0.0f;
  const AcControlMode mode = acHybridDriver.profile().mode;
  if (mode == AcControlMode::kSemantic)
    profileMode = 1.0f;
  else if (mode == AcControlMode::kRawFallback)
    profileMode = 2.0f;
  publishScalar("ac_profile_mode", profileMode);
}

void acMqttOnConnected()
{
  acMqttPublishFullState();
}

void acMqttPollRemoteSync()
{
  if (irTransport.isRxMuted())
    return;

  const uint32_t now = millis();
  char rawSlotBuf[AC_IR_MAX_SLOT_NAME] = {};
  const char *rawSlot = nullptr;
  bool gotSync = false;

  for (uint8_t drain = 0; drain < AC_IR_RX_DRAIN_MAX; drain++)
  {
    AcClimateState climate = {};
    const char *slot = nullptr;
    if (!acHybridDriver.pollRemoteSync(&climate, &slot))
      break;
    gotSync = true;
    if (slot)
    {
      strncpy(rawSlotBuf, slot, sizeof(rawSlotBuf) - 1);
      rawSlotBuf[sizeof(rawSlotBuf) - 1] = '\0';
      rawSlot = rawSlotBuf;
    }
  }

  if (!gotSync)
    return;

  const AcClimateState &cur = acStateModel.climate();
  if (sHasLastPublished && climateSame(cur, sLastPublished) &&
      (now - sLastPublishMs) < AC_IR_DEBOUNCE_MS)
    return;

  sLastPublishMs = now;
  sLastPublished = cur;
  sHasLastPublished = true;
  if (rawSlot)
  {
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"v\":\"%s\"}", rawSlot);
    if (sDtg)
      sDtg->publishJson("ac_last_slot", payload);
  }
  acMqttPublishFullState();
}

static bool handleStream(const char *streamId, long val, const char *cmdId)
{
  AcClimateState target = acStateModel.climate();

  if (strEq(streamId, "ac_power"))
  {
    const bool on = val != 0;
    if (!acHybridDriver.sendPower(on))
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return false;
    }
    publishScalar("ac_power", on ? 1.0f : 0.0f);
    if (cmdId[0])
      publishCmdAck(cmdId, streamId, "ok", on ? 1.0f : 0.0f);
    return true;
  }

  if (strEq(streamId, "ac_mode"))
  {
    if (val < 0 || val > 3)
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return false;
    }
    target.mode = modeFromVal(val);
    target.power = true;
    if (!acHybridDriver.applyClimate(target))
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return false;
    }
    publishScalar("ac_mode", static_cast<float>(val));
    publishScalar("ac_power", 1.0f);
    if (cmdId[0])
      publishCmdAck(cmdId, streamId, "ok", static_cast<float>(val));
    return true;
  }

  if (strEq(streamId, "ac_temp"))
  {
    if (val < AC_TEMP_MIN || val > AC_TEMP_MAX)
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return false;
    }
    target.tempC = static_cast<float>(val);
    target.power = true;
    target.mode = stdAc::opmode_t::kCool;
    if (!acHybridDriver.applyClimate(target))
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return false;
    }
    publishScalar("ac_temp", static_cast<float>(val));
    publishScalar("ac_power", 1.0f);
    if (cmdId[0])
      publishCmdAck(cmdId, streamId, "ok", static_cast<float>(val));
    return true;
  }

  if (strEq(streamId, "ac_fan"))
  {
    if (val < 0 || val > 3)
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return false;
    }
    target.fan = fanFromVal(val);
    if (!acHybridDriver.applyClimate(target))
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return false;
    }
    publishScalar("ac_fan", static_cast<float>(val));
    if (cmdId[0])
      publishCmdAck(cmdId, streamId, "ok", static_cast<float>(val));
    return true;
  }

  return false;
}

void acMqttHandleCommand(const char *deviceAlias, const char *streamId,
                         const uint8_t *payload, unsigned int length)
{
  if (!streamId || !deviceAlias)
    return;

  const char *dev = (deviceAlias[0] != '\0') ? deviceAlias : "_";
  if (strcmp(dev, "_") != 0)
    return;

  char cmdId[48] = {};
  char valueBuf[32] = {};
  if (!parseEnvelope(payload, length, cmdId, sizeof(cmdId), valueBuf, sizeof(valueBuf)))
    return;

  const long val = parseLongValue(valueBuf);

  if (strEq(streamId, "ac_pair"))
  {
    if (acOnboarding.start())
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "accepted", 0);
      return;
    }
    if (cmdId[0])
      publishCmdAck(cmdId, streamId, "fail", 0);
    return;
  }

  if (strEq(streamId, "learn_slot"))
  {
    if (!valueBuf[0])
    {
      if (cmdId[0])
        publishCmdAck(cmdId, streamId, "fail", 0);
      return;
    }
    if (cmdId[0])
      publishCmdAck(cmdId, streamId, "accepted", 0);
    return;
  }

  if (!acProfileStore.isPaired())
  {
    if (ENABLE_DEBUG)
    {
      Serial.print(F("[MQTT] skip "));
      Serial.print(streamId);
      Serial.println(F(" — not paired (run pair first)"));
    }
    if (cmdId[0])
      publishCmdAck(cmdId, streamId, "fail", 0);
    return;
  }

  if (ENABLE_DEBUG)
  {
    Serial.print(F("[MQTT] cmd "));
    Serial.print(streamId);
    Serial.print(F(" v="));
    Serial.println(valueBuf);
  }

  if (!handleStream(streamId, val, cmdId) && ENABLE_DEBUG)
  {
    Serial.print(F("[MQTT] unhandled stream "));
    Serial.println(streamId);
  }
}
