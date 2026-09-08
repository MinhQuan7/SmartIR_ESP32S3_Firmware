/**
 * @file iot_ir_ac_hybrid.ino
 * @brief Hybrid AC IR gateway: IRac semantic control + raw fallback (Sensibo-style pairing).
 */
#include "ac_hybrid_driver.h"
#include "ac_mqtt_handler.h"
#include "ac_onboarding.h"
#include "ac_profile.h"
#include "ac_protocol_probe.h"
#include "ac_raw_store.h"
#include "ac_state_model.h"
#include "config.h"
#include "ir_feedback.h"
#include "ir_transport.h"

#include <string.h>

#include <dtg.h>
#include <Preferences.h>

DtgWiFi dtg(PROJECT_ID, GW_ALIAS, MQTT_USER, MQTT_PASS, CLIENT_ID);

namespace {

char sLearnName[AC_IR_MAX_SLOT_NAME + 1] = {};
bool sLearnPending = false;

void clearDtgWifiNvs()
{
  Preferences prefs;
  if (!prefs.begin("dtg_wifi", false))
    return;
  prefs.remove("pri_ssid");
  prefs.remove("pri_pass");
  prefs.putUChar("pri_set", 0);
  prefs.remove("bak_ssid");
  prefs.remove("bak_pass");
  prefs.putUChar("bak_set", 0);
  prefs.end();
}

void printProfile()
{
  if (!acProfileStore.isPaired())
    acProfileStore.ensureLoaded();
  const AcDeviceProfile &profile = acProfileStore.cached();
  Serial.print(F("[PROFILE] mode="));
  Serial.print(profile.mode == AcControlMode::kSemantic   ? F("semantic")
               : profile.mode == AcControlMode::kRawFallback ? F("raw")
                                                             : F("none"));
  if (profile.mode != AcControlMode::kNone)
  {
    Serial.print(F(" vendor="));
    if (profile.vendor != UNKNOWN)
      Serial.print(acProtocolName(profile.vendor));
    else
      Serial.print(F("unknown"));
    Serial.print(F(" model="));
    Serial.print(profile.model);
    Serial.print(F(" conf="));
    Serial.print(profile.probeConfidence);
    Serial.print(F("% tempTrusted="));
    Serial.print(profile.tempTrusted ? 1 : 0);
    Serial.print(F(" scan="));
    Serial.println(profile.probeFromScan ? 1 : 0);
  }
  else
  {
    Serial.println();
  }
}

void onHubCommand(const char *deviceAlias, const char *streamId, const uint8_t *payload,
                  unsigned int length, void * /*user*/)
{
  acMqttHandleCommand(deviceAlias, streamId, payload, length);
}

void handleSerial()
{
  if (!Serial.available())
    return;

  char line[48];
  const int n = Serial.readBytesUntil('\n', line, sizeof(line) - 1);
  line[n] = '\0';

  char *p = line;
  while (*p == ' ' || *p == '\r')
    p++;
  // Trim trailing CR (Windows Serial Monitor)
  const size_t plen = strlen(p);
  if (plen > 0 && p[plen - 1] == '\r')
    p[plen - 1] = '\0';

  if (p[0] == '\0')
    return;
  if (strcmp(p, "wifi_clear") == 0)
  {
    clearDtgWifiNvs();
    Serial.println(F("[WIFI] NVS cleared — rebooting"));
    delay(200);
    ESP.restart();
    return;
  }

  if (strcmp(p, "pair") == 0)
  {
    if (acOnboarding.start())
      return;
    Serial.println(F("[PAIR] busy — wait or type unpair"));
    return;
  }

  if (strcmp(p, "unpair") == 0)
  {
    acProfileStore.clear();
    Serial.println(F("[PROFILE] cleared — run pair"));
    return;
  }

  if (strcmp(p, "status") == 0)
  {
    printProfile();
    return;
  }

  if (strcmp(p, "selfcheck") == 0)
  {
    const bool ok = acProtocolConsensusSelfCheck();
    Serial.print(F("[SELFCHK] consensus "));
    Serial.println(ok ? F("PASS") : F("FAIL"));
    return;
  }

  if (strncmp(p, "power ", 6) == 0)
  {
    const bool on = p[6] == '1' || strcmp(p + 6, "on") == 0;
    const bool ok = acHybridDriver.sendPower(on);
    if (!ok)
      Serial.println(F("[TX] power FAIL — run pair first"));
    else
      acMqttPublishFullState();
    return;
  }

  if (strncmp(p, "temp ", 5) == 0)
  {
    const int temp = atoi(p + 5);
    AcClimateState target = acStateModel.climate();
    target.power = true;
    target.tempC = (float)temp;
    target.mode = stdAc::opmode_t::kCool;
    const bool ok = acHybridDriver.applyClimate(target);
    Serial.print(F("[TX] temp "));
    Serial.print(temp);
    Serial.println(ok ? F(" OK") : F(" FAIL"));
    if (ok)
      acMqttPublishFullState();
    return;
  }

  if (strncmp(p, "learn ", 6) == 0)
  {
    const char *name = p + 6;
    while (*name == ' ')
      name++;
    strncpy(sLearnName, name, sizeof(sLearnName) - 1);
    sLearnName[sizeof(sLearnName) - 1] = '\0';
    sLearnPending = sLearnName[0] != '\0';
    Serial.print(F("[LEARN] press remote for "));
    Serial.println(sLearnName);
    return;
  }
}

void pollLearnCapture()
{
  if (!sLearnPending)
    return;

  AcIrFrame frame = {};
  if (!irTransport.pollCapture(&frame))
    return;

  if (acHybridDriver.learnRawSlot(sLearnName, frame))
  {
    Serial.print(F("[LEARN] saved "));
    Serial.println(sLearnName);
  }
  sLearnPending = false;
  sLearnName[0] = '\0';
}

} // namespace

void dtgOnConnected()
{
  acMqttOnConnected();
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  irTransport.begin();
  irTransport.enableRx();
  irFeedback.begin();
  acOnboarding.begin();
  acHybridDriver.begin();
  acMqttInit(&dtg);

  if (ENABLE_DEBUG)
    dtg.setLogStream(&Serial);

  dtg.enableHubCmd();
  detail::HubCmd::instance().setCommandChain(onHubCommand, nullptr);
  dtg.enableWatchdog();
  dtg.begin(WIFI_SSID, WIFI_PASS);

  Serial.print(F("[BOOT] iot_ir_ac_hybrid "));
  Serial.print(F(DTG_FIRMWARE_VERSION));
  Serial.println(F(" — pair ON/TEMP+/TEMP-/OFF"));
  Serial.println(F("[SERIAL] pair | status | selfcheck | unpair | power on|off | temp 26"));
  printProfile();
}

void loop()
{
  handleSerial();
  irTransport.poll();
  acOnboarding.poll();
  pollLearnCapture();
  acMqttPollRemoteSync();
  irFeedback.poll();
  dtg.run();
}
