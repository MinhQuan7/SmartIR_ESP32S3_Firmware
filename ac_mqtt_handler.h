#pragma once

#ifndef IOT_IR_AC_HYBRID_MQTT_HANDLER_H
#define IOT_IR_AC_HYBRID_MQTT_HANDLER_H

#include <Arduino.h>

class DtgWiFi;

void acMqttInit(DtgWiFi *dtg);
void acMqttOnConnected();
void acMqttHandleCommand(const char *deviceAlias, const char *streamId,
                         const uint8_t *payload, unsigned int length);
void acMqttPublishFullState();
void acMqttPollRemoteSync();

#endif
