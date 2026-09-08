#pragma once

#ifndef IOT_IR_AC_HYBRID_CONFIG_H
#define IOT_IR_AC_HYBRID_CONFIG_H

#define DTG_PROFILE_MQTT_ONLY 0
#define DTG_FEATURE_HUB_CMD 1
#define DTG_FEATURE_GPS 0
#define DTG_NO_DEFAULT_BANNER 1
#define DTG_WIFI_FORCE_SKETCH_CREDS 1
#define DTG_WIFI_BOOT_PIN 255

#define DTG_MQTT_BUFFER_CORE 2048

static const char *WIFI_SSID = "your-wifi-ssid";
static const char *WIFI_PASS = "your-wifi-password";
static const char *PROJECT_ID = "your-project-id";
static const char *GW_ALIAS = "your-gateway-alias";
static const char *MQTT_USER = "your-mqtt-username";
static const char *MQTT_PASS = "your-mqtt-password";
static const char *CLIENT_ID = "your-gateway-id";


static const bool ENABLE_DEBUG = true;

#ifndef DTG_FIRMWARE_VERSION
#define DTG_FIRMWARE_VERSION "1.4.7-hybrid"
#endif

// IR capture
#define AC_IR_CAPTURE_BUF 1024
#define AC_IR_TIMEOUT_MS 50
#define AC_IR_MIN_UNKNOWN 24
#define AC_IR_MIN_AC_RAW_LEN 48
#define AC_IR_RX_TOLERANCE_PCT 22
#define AC_IR_MAX_RAW_LEN 500
#define AC_IR_MAX_SLOT_NAME 16
#define AC_IR_MAX_RAW_SLOTS 24
#define AC_IR_ECHO_GUARD_MS 400
/** Keep IR RX hardware off after any local TX (blocks self-echo / leakage). */
#define AC_IR_TX_RX_MUTE_MS 1200
/** Coalesce duplicate MQTT uplinks only; IR capture is never debounced. */
#define AC_IR_DEBOUNCE_MS 120
/** Max IR frames drained per loop (one remote burst is often 3-6 packets). */
#define AC_IR_RX_DRAIN_MAX 8
#define AC_IR_CARRIER_KHZ 38

// Fuzzy raw match (passive sync + fallback path)
#define AC_IR_MATCH_MIN_SCORE 85
#define AC_IR_MATCH_TOLERANCE_US 80
/** Looser match for passive RX sync (full-state remotes change payload per temp). */
#define AC_IR_SYNC_MIN_SCORE 62
#define AC_IR_SYNC_TOLERANCE_US 140
#define AC_IR_SYNC_PREFIX_CELLS 40
/** Min score gap when both ON/OFF frames match (avoids wrong power sync). */
#define AC_IR_POWER_MATCH_MIN_DELTA 3

// Protocol probe from two captured frames (Sensibo-style pairing)
#define AC_PROBE_MIN_CONFIDENCE 70
#define AC_PROBE_MIN_CONFIDENCE_SCAN 85
#define AC_PROBE_STRONG_OVERRIDE_CONF 92
#define AC_PROBE_WINNER_MARGIN 8
#define AC_RX_DECODE_MIN_SCORE 45
#define AC_RX_DECODE_MIN_SCORE_RAW 28
#define AC_RX_RAW_ANCHOR_MIN 50
#define AC_RX_WINNER_MARGIN 8
#define AC_RX_ENFORCE_PAIRED_VENDOR 1
#define AC_PROBE_CAPTURE_TIMEOUT_MS 60000
/** Ignore IR captures briefly after entering pair (settle TSOP / drain noise). */
#define AC_PAIR_SETTLE_MS 1000
/** Pairing: collect the full burst of one keypress, keep the best frame
 *  (native-decoded tag beats UNKNOWN, then longer raw length wins). */
#define AC_PAIR_STEP_COLLECT_MS 450
/** Pairing: drop stray tail fragments after a step frame is committed. */
#define AC_PAIR_STEP_GUARD_MS 400

// Climate
#define AC_TEMP_MIN 16
#define AC_TEMP_MAX 30
#define AC_TEMP_DEFAULT 25

// NVS
#define AC_PROFILE_NVS_NS "ir_ac_hyb"
#define AC_RAW_NVS_NS "ir_ac_raw"

// LED / buzzer on GPIO6-8 (non-blocking ir_feedback.cpp)
#define FEEDBACK_ENABLE 1
#define FEEDBACK_BUZZER_MS 80
#define FEEDBACK_CMD_BLINK_MS 120
#define FEEDBACK_LEARN_BLINK_MS 400

#endif
