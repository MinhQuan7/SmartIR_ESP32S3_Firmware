# IoT IR AC Hybrid (Semantic + Raw Fallback)

ESP32-S3 firmware combining **IRremoteESP8266 `IRac`** (synthetic AC frames) with **raw replay fallback**, inspired by **Sensibo Sky** pairing (minimal remote presses).

## Why this folder exists

The legacy [`iot_ir_airconditioner`](../iot_ir_airconditioner) replays learned raw waveforms per slot (`cool_22`, `pw_on`, …). That is 100% faithful but painful to onboard (15+ temperature learns per unit).

This hybrid module:

1. **Pairing (2 frames)** — user presses ON then OFF (or ON twice with different temp)
2. **Protocol probe** — tries `IRac::decodeToState` across common AC vendors
3. **Semantic mode** — if probe succeeds, Hub sets `ac_temp=25` without learning `cool_25`
4. **Raw fallback** — if probe fails (e.g. Sharp `UNKNOWN`), stores `pw_on` / `pw_of` and optional `learn cool_XX`

## Architecture

```
                    ┌─────────────────┐
  Hub downlink ───► │  AcHybridDriver │
                    └────────┬────────┘
                             │
              ┌──────────────┴──────────────┐
              ▼                             ▼
     AcSemanticDriver (IRac)        Raw replay (NVS slots)
              │                             │
              └──────────────┬──────────────┘
                             ▼
                      IrTransport (TX/RX)
```

### Onboarding FSM (`ac_onboarding`)

| Step | User action | Firmware |
|------|-------------|----------|
| 1 | Press **POWER ON** | Capture frame A |
| 2a | Press **POWER OFF** | Capture frame B → probe ON/OFF |
| 2b | Change temp, press **ON** again | Capture frame B → probe temp delta |
| 3 | — | `ac_protocol_probe` → semantic or raw profile saved to NVS |

### Protocol probe (`ac_protocol_probe`)

- Compares two frames with similar carrier structure (≥55% raw similarity)
- Tries native `decode_type` then forces ~30 common AC protocols
- **ON/OFF flow**: requires `stateA.power != stateB.power`
- **ON×2 flow**: prefers `degrees` delta ≥ 1°C (helps infer temp encoding)
- Confidence ≥ `AC_PROBE_MIN_CONFIDENCE` (70) → **semantic** profile
- Else if frames differ in predictable cells → **raw fallback**

## Sensibo-style approach — assessment

**Good fit for IoT Hub:**

- Onboarding UX: 2 button presses vs 15+ temperature learns
- When `IRac` recognizes the remote (Daikin, Mitsubishi, many Sharp models), full climate control is immediate
- Raw fallback keeps unknown remotes working (your current Sharp `UNKNOWN` case)

**Limits (be aware):**

- Not magic: if library cannot decode the remote, you still need raw slots for each temperature
- `ON×2` temp inference only works when probe finds semantic decode — not bit-level synthesis from diff alone (future work)
- Enabling many `DECODE_*` AC protocols increases flash ~50–150 KB — enable only what you need in IRremoteESP8266 config

## Hardware

Same as [`iot_ir_airconditioner`](../iot_ir_airconditioner): ESP32-S3 Super Mini, TSOP1738 GPIO4, IR LED GPIO5.

## Libraries

- [IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266) v2.8.6+
- `dtg` from [`firmware/libraries/dtg`](../libraries/dtg)

### IRremote build tip

In `IRremoteESP8266.h` (or project override), keep AC decoders enabled for brands you deploy. Comment out unused protocols to save flash.

## Hub datastreams

| streamId | Values | Notes |
|----------|--------|-------|
| `ac_pair` | Any value (uses fixed 4-step flow) | Starts pairing: ON -> TEMP UP -> TEMP DOWN -> OFF |
| `ac_power` | 0/1 | Works in both modes |
| `ac_mode` | 0–3 | Semantic preferred |
| `ac_temp` | 16–30 | Semantic: no per-slot learn |
| `ac_fan` | 0–3 | Semantic sets absolute fan |
| `ac_profile_mode` | tele 1=semantic, 2=raw | Uplink |
| `learn_slot` | string e.g. `cool_25` | Raw fallback only |

## Serial commands

```
pair          # 4-step: ON, TEMP UP, TEMP DOWN, OFF
status        # show NVS profile
unpair        # clear profile
learn cool_25 # raw fallback slot capture
power on|off  # test TX
wifi_clear    # erase DTG Wi-Fi NVS
```

## Module map

| File | Role |
|------|------|
| `ir_transport` | Low-level IRrecv/IRsend, echo guard |
| `ac_ir_frame` | Frame struct, similarity / diff helpers |
| `ac_protocol_probe` | Two-frame vendor inference |
| `ac_profile` | NVS device profile |
| `ac_raw_store` | NVS raw slots |
| `ac_semantic_driver` | IRac send/decode wrapper |
| `ac_hybrid_driver` | Routes semantic vs raw |
| `ac_onboarding` | Pairing state machine |
| `ac_mqtt_handler` | Hub downlink + passive sync |

## Migration from `iot_ir_airconditioner`

1. Flash `iot_ir_ac_hybrid` on new gateways
2. Run `pair` once per site
3. If semantic: remove manual `cool_XX` learns from ops runbook
4. If raw fallback: existing learned slots can be re-imported via `learn` serial command

## Next steps (future)

- Hub UI wizard for `ac_pair` flow with progress states
- Bit-diff template synthesis for UNKNOWN protocols (infer temp from ON×2 without full IRac)
- Per-project protocol allowlist to trim flash
- Auto-test TX after probe (verify AC beeps)
"# SmartIR_ESP32S3_Firmware" 
