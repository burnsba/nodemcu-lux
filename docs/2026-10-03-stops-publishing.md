# Board stops publishing to Home Assistant after ~1 day

Date: 2026-10-03

## Issue

The sensor board (ESP8266 NodeMCU + AHT20 + VEML7700) works normally after boot, then after a day or so stops sending readings. No new messages show up in Home Assistant. Power cycling the board brings it back.

Suspects at the start: a slow memory leak, rare I2C bus contention, or a Wi-Fi connection that needs a periodic refresh.

## Findings

Ranked by likelihood. The fixes below address all four, so the next failure (if any) should leave evidence in the serial log.

### 1. The AHT20 read can hang forever after one I2C error (most likely)

`loop()` called `aht.getEvent()`. In `Adafruit_AHTX0.cpp`:

```cpp
uint8_t Adafruit_AHTX0::getStatus(void) {
  uint8_t ret;
  if (!i2c_dev->read(&ret, 1)) {
    return 0xFF;          // read failed
  }
  return ret;
}

  // in getEvent / _readData:
  while (getStatus() & AHTX0_STATUS_BUSY) {   // 0xFF & 0x80 is always true
    delay(10);
  }
```

A single failed status read returns `0xFF`, which has the busy bit (`0x80`) set, so the loop never exits. `delay()` yields to the ESP8266 system tasks, so the hardware watchdog never fires either. The board stays powered with Wi-Fi up and the status LED on, but does nothing.

The I2C bus is moved between two pin pairs with `Wire.begin()` on every loop pass (AHT20 on 12/14, VEML7700 on 5/4), and a reading is taken every ~1.3 s. A rare glitch from that, from electrical noise, or from a supply blip fits "works for about a day."

`aht.begin()` has the same loop pattern, so a bad read during `setup()` hangs the same way.

How to tell it was this: the serial log ends with `wire: reset for AHT` and no `humidity: ...` line after it.

### 2. `client.loop()` was never called

PubSubClient sends MQTT keepalive pings only from `client.loop()`. The default keepalive is 15 s, and the broker drops a client it hasn't heard from in 1.5 × keepalive (22.5 s). Readings are published every 30 s, so the broker was likely dropping the connection between most publishes, and the board reconnected nearly every cycle.

Effects:
- Constant TCP connection churn. On the ESP8266, closed connections take a while to be cleaned up and use memory in the meantime.
- If the board never sees the broker's disconnect, `client.connected()` keeps returning true and `client.publish()` fails. The return value was ignored, so this was silent.

### 3. Discovery configs were not retained

`publish_discover_sensor()` / `publish_discover_motion()` published the `homeassistant/.../config` topics without the retain flag, and only once at boot. When Home Assistant or the MQTT broker restarts (updates, host reboot), HA has no config for these sensors and ignores the state messages until the board reboots.

How to tell it was this: the serial log still shows `publish topic: ...` every 30 s, but HA shows nothing, and HA restarted around the time the readings stopped.

### 4. Reconnect loops with no exit (less likely)

`connect_to_wifi()` and `connect_to_mqtt()` loop forever. `WiFi.begin()` is called only once per attempt. If the router or broker has a bad moment, the board can get stuck there. The serial log would show `Connecting to WiFi...` or repeated `failed to connect to broker: <state>`.

### Ruled out / unlikely

- **Memory leak:** none found. The `String` building in the publish path can fragment the heap over time, but it frees what it allocates.
- **`millis()` wraparound (~49.7 days):** the elapsed-time checks use unsigned subtraction, which handles it correctly.

## Changes

All in `sensor/sensor.ino`.

### AHT20 read with a timeout

Added `aht20_read(float *humidity, float *temperature)`, which talks to the sensor (address `0x38`) over `Wire` directly instead of calling `aht.getEvent()`:

1. Sends the measurement trigger (`0xAC 0x33 0x00`).
2. Polls every 20 ms, reading 6 bytes, until the status byte's busy bit clears.
3. Gives up and returns `false` after `AHT20_READ_TIMEOUT_MS` (500 ms), or right away if the trigger write fails.

The raw-to-%RH and raw-to-°C conversion is the same as the Adafruit library's. If the read fails, `loop()` skips that pass (no publish) and tries again after 1 s.

`aht.begin()` is still used in `setup()` for the sensor's reset/calibration. Its hang risk is covered by the software watchdog.

### `client.loop()`

Called at the top of every `loop()` pass (about every 1.3 s), so keepalive pings go out on time and the connection stays up between publishes.

### Retained discovery configs

Both discovery functions now call `client.publish(topic, payload, true)`. The broker keeps the configs and hands them to HA whenever it (re)subscribes.

### Software watchdog

- A `Ticker` calls `soft_watchdog_tick()` once a second, which counts up and calls `ESP.reset()` past `SOFT_WATCHDOG_S` (180 s).
- `soft_watchdog_feed()` sets the count back to zero. It is called at the end of `setup()`, on the no-publish path of `loop()`, and after a publish.
- The ticker starts at the beginning of `setup()`, so hangs during sensor startup, Wi-Fi connect, or MQTT connect are covered too.
- A failed AHT20 read does **not** reset the count. If the sensor stays unreadable for 3 minutes, the board resets, which also reinitializes the I2C bus.
- This also gives the reconnect loops (finding 4) a way out.

### Diagnostics

- Boot prints `reset reason: ...` (`ESP.getResetReason()`). A watchdog reset shows as a software/system restart rather than power-on.
- A failed state publish now logs `publish failed, mqtt state: <n>` (see the PubSubClient `state()` codes).

## Verifying

- Compile and upload from the Arduino IDE. These changes were not compiled when written (no `arduino-cli` was available).
- Watch the serial monitor and confirm the temperature and humidity readings look the same as before, since the AHT20 read code is new.
- In an MQTT tool (for example MQTT Explorer), confirm the `homeassistant/sensor/.../config` topics show as retained.
- Leave it running for a few days. If it reboots, the `reset reason` line plus the log just before the reboot show which step hung.

## Possible follow-ups

- Put both sensors on one pair of pins (their addresses, `0x38` and `0x10`, don't conflict) to stop switching the bus on every loop pass.
- Subscribe to `homeassistant/status` and republish discovery when HA sends `online`.
- Log `ESP.getFreeHeap()` each loop to confirm memory stays flat.
