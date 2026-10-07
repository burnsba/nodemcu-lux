#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

#include "Adafruit_VEML7700.h"
#include <Adafruit_AHTX0.h>

/***
The physical board consists of: esp8266 node mcu board, connected to aht20 temperature sensor, and veml7700 light sensor.
optional: PIR motion sensor.

The physical board has two status LEDs: power, wifi connected.

The board is always connected to power over usb.

Sensor readings are taken periodically then published to Home Assistant mqtt over wifi.
*/

/*
  secrets.h: local configuration.
  contains definitions:
      #define CLIENT_MAC
      const char *wifi_ssid
      const char *wifi_password
      const char *mqtt_username
      const char *mqtt_password
*/
#include "secrets.h"

/////////////////////////////////////////////////////////////////
// hardware specific configuration

#define HOME_ASSISTANT_ENABLE 1

// whether motion sensor is available.
// I have two different physical boards, one has a motion sensor, the other doesn't.
#define HAS_PIR 1

/////////////////////////////////////////////////////////////////

// whether or not to blink the led on the node mcu board.
#define ENABLE_BLINK 0

#define I2C_MASTER 0x42

#define AHTX0_I2C_CLOCK 100000

#define AHTX0_SDA_PIN 12
#define AHTX0_SCL_PIN 14

#define AHT20_I2C_ADDR 0x38
// max time to wait for the AHT20 to finish a measurement (datasheet: ~80ms).
#define AHT20_READ_TIMEOUT_MS 500

#define VEML7700_I2C_CLOCK 10000

#define VEML7700_SDA_PIN 5
#define VEML7700_SCL_PIN 4

#define PIR_PIN 13
#define WIFI_STATUS_PIN 16

// number of milliseconds between publish events
#define PUBLISH_MS 30000

// software watchdog: reset the board if loop() hasn't completed in this many seconds.
// The hardware watchdog doesn't catch hangs that call delay() or yield().
#define SOFT_WATCHDOG_S 180

volatile int _isr_motion_flag = 0;

Ticker soft_watchdog_ticker;
volatile unsigned int soft_watchdog_seconds = 0;

Adafruit_AHTX0 aht;
Adafruit_VEML7700 veml = Adafruit_VEML7700();

const int mqtt_port = 1883;


String client_mac = CLIENT_MAC;

// device manufacture, model, and name will show up in Home Assistant.
String device_identifier = "esp8266_" + client_mac; // mac address
String device_manufacturer = "burnsba";
String device_model = "BBNMCU-LUX-V1_2";
String device_name = "lux sensor v1.2";

// MQTT topic to publish events to.
String sensor_topic_state = "esp8266/luxsensorv1/" + client_mac;

// The discovery topic needs to follow a specific format:
// https://www.home-assistant.io/docs/mqtt/discovery/
#if HAS_PIR
String sensor_topic_motion_uid = "r" + client_mac + "_occupancy";
String sensor_topic_motion_config = "homeassistant/binary_sensor/" + sensor_topic_motion_uid + "/config";
String sensor_topic_motion_isr_uid = "r" + client_mac + "_occupancy_isr";
String sensor_topic_motion_isr_config = "homeassistant/binary_sensor/" + sensor_topic_motion_isr_uid + "/config";
#endif
String sensor_topic_temperature_uid = "r" + client_mac + "_temperature";
String sensor_topic_temperature_config = "homeassistant/sensor/" + sensor_topic_temperature_uid + "/config";
String sensor_topic_temperature_f_uid = "r" + client_mac + "_temperature_f";
String sensor_topic_temperature_f_config = "homeassistant/sensor/" + sensor_topic_temperature_f_uid + "/config";
String sensor_topic_humidity_uid = "r" + client_mac + "_humidity";
String sensor_topic_humidity_config = "homeassistant/sensor/" + sensor_topic_humidity_uid + "/config";
String sensor_topic_als_uid = "r" + client_mac + "_als";
String sensor_topic_als_config = "homeassistant/sensor/" + sensor_topic_als_uid + "/config";
String sensor_topic_white_uid = "r" + client_mac + "_white";
String sensor_topic_white_config = "homeassistant/sensor/" + sensor_topic_white_uid + "/config";
String sensor_topic_lux_uid = "r" + client_mac + "_lux";
String sensor_topic_lux_config = "homeassistant/sensor/" + sensor_topic_lux_uid + "/config";

String client_prefix = "esp8266-client-lux-";

unsigned long last_publish_time = 0;
unsigned long last_led_time = 0;
#if HAS_PIR
int last_isr_state = 0;
int last_motion_pin_state = 0;
#endif
int led_inv_state = 0; // LOW = ON

WiFiClient espClient;
PubSubClient client(espClient);

void veml7700_take_wire() {
  Serial.printf("wire: reset for VEML7700\n");
  Wire.begin(VEML7700_SDA_PIN, VEML7700_SCL_PIN, I2C_MASTER);        // join i2c bus (address optional for master)
  delay(100);

  Wire.setClock(VEML7700_I2C_CLOCK);

  delay(100);
}

void veml7700_setup() {
  Serial.printf("wire: reset for VEML7700\n");
  Wire.begin(VEML7700_SDA_PIN, VEML7700_SCL_PIN, I2C_MASTER);        // join i2c bus (address optional for master)
  delay(100);

  Wire.setClock(VEML7700_I2C_CLOCK);

  delay(100);

  veml.begin();
}

void aht20_setup() {
  Serial.printf("wire: reset for AHT20\n");
  Wire.begin(AHTX0_SDA_PIN, AHTX0_SCL_PIN, I2C_MASTER);        // join i2c bus (address optional for master)
  delay(100);

  Wire.setClock(AHTX0_I2C_CLOCK);

  delay(100);

  aht.begin();
}

void aht_take_wire() {
  Serial.printf("wire: reset for AHT\n");
  Wire.begin(AHTX0_SDA_PIN, AHTX0_SCL_PIN, I2C_MASTER);        // join i2c bus (address optional for master)
  delay(100);

  Wire.setClock(AHTX0_I2C_CLOCK);

  delay(100);
}

// Read the AHT20 directly instead of aht.getEvent(). The Adafruit library spins
// forever if an i2c read fails (failed status read returns 0xFF, which has the busy bit set).
// Conversion matches Adafruit_AHTX0::getEvent.
// Returns false on i2c error or timeout.
bool aht20_read(float *humidity, float *temperature) {
  uint8_t data[6];
  unsigned long start_time;
  uint32_t raw;
  int i;

  // trigger measurement
  Wire.beginTransmission(AHT20_I2C_ADDR);
  Wire.write(0xAC);
  Wire.write(0x33);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.printf("aht20: trigger failed\n");
    return false;
  }

  start_time = millis();
  while (1) {
    delay(20);

    if (Wire.requestFrom(AHT20_I2C_ADDR, 6) == 6) {
      for (i = 0; i < 6; i++) {
        data[i] = Wire.read();
      }

      // status byte, bit 7 = busy
      if (!(data[0] & 0x80)) {
        break;
      }
    }

    if (millis() - start_time > AHT20_READ_TIMEOUT_MS) {
      Serial.printf("aht20: read timeout\n");
      return false;
    }
  }

  raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
  *humidity = ((float)raw * 100) / 0x100000;

  raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
  *temperature = ((float)raw * 200 / 0x100000) - 50;

  return true;
}

// Runs once a second from a timer. ESP.reset() is safe to call from timer context.
void soft_watchdog_tick() {
  soft_watchdog_seconds++;
  if (soft_watchdog_seconds > SOFT_WATCHDOG_S) {
    ESP.reset();
  }
}

void soft_watchdog_feed() {
  soft_watchdog_seconds = 0;
}

void connect_to_wifi() {
  int wifi_pin_toggle = 0;

  digitalWrite(WIFI_STATUS_PIN, LOW);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(wifi_ssid, wifi_password);
  wifi_pin_toggle = 1;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (wifi_pin_toggle) {
      digitalWrite(WIFI_STATUS_PIN, HIGH);
      wifi_pin_toggle = 0;
    } else {
      digitalWrite(WIFI_STATUS_PIN, LOW);
      wifi_pin_toggle = 1;
    }
  }

  Serial.print("ip: ");
  Serial.println(WiFi.localIP());

  digitalWrite(WIFI_STATUS_PIN, HIGH);
}

void connect_to_mqtt() {
#if HOME_ASSISTANT_ENABLE
  Serial.println("Connecting to MQTT broker...");
  
  while (!client.connected()) {
    String client_id = client_prefix + String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.print("...connected\n");
    } else {
      Serial.print("failed to connect to broker: ");
      Serial.print(client.state());
      Serial.print("\n");
      delay(2000);
    }
  }
#endif
}

void publish_discover_motion(String append_device_name, String uid, String value_template, String publish_topic) {
#if HOME_ASSISTANT_ENABLE
#if HAS_PIR
  String json_publish;
  int pub_response = 0;

  Serial.print("Enter publish_discover_motion\n");

  while (pub_response == 0) {

    json_publish = String("{\"device\": {") +
        String("\"identifiers\": [\"m" + String(device_identifier) + "\"],") +
        String("\"manufacturer\": \"" + String(device_manufacturer) + "\",") +
        String("\"model\": \"" + String(device_model) + "\",") +
        String("\"name\": \"" + String(device_name) + "\"},") + 
      String("\"device_class\": \"motion\",") +
      String("\"name\": \"" + String(device_name) + " " + String(append_device_name) + "\",") +
      String("\"payload_off\": false,") +
      String("\"payload_on\": true,") +
      String("\"state_topic\": \"" + String(sensor_topic_state) + "\",") +
      String("\"unique_id\": \"" + String(uid) + "\",") +    
      String("\"value_template\": \"{{ " + String(value_template) + " }}\"}");
  
    Serial.printf("config topic: %s\n", publish_topic.c_str());
    Serial.printf("payload:\n");
    Serial.print(json_publish.c_str());
    Serial.print("\n\n");
  
    // retained, so Home Assistant gets the config again after HA or broker restart.
    pub_response = client.publish(publish_topic.c_str(), json_publish.c_str(), true);
    Serial.printf("pub_response: %d\n", pub_response);

    if (pub_response == 0) {
      delay(4000);
    } else {
      break;
    }
  }
#endif
#endif
}

void publish_discover_sensor(String append_device_name, String device_class, String uom, String uid, String value_template, String publish_topic) {
#if HOME_ASSISTANT_ENABLE
  String json_publish;
  int pub_response = 0;

  Serial.print("Enter publish_discover_temp\n");

  while (pub_response == 0) {

    json_publish = String("{\"device\": {") +
        String("\"identifiers\": [\"m" + String(device_identifier) + "\"],") +
        String("\"manufacturer\": \"" + String(device_manufacturer) + "\",") +
        String("\"model\": \"" + String(device_model) + "\",") +
        String("\"name\": \"" + String(device_name) + "\"},") + 
      String("\"device_class\": \"" + String(device_class) + "\",") +
      (uom == "" ? "" : String("\"unit_of_measurement\": \"" + String(uom) + "\",")) +
      String("\"name\": \"" + String(device_name) + " " + String(append_device_name) + "\",") +
      String("\"state_topic\": \"" + String(sensor_topic_state) + "\",") +
      String("\"unique_id\": \"" + String(uid) + "\",") +
      String("\"value_template\": \"{{ " + String(value_template) + " }}\"}");
  
    Serial.printf("config topic: %s\n", publish_topic.c_str());
    Serial.printf("payload:\n");
    Serial.print(json_publish.c_str());
    Serial.print("\n\n");
  
    // retained, so Home Assistant gets the config again after HA or broker restart.
    pub_response = client.publish(publish_topic.c_str(), json_publish.c_str(), true);
    Serial.printf("pub_response: %d\n", pub_response);

    if (pub_response == 0) {
      delay(4000);
    } else {
      break;
    }
  }
#endif
}

void setup() {
  
  Serial.begin(115200);  // start serial for output
  delay(50);

  Serial.print("\n\nboot\n");
  Serial.printf("reset reason: %s\n", ESP.getResetReason().c_str());

  // armed before sensor/wifi/mqtt setup, so a hang during setup also resets.
  soft_watchdog_ticker.attach(1, soft_watchdog_tick);

#if ENABLE_BLINK
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  pinMode(WIFI_STATUS_PIN, OUTPUT);
#if HAS_PIR
  pinMode(PIR_PIN, INPUT_PULLUP);
#endif

  digitalWrite(WIFI_STATUS_PIN, LOW);

  aht20_setup();
  veml7700_setup();

  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  connect_to_wifi();

  // By default, buffer size is 256 bytes which will fail on trying to send config message.
  client.setBufferSize(1024);
  client.setServer(mqtt_broker, mqtt_port);

  connect_to_mqtt();

  delay(5000);

#if HAS_PIR
  publish_discover_motion(String("occupancy"), sensor_topic_motion_uid, String("value_json.occupancy"), sensor_topic_motion_config);
  publish_discover_motion(String("occupancy_isr"), sensor_topic_motion_isr_uid, String("value_json.occupancy_isr"), sensor_topic_motion_isr_config);
#endif
  publish_discover_sensor(String("temperature"), String("temperature"), String("°C"), sensor_topic_temperature_uid, String("value_json.temperature"), sensor_topic_temperature_config);
  publish_discover_sensor(String("temperature_f"), String("temperature"), String("°F"), sensor_topic_temperature_f_uid, String("value_json.temperature_f"), sensor_topic_temperature_f_config);
  publish_discover_sensor(String("humidity"), String("humidity"), String("%rH"), sensor_topic_humidity_uid, String("value_json.humidity"), sensor_topic_humidity_config);

  publish_discover_sensor(String("Ambient Light"), String("illuminance"), String(""), sensor_topic_als_uid, String("value_json.lux_als"), sensor_topic_als_config);
  publish_discover_sensor(String("White Light"), String("illuminance"), String(""), sensor_topic_white_uid, String("value_json.lux_white"), sensor_topic_white_config);
  publish_discover_sensor(String("Lux"), String("illuminance"), String(""), sensor_topic_lux_uid, String("value_json.lux"), sensor_topic_lux_config);

  soft_watchdog_feed();
}

#if HAS_PIR
IRAM_ATTR void movement_detection() {
  _isr_motion_flag = 1;
}
#endif

void loop() {
  float temperature_f;
#if HAS_PIR
  int motion_pin_value;
#endif
  int raw_als;
  int raw_white;
  float lux;
  float humidity;
  float temperature;
  String json_publish;
  int need_to_publish;
  unsigned long loop_start_time;

  loop_start_time = millis();

  // service mqtt keepalive; otherwise the broker drops the connection between publishes.
  client.loop();

#if ENABLE_BLINK
  if (last_led_time == 0 || millis() - last_led_time > 1000) {
    digitalWrite(LED_BUILTIN, led_inv_state);
    last_led_time = millis();
    led_inv_state = !led_inv_state;
  }
#endif
  
  aht_take_wire();
  if (!aht20_read(&humidity, &temperature)) {
    // Skip this round. Watchdog is not fed, so if the sensor stays
    // unreadable the board resets after SOFT_WATCHDOG_S.
    delay(1000);
    return;
  }
  temperature_f = (temperature * 1.8) + 32;
  Serial.printf("humidity: %f, temperature c: %f, temperature f: %f\n", humidity, temperature, temperature_f);

  delay(200);

  veml7700_take_wire();
  raw_als = veml.readALS();
  raw_white = veml.readWhite();
  lux = veml.readLux();
  Serial.printf("als: %d, white: %d, lux: %f\n", raw_als, raw_white, lux);

#if HAS_PIR
  motion_pin_value = digitalRead(PIR_PIN);
  // If there has been no change on the pin, then sleep and watch for new motion.
  // Otherwise, jump straight to publishing.
  if (last_motion_pin_state == motion_pin_value) {
    // enable interrupt on the PIR pin, but only while not communicating on i2c.
    _isr_motion_flag = 0;
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), movement_detection, RISING);
    delay(800);
    if (last_led_time == 0 || millis() - last_led_time > 1000) {
#if ENABLE_BLINK
      digitalWrite(LED_BUILTIN, led_inv_state);
      last_led_time = millis();
      led_inv_state = !led_inv_state;
#endif
    }
    delay(800);
    if (last_led_time == 0 || millis() - last_led_time > 1000) {
#if ENABLE_BLINK
      digitalWrite(LED_BUILTIN, led_inv_state);
      last_led_time = millis();
      led_inv_state = !led_inv_state;
#endif
    }
    detachInterrupt(digitalPinToInterrupt(PIR_PIN));
  }
  
  Serial.printf("_isr_motion_flag: %d, motion_pin_value: %d\n", _isr_motion_flag, motion_pin_value);
#endif

  need_to_publish = (last_publish_time == 0) ||
#if HAS_PIR
    (last_isr_state != _isr_motion_flag) ||
    (last_motion_pin_state != motion_pin_value) ||
#endif
    (millis() - last_publish_time > PUBLISH_MS);

#if HAS_PIR
  last_isr_state = _isr_motion_flag;
  last_motion_pin_state = motion_pin_value;
#endif

  if (!need_to_publish) {
    Serial.printf("need_to_publish: false\n");
    soft_watchdog_feed();
    delay(500);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    delay(1000);
    connect_to_wifi();
  }

  if (!client.connected()) {
    connect_to_mqtt();
  }

#if HAS_PIR
  json_publish = "{ \"humidity\":" + String(humidity, 2) + ", " +
    "\"temperature\":" + String(temperature, 2) + ", " +
    "\"temperature_f\":" + String(temperature_f, 2) + ", " +
    "\"lux_als\":" + raw_als + ", " +
    "\"lux_white\":" + raw_white + ", " +
    "\"lux\":" + String(lux, 2) + ", " +
    "\"occupancy_isr\":" + (_isr_motion_flag ? "true" : "false") + ", " +
    "\"occupancy\":" + (motion_pin_value ? "true" : "false") + "}";
#else
  json_publish = "{ \"humidity\":" + String(humidity, 2) + ", " +
    "\"temperature\":" + String(temperature, 2) + ", " +
    "\"temperature_f\":" + String(temperature_f, 2) + ", " +
    "\"lux_als\":" + raw_als + ", " +
    "\"lux_white\":" + raw_white + ", " +
    "\"lux\":" + String(lux, 2) + "}";
#endif

#if HOME_ASSISTANT_ENABLE
  Serial.printf("publish topic: %s\n", sensor_topic_state.c_str());
  if (!client.publish(sensor_topic_state.c_str(), json_publish.c_str())) {
    Serial.printf("publish failed, mqtt state: %d\n", client.state());
  }
#endif

  last_publish_time = millis();
  soft_watchdog_feed();

  delay(500);
}
