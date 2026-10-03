#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "Adafruit_VEML7700.h"
#include <Adafruit_AHTX0.h>

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

// whether motion sensor is available
#define HAS_PIR 1

/////////////////////////////////////////////////////////////////

// whether or not to blink the led on the node mcu board.
#define ENABLE_BLINK 0

#define I2C_MASTER 0x42

#define AHTX0_I2C_CLOCK 100000

#define AHTX0_SDA_PIN 12
#define AHTX0_SCL_PIN 14

#define VEML7700_I2C_CLOCK 10000

#define VEML7700_SDA_PIN 5
#define VEML7700_SCL_PIN 4

#define PIR_PIN 13
#define WIFI_STATUS_PIN 16

// number of milliseconds between publish events
#define PUBLISH_MS 30000

volatile int _isr_motion_flag = 0;

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
  
    pub_response = client.publish(publish_topic.c_str(), json_publish.c_str());
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
  
    pub_response = client.publish(publish_topic.c_str(), json_publish.c_str());
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
  sensors_event_t humidity, temp;
  String json_publish;
  int need_to_publish;
  unsigned long loop_start_time;

  loop_start_time = millis();

#if ENABLE_BLINK
  if (last_led_time == 0 || millis() - last_led_time > 1000) {
    digitalWrite(LED_BUILTIN, led_inv_state);
    last_led_time = millis();
    led_inv_state = !led_inv_state;
  }
#endif
  
  aht_take_wire();
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  temperature_f = (temp.temperature * 1.8) + 32;
  Serial.printf("humidity: %f, temperature c: %f, temperature f: %f\n", humidity.relative_humidity, temp.temperature, temperature_f);

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
  json_publish = "{ \"humidity\":" + String(humidity.relative_humidity, 2) + ", " +
    "\"temperature\":" + String(temp.temperature, 2) + ", " +
    "\"temperature_f\":" + String(temperature_f, 2) + ", " +
    "\"lux_als\":" + raw_als + ", " +
    "\"lux_white\":" + raw_white + ", " +
    "\"lux\":" + String(lux, 2) + ", " +
    "\"occupancy_isr\":" + (_isr_motion_flag ? "true" : "false") + ", " +
    "\"occupancy\":" + (motion_pin_value ? "true" : "false") + "}";
#else
  json_publish = "{ \"humidity\":" + String(humidity.relative_humidity, 2) + ", " +
    "\"temperature\":" + String(temp.temperature, 2) + ", " +
    "\"temperature_f\":" + String(temperature_f, 2) + ", " +
    "\"lux_als\":" + raw_als + ", " +
    "\"lux_white\":" + raw_white + ", " +
    "\"lux\":" + String(lux, 2) + "}";
#endif

#if HOME_ASSISTANT_ENABLE
  Serial.printf("publish topic: %s\n", sensor_topic_state.c_str());
  client.publish(sensor_topic_state.c_str(), json_publish.c_str());
#endif

  last_publish_time = millis();

  delay(500);
}
