#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// WiFi Credentials
const char *ssid = "your ssid";
const char *password = "your password";

// MQTT Broker & Topics
const char *mqtt_server = "broker.emqx.io";
const char *TOPIC_TEMP  = "kelompok1/temperature";
const char *TOPIC_HUMID = "kelompok1/humidity";
const char *TOPIC_JSON  = "kelompok1/dht";
const char *TOPIC_RELAY1 = "kelompok1/relay1";
const char *TOPIC_RELAY2 = "kelompok1/relay2";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// LCD setup (I2C address bisa berbeda, biasanya 0x27 atau 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Touch Pins (ESP32)
#define TOUCH1 12  // T5
#define TOUCH2 14  // T6

// Touch Thresholds & Debounce
const uint16_t THRESH1 = 500;
const uint16_t THRESH2 = 500;
const uint32_t DEBOUNCE_MS = 80;

// Relay states
bool state1 = false, state2 = false;
uint32_t last1 = 0, last2 = 0;

// Sensor data
String temperature = "--";

// === WiFi Setup ===
void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// === MQTT Reconnect ===
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT... ");
    String clientId = "ESP32Sub-";
    clientId += String(random(0xffff), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("connected");
      mqtt.subscribe(TOPIC_TEMP);
      mqtt.subscribe(TOPIC_HUMID);
      mqtt.subscribe(TOPIC_JSON);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" | retry in 5s");
      delay(5000);
    }
  }
}

// === MQTT Callback ===
void callback(char *topic, byte *payload, unsigned int length) {
  static char msg[256];
  length = min(length, (unsigned int)(sizeof(msg) - 1));
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.print("Topic [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  // Parse JSON message (only for DHT topic)
  if (strcmp(topic, TOPIC_JSON) == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (!error) {
      temperature = String((const char*)doc["temperature"]);
      Serial.print("-> Temp: ");
      Serial.print(temperature);
      Serial.println(" °C");
    } else {
      Serial.println("-> JSON parse error");
    }
  }

  // Update LCD after receiving new data
  updateLCD();
}

// === Handle Touch ===
void handleTouch() {
  uint16_t v1 = touchRead(TOUCH1);
  uint16_t v2 = touchRead(TOUCH2);
  uint32_t now = millis();

  bool t1 = (v1 <= THRESH1);
  bool t2 = (v2 <= THRESH2);

  if (t1 != state1 && (now - last1) >= DEBOUNCE_MS) {
    state1 = t1;
    last1 = now;
    mqtt.publish(TOPIC_RELAY1, state1 ? "1" : "0");
    Serial.print("Relay1 <- ");
    Serial.println(state1 ? "1" : "0");
    updateLCD();
  }

  if (t2 != state2 && (now - last2) >= DEBOUNCE_MS) {
    state2 = t2;
    last2 = now;
    mqtt.publish(TOPIC_RELAY2, state2 ? "1" : "0");
    Serial.print("Relay2 <- ");
    Serial.println(state2 ? "1" : "0");
    updateLCD();
  }
}

// === LCD Update Function ===
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("R1:");
  lcd.print(state1 ? "ON " : "OFF");
  lcd.print(" R2:");
  lcd.print(state2 ? "ON" : "OFF");
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Subscriber + Touch + LCD ===");

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");

  setup_wifi();

  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

  lcd.clear();
  lcd.print("MQTT Connecting...");
}

// === Loop ===
void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();
  handleTouch();

  // Auto reconnect WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) setup_wifi();

  delay(10);
}
