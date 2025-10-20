#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// WiFi Credentials
const char *ssid     = "your ssid";
const char *password = "your password";

// MQTT Broker & Topics
const char *mqtt_server = "broker.emqx.io";
const char *TOPIC_RELAY1 = "kelompok1/relay1";
const char *TOPIC_RELAY2 = "kelompok1/relay2";
const char *TOPIC_TEMP   = "kelompok1/temperature";
const char *TOPIC_HUMID  = "kelompok1/humidity";
const char *TOPIC_JSON   = "kelompok1/dht";

// Pins & Sensors
#define RELAY1   25      // active LOW
#define RELAY2   26      // active LOW
#define DHTPIN    4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Publish interval (seconds)
const uint32_t SEND_INTERVAL_S = 5;

// (Optional) Auto-control by temperature
const bool     AUTO_MODE = true;
const float    T1_ON  = 30.0;   // relay1 ON if temp >= T1_ON
const float    T2_ON  = 35.0;   // relay2 ON if temp >= T2_ON

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Local states (for change detection)
bool relay1_on = false;
bool relay2_on = false;

unsigned long prevMillis = 0;

// --- Helpers: set relays and publish retained state ---
void applyRelay1(bool on) {
  relay1_on = on;
  digitalWrite(RELAY1, on ? LOW : HIGH);
  mqtt.publish(TOPIC_RELAY1, on ? "on" : "off", true); // retained
  Serial.print("Relay1 "); Serial.println(on ? "ON" : "OFF");
}

void applyRelay2(bool on) {
  relay2_on = on;
  digitalWrite(RELAY2, on ? LOW : HIGH);
  mqtt.publish(TOPIC_RELAY2, on ? "on" : "off", true); // retained
  Serial.print("Relay2 "); Serial.println(on ? "ON" : "OFF");
}

// WiFi Setup 
void setup_wifi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: "); Serial.println(WiFi.localIP());
}

// Reconnect to MQTT 
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT... ");
    String clientId = "ESP32Pub-"; clientId += String(random(0xffff), HEX);
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("connected");
      mqtt.subscribe(TOPIC_RELAY1);
      mqtt.subscribe(TOPIC_RELAY2);

      // Publish current states on connect (source of truth)
      mqtt.publish(TOPIC_RELAY1, relay1_on ? "on" : "off", true);
      mqtt.publish(TOPIC_RELAY2, relay2_on ? "on" : "off", true);
    } else {
      Serial.print("failed, rc="); Serial.print(mqtt.state());
      Serial.println(" | retry in 5s");
      delay(5000);
    }
  }
}

// Handle Incoming Messages 
void callback(char *topic, byte *payload, unsigned int length) {
  static char msg[32];
  length = min(length, (unsigned int)(sizeof(msg) - 1));
  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.print("Topic ["); Serial.print(topic); Serial.print("]: ");
  Serial.println(msg);

  // Accept "on"/"off" from any client (subscriber touch, UI, etc.)
  if (strcmp(topic, TOPIC_RELAY1) == 0) {
    bool reqOn = (strcmp(msg, "on") == 0);
    if (reqOn != relay1_on) applyRelay1(reqOn);
  } else if (strcmp(topic, TOPIC_RELAY2) == 0) {
    bool reqOn = (strcmp(msg, "on") == 0);
    if (reqOn != relay2_on) applyRelay2(reqOn);
  }
}

// Publish numeric topics + JSON
void publishMeasurements(float t, float h) {
  char bufT[8], bufH[8];
  dtostrf(t, 0, 1, bufT);
  dtostrf(h, 0, 1, bufH);

  mqtt.publish(TOPIC_TEMP,  bufT);
  mqtt.publish(TOPIC_HUMID, bufH);

  char json[128];
  JsonDocument doc;
  doc["temperature"] = bufT;
  doc["humidity"]    = bufH;
  serializeJson(doc, json, sizeof(json));
  mqtt.publish(TOPIC_JSON, json);

  Serial.print("Publish T="); Serial.print(bufT);
  Serial.print(" °C, H=");   Serial.print(bufH);
  Serial.println(" %");
}

// Auto control based on temperature thresholds
void autoControlByTemp(float t) {
  if (!AUTO_MODE) return;

  bool wantR1 = (t >= T1_ON);
  bool wantR2 = (t >= T2_ON);

  if (wantR1 != relay1_on) applyRelay1(wantR1);
  if (wantR2 != relay2_on) applyRelay2(wantR2);
}

void setup() {
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY1, HIGH); // default OFF (active LOW)
  digitalWrite(RELAY2, HIGH); // default OFF

  Serial.begin(115200);
  Serial.println("\n=== MQTT Publisher: DHT + Relay Control ===");

  dht.begin();
  setup_wifi();

  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  if (millis() - prevMillis >= SEND_INTERVAL_S * 1000UL) {
    prevMillis = millis();

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (isnan(t) || isnan(h)) {
      Serial.println("DHT read failed");
      return;
    }

    // Optional: drive relays by temperature, then publish readings
    autoControlByTemp(t);
    publishMeasurements(t, h);
  }
}