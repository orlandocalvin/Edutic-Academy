#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

// WiFi Credentials
const char *ssid = "your ssid";
const char *password = "your password";

// MQTT Broker & Topics
const char *mqtt_server = "broker.emqx.io";
const char *TOPIC_JSON  = "kelompok1/dht";
const char *TOPIC_RELAY = "kelompok1/relay";

WiFiClient espClient;
PubSubClient mqtt(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);

char lastTemp[8] = "-";
char lastHum[8] = "-";
bool relay1 = false;
bool relay2 = false;

// ADD: toggle states + edge tracking
bool pressed1 = false, pressed2 = false;     // last raw touch state
bool relay1State = false, relay2State = false; // latched relay states
uint32_t lastBounce1 = 0, lastBounce2 = 0;   // debounce

// Touch Pins (ESP32)
#define TOUCH1 12  // T5
#define TOUCH2 14  // T6

// Simple touch logic (tune these if needed)
const uint16_t THRESH1 = 500;
const uint16_t THRESH2 = 500;
const uint32_t DEBOUNCE_MS = 80;

bool state1 = false, state2 = false;   // last published states
uint32_t last1 = 0, last2 = 0;         // debounce timers

// WiFi Setup 
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

// Reconnect to MQTT 
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT... ");

    String clientId = "ESP32Sub-";
    clientId += String(random(0xffff), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("connected");
      mqtt.subscribe(TOPIC_JSON);
      mqtt.subscribe(TOPIC_RELAY);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" | retry in 5s");
      delay(5000);
    }
  }
}

void publishRelayState(bool r1, bool r2) {
  StaticJsonDocument<128> doc;
  doc["relay1"] = r1 ? 1 : 0;
  doc["relay2"] = r2 ? 1 : 0;
  char payload[64];
  serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(TOPIC_RELAY, payload);
}

// Handle Incoming Messages 
void callback(char *topic, byte *payload, unsigned int length) {
  // copy payload
  static char msg[256];
  length = min(length, (unsigned int)(sizeof(msg) - 1));
  memcpy(msg, payload, length);
  msg[length] = '\0';

  // Sensor JSON from publisher
  if (strcmp(topic, TOPIC_JSON) == 0) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      // temperature: accept string or number
      if (doc["temperature"].is<const char*>()) {
        const char* t = doc["temperature"];
        strncpy(lastTemp, t, sizeof(lastTemp) - 1);
        lastTemp[sizeof(lastTemp) - 1] = '\0';
      } else if (doc["temperature"].is<float>() || doc["temperature"].is<double>() || doc["temperature"].is<long>()) {
        float t = doc["temperature"];
        snprintf(lastTemp, sizeof(lastTemp), "%.0f", t); // round to integer °C
      }

      // humidity: accept string or number
      if (doc["humidity"].is<const char*>()) {
        const char* h = doc["humidity"];
        strncpy(lastHum, h, sizeof(lastHum) - 1);
        lastHum[sizeof(lastHum) - 1] = '\0';
      } else if (doc["humidity"].is<float>() || doc["humidity"].is<double>() || doc["humidity"].is<long>()) {
        float h = doc["humidity"];
        snprintf(lastHum, sizeof(lastHum), "%.0f", h);   // round to integer %
      }

      updateLCD();
    }
    return;
  }

  // Relay control JSON from publisher
  if (strcmp(topic, TOPIC_RELAY) == 0) {
    JsonDocument doc;
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      if (doc.containsKey("relay1")) relay1 = (int)doc["relay1"] == 1;
      if (doc.containsKey("relay2")) relay2 = (int)doc["relay2"] == 1;
      // keep local latched states in sync with network command
      relay1State = relay1;
      relay2State = relay2;
      updateLCD();
    }
    return;
  }
}

void handleTouchToggle() {
  uint16_t v1 = touchRead(TOUCH1);
  uint16_t v2 = touchRead(TOUCH2);
  uint32_t now = millis();

  bool p1 = (v1 <= THRESH1);
  bool p2 = (v2 <= THRESH2);

  // Channel 1 edge
  if (p1 != pressed1 && (now - lastBounce1) >= DEBOUNCE_MS) {
    pressed1 = p1;
    lastBounce1 = now;
    if (pressed1) {                // on press
      relay1State = !relay1State;  // toggle
      relay1 = relay1State;        // sync display var
      publishRelayState(relay1State, relay2State);
      updateLCD();
    }
  }

  // Channel 2 edge
  if (p2 != pressed2 && (now - lastBounce2) >= DEBOUNCE_MS) {
    pressed2 = p2;
    lastBounce2 = now;
    if (pressed2) {
      relay2State = !relay2State;
      relay2 = relay2State;
      publishRelayState(relay1State, relay2State);
      updateLCD();
    }
  }
}

void updateLCD() {
  // Line 1: T: 29°C H: 56%
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(lastTemp);
  lcd.print((char)223);  // degree symbol on HD44780
  lcd.print("C H: ");
  lcd.print(lastHum);
  lcd.print("%  ");      // pad to clear tail

  // Line 2: R1: ON/OFF R2: ON/OFF
  lcd.setCursor(0, 1);
  lcd.print("R1: ");
  lcd.print(relay1 ? "ON " : "OFF");
  lcd.print("R2: ");
  lcd.print(relay2 ? "ON" : "OFF");
  lcd.print("  ");      // pad to clear tail
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MQTT Subscriber ===");
  setup_wifi();
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("T: --"); lcd.print((char)223); lcd.print("C H: --%");
  lcd.setCursor(0, 1); lcd.print("R1: --- R2: ---");
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();
  handleTouchToggle();
  delay(10);
}