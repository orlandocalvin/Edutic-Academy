#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
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
LiquidCrystal_I2C lcd(0x27, 16, 2);

char lastTemp[8] = "-";
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
      mqtt.subscribe(TOPIC_TEMP);
      mqtt.subscribe(TOPIC_HUMID);
      mqtt.subscribe(TOPIC_JSON);
      mqtt.subscribe(TOPIC_RELAY1);
      mqtt.subscribe(TOPIC_RELAY2);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" | retry in 5s");
      delay(5000);
    }
  }
}

// Handle Incoming Messages 
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
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      Serial.print("-> Temp: ");
      Serial.print((const char*)doc["temperature"]);
      Serial.print(" °C | Hum: ");
      Serial.print((const char*)doc["humidity"]);
      Serial.println(" %");
      strncpy(lastTemp, (const char*)doc["temperature"], sizeof(lastTemp) - 1);
      lastTemp[sizeof(lastTemp) - 1] = '\0';
    } else {
      Serial.println("-> JSON parse error");
    }
  }

  Serial.println();

  // ADD: support plain temperature topic (fallback)
  if (strcmp(topic, TOPIC_TEMP) == 0) {
  strncpy(lastTemp, msg, sizeof(lastTemp) - 1);
  lastTemp[sizeof(lastTemp) - 1] = '\0';
  }

  // ADD: relay status topics
  if (strcmp(topic, TOPIC_RELAY1) == 0) {
    relay1 = (strcmp(msg, "1") == 0);
  }
  if (strcmp(topic, TOPIC_RELAY2) == 0) {
    relay2 = (strcmp(msg, "1") == 0);
  }

  // ADD: refresh LCD after any update
  updateLCD();
}

void handleTouchToggle() {
  uint16_t v1 = touchRead(TOUCH1);
  uint16_t v2 = touchRead(TOUCH2);
  uint32_t now = millis();

  bool p1 = (v1 <= THRESH1);  // pressed when below threshold
  bool p2 = (v2 <= THRESH2);

  // Channel 1: detect rising edge
  if (p1 != pressed1 && (now - lastBounce1) >= DEBOUNCE_MS) {
    pressed1 = p1;
    lastBounce1 = now;
    if (pressed1) { // press edge
      relay1State = !relay1State;                    // toggle
      mqtt.publish(TOPIC_RELAY1, relay1State ? "1" : "0");
      Serial.print("Relay1 <- "); Serial.println(relay1State ? "1" : "0");
      // if you use LCD vars: relay1 = relay1State; updateLCD();
    }
  }

  // Channel 2: detect rising edge
  if (p2 != pressed2 && (now - lastBounce2) >= DEBOUNCE_MS) {
    pressed2 = p2;
    lastBounce2 = now;
    if (pressed2) { // press edge
      relay2State = !relay2State;                    // toggle
      mqtt.publish(TOPIC_RELAY2, relay2State ? "1" : "0");
      Serial.print("Relay2 <- "); Serial.println(relay2State ? "1" : "0");
      // if you use LCD vars: relay2 = relay2State; updateLCD();
    }
  }
}

void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Suhu: ");
  lcd.print(lastTemp);
  lcd.print("   "); // clear tail

  lcd.setCursor(0, 1);
  lcd.print("R1: ");
  lcd.print(relay1 ? "ON " : "OFF");
  lcd.print(", R2: ");
  lcd.print(relay2 ? "ON " : "OFF");
  lcd.print(" ");  // clear tail
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Simple MQTT Subscriber + Touch Control ===");
  setup_wifi();
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting data...");
  lcd.setCursor(0, 1);
  lcd.print("R1: ---, R2: ---");
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();
  handleTouchToggle();
  delay(10);
}