#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ====== WiFi Credentials ======
const char *ssid     = "SUPER-ORCA";
const char *password = "zxcvbnmv";

// ====== MQTT Broker & Topics ======
const char *mqtt_server = "broker.emqx.io";
const char *TOPIC_TEMP  = "kelompok1/temperature";
const char *TOPIC_HUMID = "kelompok1/humidity";
const char *TOPIC_JSON  = "kelompok1/dht";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// --- WiFi Setup ---
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

// --- Reconnect to MQTT ---
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

// --- Handle Incoming Messages ---
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
    } else {
      Serial.println("-> JSON parse error");
    }
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Simple MQTT Subscriber ===");
  setup_wifi();
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();
}