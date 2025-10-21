#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>

// ===== WiFi / MQTT =====
const char* ssid = "SUPER-ORCA";
const char* password = "zxcvbnmv";
const char* mqtt_server = "broker.emqx.io";

const char* TOPIC_RELAY = "kelompok1/relay";
const char* TOPIC_JSON  = "kelompok1/dht";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ===== Relay & interval =====
#define RELAY1 25
#define RELAY2 26
int intervalPengiriman = 5; // seconds
unsigned long prevMillis = 0;

// ===== Modbus (XY-MD02) =====
// RO=RX, DI=TX to MAX485
#define MODBUS_EN_PIN     4
#define MODBUS_RO_PIN    18
#define MODBUS_DI_PIN    19
#define MODBUS_BAUD   9600
#define MODBUS_PARITY SERIAL_8N1
#define MODBUS_SLAVE_ID 1
#define MODBUS_ADDR   0x0001   // start register
#define MODBUS_QTY    2        // temp, hum

ModbusMaster modbus;
float temperature = NAN;
float humidity    = NAN;

//  RS485 DE/RE control 
void modbusPreTransmission() {
  digitalWrite(MODBUS_EN_PIN, HIGH); // TX
  delay(2);                          // allow driver to settle
}
void modbusPostTransmission() {
  delay(2);                          // flush last byte
  digitalWrite(MODBUS_EN_PIN, LOW);  // RX
}

//  WiFi STA connect 
void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to " + String(ssid));

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println(" WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

//  MQTT inbound control 
void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_RELAY) != 0) return;

  // Parse compact JSON payload
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return; // ignore malformed payload

  // Apply relay states (active LOW)
  if (doc.containsKey("relay1")) {
    int v = doc["relay1"];           // 0 or 1
    digitalWrite(RELAY1, v ? LOW : HIGH); // 1->ON, 0->OFF
  }
  if (doc.containsKey("relay2")) {
    int v = doc["relay2"]; 
    digitalWrite(RELAY2, v ? LOW : HIGH);
  }
}

//  Ensure MQTT session 
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("connected");
      mqtt.subscribe(TOPIC_RELAY);
      Serial.println("Subscribed: " + String(TOPIC_RELAY));
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retry in 5s");
      delay(5000);
    }
  }
}

void publish_json() {
  StaticJsonDocument<128> doc;
  doc["temperature"] = temperature;
  doc["humidity"]    = humidity;
  char json[128];
  serializeJson(doc, json);
  mqtt.publish(TOPIC_JSON, json);
}

//  Read XY-MD02 via Modbus 
bool readXYMD02(float& t, float& h) {
  uint8_t status = modbus.readInputRegisters(MODBUS_ADDR, MODBUS_QTY);
  if (status == modbus.ku8MBSuccess) {
    uint16_t r0 = modbus.getResponseBuffer(0);
    uint16_t r1 = modbus.getResponseBuffer(1);
    t = r0 / 10.0f;
    h = r1 / 10.0f;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  // Relays (active LOW)
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);

  // RS485 transceiver control
  pinMode(MODBUS_EN_PIN, OUTPUT);
  digitalWrite(MODBUS_EN_PIN, LOW); // default RX

  // UART for RS485
  Serial2.begin(MODBUS_BAUD, MODBUS_PARITY, MODBUS_RO_PIN, MODBUS_DI_PIN);
  Serial2.setTimeout(1000);

  // Modbus init
  modbus.begin(MODBUS_SLAVE_ID, Serial2);
  modbus.preTransmission(modbusPreTransmission);
  modbus.postTransmission(modbusPostTransmission);

  // Network/MQTT
  setup_wifi();
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);
}

void loop() {
  if (!mqtt.connected()) reconnect();
  mqtt.loop();

  if (millis() - prevMillis >= (unsigned long)intervalPengiriman * 1000UL) {
    if (readXYMD02(temperature, humidity)) {
      publish_json();
    } else {
      Serial.println("Modbus read FAILED\n");
    }
    prevMillis = millis();
  }
}