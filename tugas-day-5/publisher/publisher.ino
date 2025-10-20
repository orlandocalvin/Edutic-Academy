#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

const char *ssid = "your wifi";
const char *password = "your password";

float temperature;
float humidity;
char bufferTemp[5];
char bufferHum[5];
unsigned long prevMillis;

const char *mqtt_server = "broker.emqx.io";
const char *TOPIC_RELAY1 = "kelompok1/relay1";
const char *TOPIC_RELAY2 = "kelompok1/relay2";
const char *TOPIC_TEMP = "kelompok1/temperature";
const char *TOPIC_HUMID = "kelompok1/humidity";
const char *TOPIC_JSON = "kelompok1/dht";

int intervalPengiriman = 5; // satuan detik

#define RELAY1 25
#define RELAY2 26
#define DHTPIN 4
#define DHTTYPE DHT11

WiFiClient espClient;
PubSubClient mqtt(espClient);
DHT dht(DHTPIN, DHTTYPE);

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Terhubung");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Pesan Masuk dari topic [");
  Serial.print(topic);
  Serial.print("] : ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Relay1
  if (strcmp(topic, TOPIC_RELAY1) == 0)
  {
    if ((char)payload[0] == 'on')
    {
      digitalWrite(RELAY1, LOW); // Relay aktif LOW
      Serial.println("Relay1 ON");
    }
    else
    {
      digitalWrite(RELAY1, HIGH);
      Serial.println("Relay1 OFF");
    }
  }

  // Relay2
  if (strcmp(topic, TOPIC_RELAY2) == 0)
  {
    if ((char)payload[0] == 'on')
    {
      digitalWrite(RELAY2, LOW);
      Serial.println("Relay2 ON");
    }
    else
    {
      digitalWrite(RELAY2, HIGH);
      Serial.println("Relay2 OFF");
    }
  }
}

void reconnect()
{
  while (!mqtt.connected())
  {
    Serial.print("Attempting MQTT connection...");

    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (mqtt.connect(clientId.c_str()))
    {
      Serial.println("connected!");
      mqtt.subscribe(TOPIC_RELAY1);
      mqtt.subscribe(TOPIC_RELAY2);
      Serial.println("Subscribe ke topic relay1 & relay2");
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" Reconnect dalam 5 detik");
      delay(5000);
    }
  }
}

void publish_data()
{
  sprintf(bufferTemp, "%.1f", temperature);
  mqtt.publish(TOPIC_TEMP, bufferTemp);

  sprintf(bufferHum, "%.1f", humidity);
  mqtt.publish(TOPIC_HUMID, bufferHum);
}

void publish_json()
{
  char json[256];
  JsonDocument doc;

  sprintf(bufferTemp, "%.1f", temperature);
  sprintf(bufferHum, "%.1f", humidity);

  doc["temperature"] = bufferTemp;
  doc["humidity"] = bufferHum;

  serializeJson(doc, json);
  mqtt.publish(TOPIC_JSON, json);
}

void setup()
{
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY1, HIGH); // default OFF
  digitalWrite(RELAY2, HIGH); // default OFF

  Serial.begin(115200);
  setup_wifi();
  dht.begin();
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);
}

void loop()
{
  if (!mqtt.connected())
  {
    reconnect();
  }
  mqtt.loop();

  if (millis() - prevMillis >= intervalPengiriman * 1000)
  {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    Serial.println("Temperature: " + String(temperature));
    Serial.println("Humidity: " + String(humidity));
    Serial.println();

    publish_json();
    publish_data();
    prevMillis = millis();
  }
  delay(5);
}