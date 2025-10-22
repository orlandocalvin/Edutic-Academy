#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <DHT.h>

// === Hardware Pins ===
#define DHTPIN        15
#define DHTTYPE       DHT11
#define RELAY1_PIN    25
#define RELAY2_PIN    26
#define TOUCH1_GPIO   13
#define TOUCH2_GPIO   14

// === Modbus Address Map ===
#define COIL_RELAY1           1
#define COIL_RELAY2           2
#define TEMPERATURE_ADDRESS 100
#define HUMIDITY_ADDRESS    101

// === Wi-Fi Credentials ===
const char* WIFI_SSID = "SUPER-ORCA";
const char* WIFI_PASS = "zxcvbnmv";

// === Touch Handling ===
const uint16_t TOUCH_THRESHOLD = 800;  // pressed if touchRead() < this
const uint16_t RELEASE_MS      = 120;  // release must be stable this long (ms)
const uint16_t SCAN_MS         = 20;   // non-blocking scan period (ms)

// Latch states
bool touch1Latched = false;
bool touch2Latched = false;
unsigned long releaseStart1 = 0;
unsigned long releaseStart2 = 0;
unsigned long lastScan = 0;

// Rate limit for DHT
const uint32_t DHT_PERIOD_MS = 1500;
uint32_t lastDhtMs = 0;

// === Globals ===
DHT dht(DHTPIN, DHTTYPE);
ModbusIP mb;

// === Helpers ===
static inline bool isTouchPressed(uint8_t touchPin) { // returns true when finger is present
  return touchRead(touchPin) < TOUCH_THRESHOLD;
}

// handle a single touch channel
static inline void handleTouchChannel(bool &latched, bool touchNow, unsigned long &releaseStart, uint16_t coilId, unsigned long now) {
  if (!latched && touchNow) {
    // Rising edge: toggle coil
    latched = true;
    mb.Coil(coilId, !mb.Coil(coilId));
    releaseStart = 0;
  } else if (latched && !touchNow) {
    // Finger released — start release timer
    if (releaseStart == 0) releaseStart = now;
    if (now - releaseStart >= RELEASE_MS) {
      latched = false;          // Confirm release
      releaseStart = 0;
    }
  } else {
    // Still pressed: reset timer (avoid flicker)
    releaseStart = 0;
  }
}

// Handle touch input & toggle coil state
void handleTouch() {
  unsigned long now = millis();
  if (now - lastScan < SCAN_MS) return;
  lastScan = now;

  // Read touch sensors
  bool touch1Now = isTouchPressed(TOUCH1_GPIO);
  bool touch2Now = isTouchPressed(TOUCH2_GPIO);

  // Process each channel
  handleTouchChannel(touch1Latched, touch1Now, releaseStart1, COIL_RELAY1, now);
  handleTouchChannel(touch2Latched, touch2Now, releaseStart2, COIL_RELAY2, now);
}

// Apply coil state to relay GPIO (active-LOW)
void handleRelay() {
  digitalWrite(RELAY1_PIN, mb.Coil(COIL_RELAY1) ? LOW : HIGH);
  digitalWrite(RELAY2_PIN, mb.Coil(COIL_RELAY2) ? LOW : HIGH);
}

// Read DHT11 Every 1.5s
// Update Modbus holding registers (scaled x10)
void handleSensor() {
  uint32_t now = millis();
  if (now - lastDhtMs < DHT_PERIOD_MS) return;
  lastDhtMs = now;

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (!isnan(humidity) && !isnan(temperature)) {
    mb.Hreg(TEMPERATURE_ADDRESS, (int16_t)(temperature * 10));
    mb.Hreg(HUMIDITY_ADDRESS,    (int16_t)(humidity * 10));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nMODBUS TCP OVER WIFI - ESP32"));

  // WiFi Connect
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to " + String(WIFI_SSID));
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println(" Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  dht.begin(); // DHT Setup

  // Relay Outputs
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);

  // Start Modbus TCP server & Expose registers
  mb.server();
  mb.addHreg(TEMPERATURE_ADDRESS);
  mb.addHreg(HUMIDITY_ADDRESS);
  mb.addCoil(COIL_RELAY1);
  mb.addCoil(COIL_RELAY2);
  mb.Coil(COIL_RELAY1, 0);
  mb.Coil(COIL_RELAY2, 0);
}

void loop() {
  mb.task();        // handle Modbus communication
  handleTouch();    // read touch and update coil states
  handleRelay();    // apply coil states to relays
  handleSensor();   // read DHT11 and update Hregs
}