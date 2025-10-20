// ESP32 Touch Read Helper
// T0=4, T1=0, T2=2, T3=15, T4=13, T5=12, T6=14, T7=27, T8=33, T9=32

// Touch pins to test (edit as needed)
const int TOUCH_PINS[] = {12, 14};  // T5=12, T6=14
const int NUM_PINS = sizeof(TOUCH_PINS) / sizeof(TOUCH_PINS[0]);

// Sampling settings
const int CAL_SAMPLES = 40;   // baseline samples (no touch)
const int READ_SAMPLES = 8;   // average per reading
const unsigned long PRINT_MS = 250;

uint16_t baseline[10];        // store per-pin baseline
unsigned long prevMs = 0;

// Average N samples from touchRead
uint16_t touchAvg(int pin, int n) {
  uint32_t sum = 0;
  for (int i = 0; i < n; i++) {
    sum += touchRead(pin);
    delay(2);
  }
  return sum / n;
}

void calibrate() {
  // Keep hands away from wires during calibration
  for (int i = 0; i < NUM_PINS; i++) {
    baseline[i] = touchAvg(TOUCH_PINS[i], CAL_SAMPLES);
  }

  Serial.println("\n=== Calibration (no touch) ===");
  for (int i = 0; i < NUM_PINS; i++) {
    // Rule of thumb: threshold = baseline - delta (try 20..60)
    int recommend = (baseline[i] > 30) ? baseline[i] - 30 : baseline[i] - 5;
    Serial.print("Pin ");
    Serial.print(TOUCH_PINS[i]);
    Serial.print(" -> baseline=");
    Serial.print(baseline[i]);
    Serial.print(" | try threshold ≈ ");
    Serial.println(recommend);
  }
  Serial.println("==============================\n");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Touch Monitor ===");
  calibrate();
  Serial.println("Touch the wire to see raw changes. Lower value = more touch.\n");
}

void loop() {
  if (millis() - prevMs >= PRINT_MS) {
    prevMs = millis();

    // Print raw averages per pin
    for (int i = 0; i < NUM_PINS; i++) {
      uint16_t val = touchAvg(TOUCH_PINS[i], READ_SAMPLES);
      Serial.print("T");
      Serial.print(TOUCH_PINS[i]);
      Serial.print('=');
      Serial.print(val);
      Serial.print(" (base ");
      Serial.print(baseline[i]);
      Serial.print(')');
      if (i < NUM_PINS - 1) Serial.print(" | ");
    }
    Serial.println();
  }
}