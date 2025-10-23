#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// === Pins & ADC config ===
#define TDS_PIN     35
#define PH_PIN      34
#define VREF        3.3f
#define ADC_RES     4095.0f

// === TDS parameters ===
#define TDS_FACTOR  0.5f         // 0.5 for NaCl solution
float temperatureC = 25.0f;      // TODO: replace with real water temp sensor

// === LCD Object ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === Working buffers ===
int   tdsSamples[10];
int   phSamples[34];

float adcAvgTDS, voltTDS, volt25C, ec_mS, tds_ppm;
float phValue;

// Take 10 ADC samples → sort → drop extremes → average
float trimmedAverageADC_TDS(int pin) {
  for (int i = 0; i < 10; i++) {
    tdsSamples[i] = analogRead(pin);
    delay(30); // short delay between reads
  }
  // simple ascending sort
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (tdsSamples[i] > tdsSamples[j]) {
        int tmp = tdsSamples[i];
        tdsSamples[i] = tdsSamples[j];
        tdsSamples[j] = tmp;
      }
    }
  }
  // remove 2 lowest & 2 highest
  long sum = 0;
  for (int i = 2; i < 8; i++) sum += tdsSamples[i];
  return (float)sum / 6.0f;
}

// Read pH using 34 samples → sort → average middle 30 → linear calibration
float readPH_Calibrated(int pin) {
  for (int i = 0; i < 34; i++) {
    phSamples[i] = analogRead(pin);
    delay(30);
  }
  for (int i = 0; i < 33; i++) {
    for (int j = i + 1; j < 34; j++) {
      if (phSamples[i] > phSamples[j]) {
        int tmp = phSamples[i];
        phSamples[i] = phSamples[j];
        phSamples[j] = tmp;
      }
    }
  }
  unsigned long avg = 0;
  for (int i = 2; i < 32; i++) avg += phSamples[i]; // keep middle 30
  float avgADC = (float)avg / 30.0f;

  // simple linear calibration
  // from provided formula: pH = (avgADC - 5284.0) / -296.0
  return (avgADC - 5284.0f) / -296.0f;
}

void setup() {
  Serial.begin(9600);

  // ADC config
  analogReadResolution(12);                 // set ADC to 12-bit
  analogSetPinAttenuation(TDS_PIN, ADC_11db);
  analogSetPinAttenuation(PH_PIN,  ADC_11db);

  // LCD init
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {
  //  TDS: ADC → Voltage → Temp compensation → EC → PPM 
  adcAvgTDS = trimmedAverageADC_TDS(TDS_PIN);       // clean ADC reading
  voltTDS   = adcAvgTDS * (VREF / ADC_RES);         // ADC → voltage

  float tempCoef = 1.0f + 0.02f * (temperatureC - 25.0f);
  volt25C = voltTDS / tempCoef;                     // temperature compensation

  // empirical polynomial for common TDS modules (see module docs)
  ec_mS = (133.42f * volt25C * volt25C * volt25C)
        - (255.86f * volt25C * volt25C)
        + (857.39f * volt25C);

  tds_ppm = ec_mS * TDS_FACTOR;                     // EC → TDS (ppm)

  //  pH: multi-sample average + linear calibration 
  phValue = readPH_Calibrated(PH_PIN);

  //  Serial output
  Serial.print("TDS: ");
  Serial.print(tds_ppm, 1);
  Serial.print(" ppm  | pH: ");
  Serial.println(phValue, 2);

  // Line 1: pH
  lcd.setCursor(0, 0);
  lcd.print("pH: ");
  lcd.print(phValue, 2);
  lcd.print("       "); // pad spaces to clear residual chars

  // Line 2: TDS (ppm)
  lcd.setCursor(0, 1);
  lcd.print("TDS: ");
  lcd.print(tds_ppm, 1);
  lcd.print(" ppm   ");

  delay(1000);
}