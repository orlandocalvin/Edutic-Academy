// ESP32 Touch Test
// Just test touch pin

void setup() {
  Serial.begin(115200);
  delay(1000);  // give me time to bring up serial monitor
  Serial.println("ESP32 Touch Test");
}

void loop() {
  Serial.println(touchRead(15));  // get value using Pin 15
  delay(1000);
}