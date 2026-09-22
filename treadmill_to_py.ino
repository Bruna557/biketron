#define HALL_PIN 13

// Use 'volatile' for any variable modified in an ISR
volatile uint32_t pulseCount = 0;
// uint32_t lastReportedCount = 0;

void IRAM_ATTR hallISR() {
  pulseCount++;
}

void setup() {
  // Use INPUT_PULLUP to keep the line HIGH until the reed switch pulls it to GND
  pinMode(HALL_PIN, INPUT_PULLUP);
  
  // FALLING means the code triggers exactly once when GND is touched
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  Serial.begin(115200);
  Serial.println("ESP32 Treadmill Sensor Ready...");
}

void loop() {
  // Print the count in the loop, NOT the ISR
  // if (pulseCount != lastReportedCount) {
  //   lastReportedCount = pulseCount;
  //   Serial.print("Pulses: ");
  //   Serial.println(lastReportedCount);
  // }

  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // If Python sends the character 'R' (Request)
    if (command == 'R') {
      // Send data in a clean, CSV-like format: pulses,uptime_ms
      Serial.print(pulseCount);
      Serial.print(",");
      Serial.println(millis());
    }
    
    // Optional: Send 'C' to clear/reset the counter
    if (command == 'C') {
      noInterrupts(); // Protection while resetting volatile variable
      pulseCount = 0;
      interrupts();
      Serial.println("ACK:RESET");
    }
  }
}
