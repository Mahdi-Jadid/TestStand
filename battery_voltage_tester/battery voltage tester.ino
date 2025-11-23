// Analog pin where the voltage divider output is connected
const int VOLTAGE_PIN = A0;

// Voltage divider resistors:
// Battery+ → R1 → A0 → R2 → Battery-
const float R1 = 220000.0;  // 220k ohm (high-side resistor)
const float R2 = 100000.0;  // 100k ohm (low-side resistor)

/*
   Reads battery voltage with averaging to stabilize the value.
   samples = how many analog readings to average.
   Returns the battery voltage in volts.
*/
float readBatteryVoltage(int samples = 20) {
  long sum = 0;

  // Take multiple ADC readings
  for (int i = 0; i < samples; i++) {
    sum += analogRead(VOLTAGE_PIN);
    delay(2);  // small delay for ADC stability
  }

  // Average ADC value
  float raw = sum / (float)samples;

  // Convert raw ADC value to voltage at Arduino A0 (0–5V)
  float vout = raw * (5.0 / 1023.0);

  // Convert voltage at A0 to actual battery voltage using divider formula:
  // Vin = Vout * (R1 + R2) / R2
  float vin = vout * ((R1 + R2) / R2);

  return vin;
}

/*
   Converts 4S battery voltage to an estimated state of charge percentage.
   Uses a typical LiPo voltage curve (non-linear).
*/
float percentage4S(float voltage) {
  float percent;

  if (voltage >= 16.8) percent = 100;    // fully charged
  else if (voltage >= 16.4) percent = 95;
  else if (voltage >= 16.0) percent = 90;
  else if (voltage >= 15.6) percent = 85;
  else if (voltage >= 15.2) percent = 80;
  else if (voltage >= 14.8) percent = 70;
  else if (voltage >= 14.5) percent = 60;
  else if (voltage >= 14.2) percent = 50;
  else if (voltage >= 14.0) percent = 40;
  else if (voltage >= 13.8) percent = 30;
  else if (voltage >= 13.6) percent = 20;
  else if (voltage >= 13.4) percent = 10;
  else percent = 0;                      // dangerously low

  return percent;
}

void setup() {
  Serial.begin(115200);  // start serial output
}

void loop() {
  // Read stabilized battery voltage
  float voltage = readBatteryVoltage();

  // Convert voltage to estimated charge percentage
  float percent = percentage4S(voltage);

  // Print results
  Serial.print("Battery Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V   |   Charge: ");
  Serial.print(percent, 0);
  Serial.println("%");

  delay(500);  // update twice per second
}
