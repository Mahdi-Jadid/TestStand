#include "battery_module.h"

Stand_Battery::Stand_Battery(int analog_pin, float r1, float r2): R1{r1 * 1000}, R2{r2 * 1000}, BATTERY_ANALOG_PIN{analog_pin} {}

float Stand_Battery::read_voltage(int samples = 20) {
  long sum = 0;

  // Take multiple ADC readings
  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_ANALOG_PIN);
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


float Stand_Battery::percentage_4S(float voltage) {
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



/*void loop() {
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
*/