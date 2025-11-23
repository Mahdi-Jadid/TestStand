#include "ACS712.h"
#include <Servo.h>

ACS712  ACS(A1, 5.0, 1023, 66);
Servo ESC;

// Safety button on D2 (only used during setup)
const uint8_t BUTTON_PIN = 2;

// ESC control
bool started = false;
int  val     = 80;   // 80..180

// Current tracking
float maxCurrent = 0.0;   // in Amps

// --------- Helpers ---------
float readCurrentAveraged()
{
  const uint8_t NUM_SAMPLES = 25;
  long sum_mA = 0;

  for (uint8_t i = 0; i < NUM_SAMPLES; i++)
  {
    sum_mA += ACS.mA_DC();  // mA, can be slightly ± around 0
    delay(2);
  }

  float avg_mA = sum_mA / (float)NUM_SAMPLES;
  float avg_A  = avg_mA / 1000.0;  // to Amps

  // Make sure it's positive (motor DC should not go negative)
  if (avg_A < 0) avg_A = -avg_A;

  // Small noise floor: kill anything below ~0.05 A
  if (avg_A < 0.05) avg_A = 0.0;

  return avg_A;
}

// Map current "val" (80..180) to throttle % (0..100)
float throttlePercent()
{
  const int VAL_MIN = 80;
  const int VAL_MAX = 180;

  float pct = (val - VAL_MIN) * 100.0 / (VAL_MAX - VAL_MIN);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// --------- Setup ---------
void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // button to GND

  // ---------- SAFETY: wait for button BEFORE touching the ESC ----------
  Serial.println("Hold safety button to arm ESC and start calibration...");
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(50);
  }
  Serial.println("Safety button pressed, attaching ESC...");

  // Now attach ESC (no PWM before this)
  ESC.attach(9, 1000, 2000);

  // Immediately send minimum throttle
  ESC.write(0);          // 1000 µs
  delay(700);

  // Optional: ESC calibration (may spin motor!)
  Serial.println("Calibrating ESC...");
  int signal_value = 180;   // max pwm
  ESC.write(signal_value);
  delay(500);
  signal_value = 0;         // min pwm
  ESC.write(signal_value);
  delay(1500);

  // ACS712 info
  while (!Serial);
  Serial.println(__FILE__); 
  Serial.print("ACS712_LIB_VERSION: ");
  Serial.println(ACS712_LIB_VERSION);

  ACS.autoMidPoint();
  Serial.print("ACS MidPoint = ");
  Serial.println(ACS.getMidPoint());

  // Start with zero max current
  maxCurrent = 0.0;
}

// --------- Main loop ---------
void loop()
{  
  // Start the ESC once by writing low pwm (keep motor stopped)
  if (!started) {
    ESC.write(80);   // your idle/stop angle
    started = true;
  }

  // ----- Serial control for ESC -----
  if (Serial.available() > 0) {
    char inByte = Serial.read();

    const int RAMP_DELAY_MS = 50;   // faster transitions

    if (inByte == 'i') {
      // increase throttle
      val += 10;
      if (val > 180) val = 180;
      for (int i = val - 9; i <= val; i++) {
        if (i < 80) continue;
        ESC.write(i);
        delay(RAMP_DELAY_MS);
      }
    }
    else if (inByte == 'd') {
      // decrease throttle
      val -= 10; 
      if (val < 80) val = 80; 
      for (int i = val + 9; i >= val; i--) {
        if (i < 80) break;
        ESC.write(i);
        delay(RAMP_DELAY_MS);
      }
    } 
    else if (inByte == 's') {
      // smooth stop back to idle (80)
      for (int i = val; i >= 80; i--) {
        ESC.write(i);
        delay(RAMP_DELAY_MS);
      }
      val = 80;
    }
    else if (inByte == 'r') {
      // reset max current
      maxCurrent = 0.0;
      Serial.println("Max current reset.");
    }
  }

  // ----- Current measurement & max tracking -----
  float I = readCurrentAveraged();     // averaged, cleaned up
  if (I > maxCurrent) {
    maxCurrent = I;
  }

  float thr = throttlePercent();

  // Print: current, max, throttle %
  Serial.print("I = ");
  Serial.print(I, 2);
  Serial.print(" A  | Imax = ");
  Serial.print(maxCurrent, 2);
  Serial.print(" A  | Throttle = ");
  Serial.print(thr, 1);
  Serial.println(" %");

  delay(100);
}