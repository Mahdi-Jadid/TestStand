#include <PID_v1.h>
#include <Servo.h>

// ---------- Pins ----------
const int PIN_CURRENT = A0;   // ACS712 output
const int ESC_PIN     = 9;    // ESC signal
const int BUTTON_PIN  = 2;    // safety button (D2 to GND)

// ---------- ESC range (degrees) ----------
const int ESC_MIN_ANGLE = 80;
const int ESC_MAX_ANGLE = 180;

// ---------- ACS712 (30A default) ----------
const float VREF       = 5.0;      // ADC reference voltage
const float ADC_STEPS  = 1023.0;   // 10-bit ADC
const float ACS_SENS   = 0.066;    // V/A (change if 5A or 20A module)

float adcOffset = 0.0;            // measured at startup

float longAvgCurrent = 0.0;       // very long-term smooth average

// ---------- PID variables ----------
double Setpoint, Input, Output;

// Tuning (your values)
double Kp = 1.8;
double Ki = 2.2;
double Kd = 0.2;

PID current_pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Servo ESC;
int   escAngle = ESC_MIN_ANGLE;

// ---------- Moving-average filter (0.5 s at 10 Hz = 5 samples) ----------
const uint8_t WINDOW_SIZE = 5;
float  windowVals[WINDOW_SIZE];
uint8_t windowIndex = 0;
uint8_t windowCount = 0;
float   windowSum   = 0.0;

// ---------- Locking (value seeker) ----------
bool   pwmLocked     = false;
int    lockedAngle   = ESC_MIN_ANGLE;
uint8_t nearCount    = 0;

// fast coarse seek before PID
bool   preSeek       = false;     // NEW: fast ramp phase flag

unsigned long nowMs  = 0;

// ---------- Penalty integration ----------
double penaltyIntegral = 0.0;
double penaltyPoints   = 0.0;
const float PENALTY_COEFF = 0.002;
const float LOOP_DT       = 0.1;   // 10 Hz loop (100 ms)


// ================== Helper functions ==================

void calibrateCurrentOffset()
{
  const int N = 500;
  long sum = 0;

  for (int i = 0; i < N; i++)
  {
    sum += analogRead(PIN_CURRENT);
    delay(2);
  }

  adcOffset = (float)sum / (float)N;
}

float readCurrentInstant()
{
  const uint8_t NUM_SAMPLES = 10;
  long sum_adc = 0;

  for (uint8_t i = 0; i < NUM_SAMPLES; i++)
  {
    sum_adc += analogRead(PIN_CURRENT);
  }

  float avg_adc = sum_adc / (float)NUM_SAMPLES;

  float centered = avg_adc - adcOffset;
  float voltage  = centered * (VREF / ADC_STEPS);
  float amps     = voltage / ACS_SENS;

  if (amps < 0) amps = -amps;
  if (amps < 0.02) amps = 0.0;

  return amps;
}

float updateAverage(float newVal)
{
  if (windowCount < WINDOW_SIZE)
  {
    windowVals[windowCount] = newVal;
    windowSum += newVal;
    windowCount++;
  }
  else
  {
    windowSum -= windowVals[windowIndex];
    windowVals[windowIndex] = newVal;
    windowSum += newVal;
    windowIndex = (windowIndex + 1) % WINDOW_SIZE;
  }

  return windowSum / (float)windowCount;
}

// read new setpoint from Serial
void handleSerialSetpoint()
{
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    double sp = line.toFloat();
    if (sp >= 0.0)
    {
      Setpoint   = sp;
      pwmLocked  = false;
      nearCount  = 0;
      preSeek    = true;      // NEW: whenever setpoint changes, start with fast seek

      Serial.print("New setpoint: ");
      Serial.print(Setpoint, 2);
      Serial.println(" A");
    }
    else
    {
      Serial.println("Invalid setpoint");
    }
  }
}


// ================== Setup ==================
void setup()
{
  Serial.begin(115200);
  pinMode(PIN_CURRENT, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("=== Current PID value seeker ===");
  Serial.println("Hold button on D2 to arm ESC and start calibration...");
  Serial.println("Then type target current in A (e.g. 12.5) in Serial Monitor and press Enter.");
  Serial.println();

  while (digitalRead(BUTTON_PIN) == HIGH)
  {
    delay(50);
  }

  Serial.println("Button pressed. Calibrating current sensor offset...");
  calibrateCurrentOffset();
  Serial.print("ADC offset: ");
  Serial.println(adcOffset);

  ESC.attach(ESC_PIN, 1000, 2000);
  ESC.write(ESC_MIN_ANGLE);
  delay(1000);

  float currentInst  = readCurrentInstant();
  float currentAvg05 = updateAverage(currentInst);

  longAvgCurrent = currentInst;

  Input    = currentAvg05;
  Setpoint = 0.0;

  current_pid.SetOutputLimits(0, ESC_MAX_ANGLE - ESC_MIN_ANGLE);
  current_pid.SetSampleTime(100);
  current_pid.SetMode(AUTOMATIC);

  Serial.println("Ready. Set setpoint via Serial.");
}


// ================== Main loop ==================
void loop()
{
  nowMs = millis();

  handleSerialSetpoint();

  float currentInst  = readCurrentInstant();
  float currentAvg05 = updateAverage(currentInst);  // 0.5 s mean (PID input)
  const float LT_ALPHA = 0.98;
  longAvgCurrent = LT_ALPHA * longAvgCurrent + (1.0 - LT_ALPHA) * currentInst;  // display only

  if (Setpoint <= 0.0)
  {
    escAngle = ESC_MIN_ANGLE;
    ESC.write(escAngle);
    pwmLocked = false;
    nearCount = 0;
    preSeek   = false;
  }
  else if (!pwmLocked)
  {
    // --------- FAST PRE-SEEK PHASE ---------
    if (preSeek)
    {
      // ramp up quickly until we're close to setpoint using currentAvg05
      if (currentAvg05 < (Setpoint - 0.5))      // 0.5 A below target
      {
        escAngle += 2;                          // coarse step
        if (escAngle > ESC_MAX_ANGLE) escAngle = ESC_MAX_ANGLE;
        ESC.write(escAngle);
      }
      else
      {
        // we're close enough, switch to PID fine control
        preSeek = false;
        // initialize PID output around current angle
        Output = escAngle - ESC_MIN_ANGLE;
      }
    }
    // --------- PID FINE CONTROL PHASE ---------
    else
    {
      Input = (double)currentAvg05;             // use 0.5 s avg for PID
      current_pid.Compute();

      double angleD = ESC_MIN_ANGLE + Output;
      if (angleD < ESC_MIN_ANGLE) angleD = ESC_MIN_ANGLE;
      if (angleD > ESC_MAX_ANGLE) angleD = ESC_MAX_ANGLE;

      escAngle = (int)angleD;
      ESC.write(escAngle);

      // lock when avg is just under setpoint
      if (windowCount == WINDOW_SIZE &&
          currentAvg05 >= (Setpoint - 0.05) &&
          currentAvg05 <= Setpoint)
      {
        nearCount++;
        if (nearCount >= 5)   // 0.5 s near target
        {
          pwmLocked   = true;
          lockedAngle = escAngle;
          Serial.println("PWM locked for stable measurement.");
        }
      }
      else
      {
        nearCount = 0;
      }
    }
  }
  else
  {
    // locked: hold ESC, no more PID changes
    ESC.write(lockedAngle);
  }

  // Penalty: only when locked and above setpoint
  if (pwmLocked && Setpoint > 0.0)
  {
    float over = currentAvg05 - (float)Setpoint;
    if (over > 0)
    {
      penaltyIntegral += (double)over * (double)LOOP_DT;  // A·s
      penaltyPoints = PENALTY_COEFF * penaltyIntegral;
    }
  }

  // 5) CSV log:
  // time, escAngle, instCurrent, avg0.5s, longAvg, Setpoint, locked, penaltyPoints
  Serial.print(nowMs);
  Serial.print(", ");
  Serial.print(escAngle);
  Serial.print(", ");
  Serial.print(currentInst, 3);
  Serial.print(", ");
  Serial.print(currentAvg05, 3);
  Serial.print(", ");
  Serial.print(longAvgCurrent, 3);
  Serial.print(", ");
  Serial.print(Setpoint, 3);
  Serial.print(", ");
  Serial.print(pwmLocked ? 1 : 0);
  Serial.print(", ");
  Serial.println(penaltyPoints, 3);

  delay(100);   // 10 Hz
}
