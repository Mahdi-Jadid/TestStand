#include "pid_module.h"

// Constructor
Stand_PID::Stand_PID(double Kp, double Ki, double Kd)
    : current_pid(&current_input, &current_output, &current_setpoint, Kp, Ki, Kd, DIRECT),
      PENALTY_COEFFICIENT{0.002f},
      LOOP_DT{0.1f},
      LED_PIN{10},
      nearCount(0),
      farCount(0),
      FAR_REQUIRED(5)  // nombre de cycles pour déclencher unlock (0.5 s)
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

// Initialize PID and averages
void Stand_PID::start(Stand_ACS* stand_acs) {

  current_long_time_average = current_instant;

  current_input  = current_windowed_average;
  current_setpoint = 0.0;

  current_instant  = stand_acs->get_current_instant();
  current_windowed_average = update_windowed_average(current_instant);

  current_pid.SetOutputLimits(0, ESC_MAX_ANGLE - ESC_MIN_ANGLE);
  current_pid.SetSampleTime(100);
  current_pid.SetMode(AUTOMATIC);
}

// Reset lock flags
void Stand_PID::reset_bools() {
    pwmLocked = false;
    nearCount = 0;
    preSeek   = false;
    digitalWrite(LED_PIN, LOW);
}

// Update instantaneous and windowed current
void Stand_PID::update_current_stats(Stand_ACS* stand_acs) {
  current_instant  = stand_acs->get_current_instant();
  current_windowed_average = update_windowed_average(current_instant);

  const float LT_ALPHA = 0.98;
  current_long_time_average = LT_ALPHA * current_long_time_average
                            + (1.0 - LT_ALPHA) * current_instant;
}

// ---------------------- MAIN LOOP ----------------------
void Stand_PID::monitor_and_compute(Stand_ESC* stand_esc, Stand_ACS* stand_acs, Stand_Loadcell* stand_loadcell) {

   now_ms = millis();

   update_current_stats(stand_acs);

  // ---------------------- Setpoint zero → idle ----------------------
  if (current_setpoint <= 0.0) {
    stand_esc->write(ESC_MIN_ANGLE);
    reset_bools();
    return;
  }

  // ---------------------- Not locked → PID control ----------------------
  else if (!pwmLocked) {

    // ----- FAST PRE-SEEK PHASE -----
    if (preSeek) {
      if (current_windowed_average < (current_setpoint - 0.30))  // marge rapide pour atteindre le setpoint
        stand_esc->increment_throttle_by(5);
      else {
         preSeek = false;
         current_output =  stand_esc->get_throttle_angle() - ESC_MIN_ANGLE;
      }
    }
    else {
      current_input = (double)current_windowed_average;
      current_pid.Compute();

      int new_angle = (int)(ESC_MIN_ANGLE + current_output);
      if (new_angle < ESC_MIN_ANGLE) new_angle = ESC_MIN_ANGLE;
      if (new_angle > ESC_MAX_ANGLE) new_angle = ESC_MAX_ANGLE;

      stand_esc->write(new_angle);

      // ---------------------- LOCK CONDITION ----------------------
      // Ajuster sensibilité : seuil proche setpoint et nombre de cycles
      if (window_count == WINDOW_SIZE &&
          current_windowed_average >= (current_setpoint - 0.2) && // ↑ tolérance pour lock (moins sensible aux fluctuations)
          current_windowed_average <= current_setpoint)
      {
         nearCount++;
        if (nearCount >= 6) {  // ↑ nombre de cycles pour lock stable
             pwmLocked   = true;
             lockedAngle = stand_esc->get_throttle_angle();
             farCount = 0;
             digitalWrite(LED_PIN, HIGH);
         }
      } else {
         nearCount = 0;
      }
    }
  }

  // ---------------------- LOCKED ----------------------
  else {
    // ---------------------- UNLOCK CONDITION (Hysteresis) ----------------------
    // Ajuster sensibilité : seuil bas et nombre de cycles pour relancer PID
    if (current_windowed_average < (current_setpoint - 0.2)) { // ↑ tolérance pour petites chutes → moins sensible
        farCount++;
       if (farCount >= 10) {  // ↑ cycles à attendre pour relocker → plus stable
            pwmLocked = false;
           preSeek   = true;  // relance PID FAST SEEK
            farCount = 0;
            digitalWrite(LED_PIN, LOW);
            if (max_log > 0)
                if (log_count >= max_log)
                    log_count = 0;
            return;
       }
    } else {
        farCount = 0; // reset compteur si stable
    }

    // Keep output at locked value
    stand_esc->write(lockedAngle);
    loadcell_thrust = stand_loadcell->get_thrust();
  }

// ---------------------- PENALTY INTEGRAL (pourcentage) ----------------------
if (pwmLocked && current_setpoint > 0.0) {
    float over = current_windowed_average - (float)current_setpoint;

    // Commence à 100%
    penalty_points = 100.0;

    if (over > 0) {
       penalty_integral += (double)over * (double)LOOP_DT;  // accumulation A·s
       penalty_points -= PENALTY_COEFFICIENT * penalty_integral; // soustraction
       
       if (penalty_points < 0.0) penalty_points = 0.0; // clamp à 0%
    }
}
}
// ---------------------- WINDOWED AVERAGE ----------------------
float Stand_PID::update_windowed_average(float new_value) {

  if (window_count < WINDOW_SIZE) {
     values_in_window[window_count] = new_value;
     window_sum += new_value;
     window_count++;
  } else {
     window_sum -= values_in_window[window_index];
     values_in_window[window_index] = new_value;
     window_sum += new_value;
     window_index = (window_index + 1) % WINDOW_SIZE;
  }

  return window_sum / (float)window_count;
}

// ---------------------- SETPOINT FROM SERIAL ----------------------
void Stand_PID::set_setpoint_from_serial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    double sp = line.toFloat();
    if (sp >= 0.0) {
      current_setpoint   = sp;
      pwmLocked  = false; 
      nearCount  = 0;
      preSeek    = true;

      Serial.print("New current setpoint: ");
      Serial.print(current_setpoint, 2);
      Serial.println(" A");
    }
    else
      Serial.println("Invalid current_setpoint");
  }
}

void Stand_PID::set_setpoint(double setpoint) {
  current_setpoint = setpoint;
}

// ---------------------- CSV LOG ----------------------
void Stand_PID::csv_log(unsigned long max_log_count) {
    max_log = max_log_count;

    if (max_log_count != 0 && log_count >= max_log_count && pwmLocked) {
        log_finished = true; // <-- new: indicate logging done
        return;
    }

    if (pwmLocked && log_count < max_log_count) {

        if (log_count == 0) {
            Serial.println();
            Serial.println();
            Serial.println("=============== NEW DATA SET =============");
            Serial.println();
            Serial.println();
        }

        Serial.print(now_ms);
        Serial.print(", ");
        Serial.print(lockedAngle);
        Serial.print(", ");
        Serial.print(current_windowed_average, 3);
        Serial.print(", ");
        Serial.print(current_long_time_average, 3);
        Serial.print(", ");
        Serial.print(loadcell_thrust, 2);
        Serial.print(", ");
        Serial.print(penalty_points, 3);
        Serial.println();

        log_count++;
    }
}

void Stand_PID::csv_log(Stand_Battery * battery) {

  auto voltage = battery->read_voltage();
  auto percent = battery->percentage_3S(voltage); 

  if (pwmLocked) {

    if (log_count == 0) {
        Serial.println();
        Serial.println();
        Serial.println("=============== NEW DATA SET =============");
        Serial.println();
        Serial.println();
    }
      
    Serial.print(now_ms);
    Serial.print(", ");
    Serial.print(lockedAngle);
    Serial.print(", ");
    Serial.print(current_windowed_average, 3);
    Serial.print(", ");
    Serial.print(current_long_time_average, 3);
    Serial.print(", ");
    Serial.print(loadcell_thrust, 2);
    Serial.print(", ");
    Serial.print(penalty_points, 3);
    Serial.println();  // ← déplacé ici

    log_count++;
  }
}


bool Stand_PID::is_locked() {
  return pwmLocked;
}
