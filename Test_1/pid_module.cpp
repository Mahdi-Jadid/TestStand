#include "pid_module.h"



Stand_PID::Stand_PID(double Kp, double Ki, double Kd)
    : current_pid(&current_input, &current_output, &current_setpoint, Kp, Ki, Kd, DIRECT),
      PENALTY_COEFFICIENT(0.002f),
      LOOP_DT(0.1f),
      BUZZER_PIN(10)
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}


void Stand_PID::start(Stand_ACS* stand_acs) {

  current_long_time_average = current_instant;

  current_input  = current_windowed_average;
  current_setpoint = 0.0;

  current_instant  = stand_acs->get_current_instant();
  current_windowed_average =  update_windowed_average(current_instant);

  current_pid.SetOutputLimits(0, ESC_MAX_ANGLE - ESC_MIN_ANGLE); 
  current_pid.SetSampleTime(100);
  current_pid.SetMode(AUTOMATIC);
}

void Stand_PID::reset_bools() {
    pwmLocked = false;
    nearCount = 0;
    preSeek   = false;
}

void Stand_PID::update_current_stats(Stand_ACS* stand_acs) {
  current_instant  = stand_acs->get_current_instant();
  current_windowed_average = update_windowed_average(current_instant);  // 0.5 s mean (PID   current_input)

  const float LT_ALPHA = 0.98;
  current_long_time_average = LT_ALPHA * current_long_time_average + (1.0 - LT_ALPHA) * current_instant;  // display only
}

void Stand_PID:: monitor_and_compute (Stand_ESC* stand_esc, Stand_ACS* stand_acs, Stand_Loadcell* stand_loadcell) {

   now_ms = millis();

   update_current_stats(stand_acs);

  if (current_setpoint <= 0.0) {
    stand_esc->write(ESC_MIN_ANGLE);
    reset_bools();
    return;
  }

  else if (!pwmLocked) {
    // --------- FAST PRE-SEEK PHASE ---------
    if (preSeek) {
      if (current_windowed_average < (current_setpoint - 0.20))  // 0.5 A below target
        stand_esc->increment_throttle_by(5);
      else {
         preSeek = false;
         current_output =  stand_esc->get_throttle_angle() - ESC_MIN_ANGLE;
      }
    }
    else {

    //  if (current_windowed_average < (current_setpoint - 0.50)) {
      //   preSeek = true;
        // return;
      //}
        

      current_input = (double)current_windowed_average;             // use 0.5 s avg for PID
      current_pid.Compute();

      int new_angle = (int)(ESC_MIN_ANGLE +   current_output);
      if (new_angle < ESC_MIN_ANGLE) new_angle = ESC_MIN_ANGLE;
      if (new_angle > ESC_MAX_ANGLE) new_angle = ESC_MAX_ANGLE;

     
       stand_esc->write(new_angle);

      // lock when avg is just under current_setpoint
      if ( window_count == WINDOW_SIZE &&
          current_windowed_average >= (current_setpoint - 0.05) &&
          current_windowed_average <= current_setpoint)
      {
         nearCount++;
        if ( nearCount >= 5) {   // 0.5 s near target 
          pwmLocked   = true;
          lockedAngle =  stand_esc->get_throttle_angle();
          Serial.println("PWM locked for stable measurement.");
        }
      }
      else
         nearCount = 0;
    }
  }
  else {
    if  (current_windowed_average < (current_setpoint - 0.05)) { 
       pwmLocked = false;
       return;
    }

    stand_esc->write(lockedAngle);
    loadcell_thrust = stand_loadcell->get_thrust();
     
  }

  if (pwmLocked && current_setpoint > 0.0) {
    float over = current_windowed_average - (float)  current_setpoint;
    if (over > 0) {
       penalty_integral += (double)over * (double) LOOP_DT;  // A·s
       penalty_points =  PENALTY_COEFFICIENT *  penalty_integral;
    }
  }
}

float Stand_PID::update_windowed_average(float new_value) {

  if (window_count <  WINDOW_SIZE) {
     values_in_window[window_count] = new_value;
     window_sum += new_value;
     window_count++;
  }
  else {
     window_sum -=  values_in_window[window_index];
     values_in_window[window_index] = new_value;
     window_sum += new_value;
     window_index = ( window_index + 1) %  WINDOW_SIZE;
  }

  return  window_sum / (float) window_count;
}

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
      preSeek    = true;      // whenever current_setpoint changes, start with fast seek

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

void Stand_PID::csv_log(unsigned long max_log_count) {

  if (max_log_count != 0)
    if (log_count >= max_log_count && pwmLocked) {
        digitalWrite(BUZZER_PIN, HIGH); // Buzzer will sound if data for one air speed is done logging and no further logs collected
        return;
    }

  // 5) CSV log:
  
  if (pwmLocked) {
   Serial.print(now_ms);
  Serial.print(", ");
  Serial.print(lockedAngle);
  Serial.print(", ");
  Serial.print(current_instant, 3);
  Serial.print(", ");
  Serial.print(current_windowed_average, 3);
  Serial.print(", ");
  Serial.print(current_long_time_average, 3);
 // Serial.print(", ");
  //Serial.print(current_setpoint, 3);
  //Serial.print(", ");
 // Serial.print( pwmLocked ? 1 : 0);
  //Serial.print(", ");
  //Serial.print(penalty_points, 3);

    // Serial.print(", ");
    // Serial.print(loadcell_thrust, 2);
     log_count++;
  }

  Serial.println();


}

bool Stand_PID::is_locked() {
  return pwmLocked;
}