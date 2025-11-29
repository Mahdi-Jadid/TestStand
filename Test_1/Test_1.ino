/**
    The Script For Test 1 (Measuring Thrust, Current And Time For A Given Air Speed)

    Date 24 November 2025

    Mahdi Ibne Kamal Jadid

*/

#include "pid_module.h"

Stand_ESC stand_esc(9);
Stand_ACS stand_acs(A1, ACS712_30A);

 // Tuning
        double Kp = 1.9; // 1.8
        double Ki = 2.5; // 2.2
        double Kd = 0.2; // 0.2

       

Stand_PID stand_pid(Kp, Ki, Kd);

Stand_Loadcell stand_loadcell;

// --------- Setup ---------
void setup()
{
  Serial.begin(115200);

  wait_on_pullup_button();
  
  stand_esc.start();

  stand_loadcell.start();

  stand_acs.start();

  stand_pid.start(&stand_acs);

  stand_pid.set_setpoint(0.6);

  Serial.println("Ready.");
  
}

// --------- Main loop ---------
void loop() {  

  

  stand_pid.update_time();

  stand_pid.pid_logic(&stand_esc, &stand_acs, &stand_loadcell);

  stand_loadcell.update();

  if (!stand_pid.is_locked()) {
    Serial.println();
    Serial.print(stand_acs.get_current_instant());
    Serial.print(", ");
    Serial.print(stand_esc.get_throttle_angle());
    Serial.println();
  }
  stand_pid.csv_log(250UL);

  delay(100);
}



void wait_on_pullup_button() {

  // Safety button on D2 (only used during setup)
  const uint8_t BUTTON_PIN = 2;

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // button to GND

  // ---------- SAFETY: wait for button BEFORE touching the ESC ----------
  Serial.println("Hold safety button to arm ESC and start calibration...");
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(50);
  }

  Serial.println("Safety button pressed, attaching ESC...");

}

