/**
    The Script For Test 1 (Measuring Thrust, Current And Time For A Given Air Speed)

    Date 24 November 2025

    Mahdi Ibne Kamal Jadid

*/

#include "pid_module.h"

Stand_ESC stand_esc(9);
Stand_ACS stand_acs(A1, ACS712_30A);
Stand_Battery stand_battery(A0, 220, 100);

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

  stand_pid.set_setpoint(15);

  Serial.println("Ready.");
  
}

// --------- Main loop ---------
void loop() {  

  stand_pid.monitor_and_compute(&stand_esc, &stand_acs, &stand_loadcell);

  stand_loadcell.update();

  stand_pid.csv_log(&stand_battery);

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

