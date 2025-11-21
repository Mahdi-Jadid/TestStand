#include <Servo.h>


// I have not tested this code so not sure if it is operational.

const int INPUT_PIN = A0;


double global_time;

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;

Servo ESC;

int pwm_us = 0;

double setpoint_current = 15; // in Amperes

float current = 0;

bool started = false;

void setup() {

  Serial.begin(9600);

  ESC.attach(9, 1000, 2000); // Digital Pin 9 used for ESC pwm signal


  // I kept this values at 100 as guess values because the pwm_us in value is always supposed to be roughly two orders of magnitude higher than the current 
  // so as to make sure that the pid output calculated actually produces a meaningful change in the pwm...

  // Please change them with real, accurate values...
  kp = 100.0;
  ki = 100.0;
  kd = 100.0;


  last_time = 0;
  
  delay(100);
}

void loop() {

  if(!started) {
    ESC.writeMicroseconds(1500); // Writing the middle value for pwm
    started = true;
  }

  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  current = read_current();
  double error = setpoint_current - (double) current;

  pwm_us = generate_corrected_pwm(error);
  ESC.writeMicroseconds(pwm);
   current = read_current();
   global_time = millis();
   
   print_CSV_style();

}

float read_current() {
  return ( ( analogRead(PIN_INPUT)* 5.0/1023 ) - 2.5 ) / 0.066;
}

// I have two layers of functions because I dont know how well this "output" variable maps to the pwm_us required.
// Nonetheless I decided to shift the output by 1000 to match the 1000-2000 microseconds range of pwm_us.

int generate_corrected_pwm(double error) {

  double output = pid(error);
  
  return 1000 + output;
}

double pid(double error) {

  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;

  return (kp * proportional) + (ki * integral) + (kd * derivative);
}


void print_CSV_style() {

  Serial.println();
  Serial.print(pwm_us);
  Serial.print(", ");
  Serial.print(current);
  Serial.print(", ");
  Serial.println(global_time);

}