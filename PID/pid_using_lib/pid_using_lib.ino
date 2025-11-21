#include <PID_v1.h>

// I didnt test this code and the values for kp,ki and kd are not tuned. Also please fix bugs if any because I dont know how well it works.
// Use lib - PID by Brett Beauregard
#define PIN_INPUT A0

#include <Servo.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=100.0, Ki=100.0, Kd=100.0;
PID current_pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Servo ESC;

int pwm_us = 0;
float current = 0;

unsigned long global_time;

bool started = false;

void setup() {

  ESC.attach(9, 1000, 2000);
  
  current = read_current();
  Input = (double) current;
  Setpoint = 15;

  //turn the PID on
  current_pid.SetMode(AUTOMATIC);
}

void loop() {
  
   if(!started) {
    ESC.writeMicroseconds(1500); // Writing the middle value for pwm
    started = true;
   }

   pwm_us = generate_corrected_pwm();
   ESC.writeMicroseconds(pwm);
   current = read_current();
   
   print_CSV_style();

}

float read_current() {
  return ( ( analogRead(PIN_INPUT)* 5.0/1023 ) - 2.5 ) / 0.066;
}


int generate_corrected_pwm() {
  Input = analogRead(PIN_INPUT);
  global_time = millis();
  current_pid.Compute();

  return (int) (1000 + Output);
}


void print_CSV_style() {

  Serial.println();
  Serial.print(pwm_us);
  Serial.print(", ");
  Serial.print(current);
  Serial.print(", ");
  Serial.println(global_time);

}

