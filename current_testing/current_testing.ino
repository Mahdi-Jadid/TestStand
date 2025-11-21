#include "ACS712.h"
#include <Servo.h>

//  Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
//  ACS712 5A  uses 185 mV per A
//  ACS712 20A uses 100 mV per A
//  ACS712 30A uses  66 mV per A

// Define ACS712 and ESC(/Servo) objects
ACS712  ACS(A1, 5.0, 1023, 66); // --------------> (ANALOG_PIN_ON_UNO, MAX_LOGIC_VOLTS, MAX_ADC_STEPS, mV/A_OF_ACS)
Servo ESC;

// Flags related to ESC
bool started = false; //-----------> Whether the ESC has already started or not (i.e. Uno wrote a pwm > minimum to start the motor, which in this case is 90 raw pwm).
int val = 80; //-----------> raw value of pwm to be written to the ESC.

void setup()
{
  Serial.begin(115200);

  ESC.attach(9, 1000, 2000); // -----------> use the digital pin 9 (pwm) on Uno
  delay(400);

  // You can remove this probably because it doesnt seem to work 

    /*-->*/   // Auto Calibration of ESC starts here
    /*-->*/ 
    /*-->*/   int signal_value = 180; // --------> writing the maximum pwm to ESC at start puts it into calibration mode
    /*-->*/   ESC.write(signal_value);
    /*-->*/   delay(500);
    /*-->*/   signal_value = 0; // ----------> giving the minimum afterwards (within 2s) sets the minimum of the ESC (Note it didnt seem to work... the min seems to be 90 anyways)
    /*-->*/   ESC.write(signal_value);
    /*-->*/   delay(1500);

          // Auto Calibration ends here


  // Some code decorative code that came with the ACS712 example I was using
  while (!Serial);                
  Serial.println(__FILE__); 
  Serial.print("ACS712_LIB_VERSION: ");
  Serial.println(ACS712_LIB_VERSION);
  // upto this point

  // Set the midpoint of the acs readings... I tried with or without this code but didnt help with fixing the ACS712 readings.
  ACS.autoMidPoint();
  Serial.println(ACS.getMidPoint());
}


void loop()
{  
   
  // Start the ESC by writing 10 less than minimum of pwm to ESC (to keep it static until user wants to), then ignore for rest of the runtime

  if(!started) {
   ESC.write(80);
   started = true;
  }

  // _____________________________________________________________________________________________


  // Observe the following user input cases (via Serial Monitor) for the rest of the runtime: 
  // 'i' for increment -> increases raw pwm by 10 and writes to ESC
  // 'd' for decrement -> decreases raw pwm by 10 and writes to ESC
  // 's' for stop -> reduces the raw pwm to 80 (which is MINIMUM - 10) to bring it to rest.

  // NOTE: These all involve gradual change/write of pwm step by step (as shown in the code) to avoid sudden jumps of abrupt pwm changes
   
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 'i') {
      
     val += 10;
     if (val > 180) val = 180;
      for (int i = val - 9; i <= val; i++) {
        ESC.write(i);
        delay(500);
      }
      
    }
    else if (inByte == 'd') {
      val -= 10; 
      if(val<80) val = 80; 

      for (int i = val+9; i >= val; i--) {
        ESC.write(i);
        delay(500);
      }
      
      
      } 
    else if (inByte == 's') {
      
       for (int i = val; i >= 80; i --) {
           if (val == 80) break;
          ESC.write(i);
          delay(500);
       }}
  }

  // Supposed to print the values of current drawn for a given steady state of the motor (NB: Doesn't work!!!) 
  Serial.println();

  float a= ACS.mA_DC() / 1000;
  Serial.println(a);
  
  delay(100);
}


//  -- END OF FILE --
