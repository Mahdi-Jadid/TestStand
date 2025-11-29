#pragma once
#include <Arduino.h>

class Stand_Battery {

    public:

        /* (analog_pin from A0 to A5, 
            the high-side resistance in kilo ohms, 
            the low-side resistance in kilo ohms)    */

        Stand_Battery(int analog_pin, float r1, float r2);

        /*
            Reads battery voltage with averaging to stabilize the value.
            samples = how many analog readings to average.
            Returns the battery voltage in volts (20 by default)   */

        float read_voltage(int samples = 20);

        /*
            Converts 4S battery voltage to an estimated state of charge percentage.
            Uses a typical LiPo voltage curve (non-linear).    */

        float percentage_4S(float voltage);


    
    private:

    // Voltage divider resistors:
    // Battery+ → R1 → A0 → R2 → Battery-

     const float R1;  // 220k ohm (high-side resistor)
     const float R2;  // 100k ohm (low-side resistor)
     const float BATTERY_ANALOG_PIN;



};