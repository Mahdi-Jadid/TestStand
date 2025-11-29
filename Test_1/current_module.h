#pragma once

#include "ACS712.h"

#define V_REF_UNO 5.0
#define ADC_STEPS_UNO 1023

#define ACS712_5A 185.0
#define ACS712_20A 100.0
#define ACS712_30A 66.0

// Representative class for our test stand's ACS712
class Stand_ACS {

        public:

            // Constructor that that takes the analog pin, and the type of ACS712
            Stand_ACS(uint8_t analog_pin = A1, float type = ACS712_30A); 

            // Does Midpoint and stuff
            void start();
            
            float get_current_instant();

            // Get average current for a specified sampling size; default sample size is 25
            float get_current_averaged(int sampling_size = 25);

            // Get the max current recorded up until the last current average
            float get_max_current();

            // Reset max current (and other fields if needed)
            float reset_when_entered(char r, char inByte);


        private:

            // Constant parameters initialized during creation for the ACS object
            const uint8_t ANALOG_PIN;
            const float TYPE;

            // The recorded max current up until the last current average
            float max_current;

            // Defining a ACS object with the ACS712 library by 
            ACS712 ACS;

           


};
