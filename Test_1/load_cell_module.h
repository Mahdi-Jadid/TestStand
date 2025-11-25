#pragma once

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

class Stand_Loadcell {

    public:

        Stand_Loadcell(int DT = 4, int SCK = 5);

        void start();

        void update();

        float get_thrust();

    private:

        //pins:
        const int HX711_DT; // 4 
        const int HX711_SCK; // 5

        //HX711 constructor:
        HX711_ADC LoadCell;

        float thrust;

        const int calibration_value_eeprom_address;

};