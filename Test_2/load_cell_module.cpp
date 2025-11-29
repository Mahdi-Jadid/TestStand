#include "load_cell_module.h"


Stand_Loadcell::Stand_Loadcell(int DT, int SCK) : HX711_DT{DT}, HX711_SCK{SCK}, LoadCell(HX711_DT, HX711_SCK), calibration_value_eeprom_address(0) {}

void Stand_Loadcell::start() {

        LoadCell.begin();
        //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
        float calibrationValue = 0.0f;
        #if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512); 
        #endif
        EEPROM.get(calibration_value_eeprom_address, calibrationValue); 
        LoadCell.start(2000, true);
        if (LoadCell.getTareTimeoutFlag()) {
            Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
            while (1);
        }
        else {
            LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
            Serial.println("Startup is complete");
        }
}

void Stand_Loadcell::update() {
    if (!LoadCell.update()) return;

    int NUM_SAMPLES = 10;
    float sum = 0.0f;

    for (int i = 0; i < NUM_SAMPLES; i++)
        sum += LoadCell.getData();

    thrust = sum / (float) NUM_SAMPLES;
    
}
 
float Stand_Loadcell::get_thrust() {
    return thrust;
}