#include "current_module.h"


Stand_ACS::Stand_ACS(uint8_t analog_pin, float type): ANALOG_PIN{analog_pin}, TYPE{type}, max_current(0.0f), ACS(ANALOG_PIN, V_REF_UNO, ADC_STEPS_UNO, TYPE) {}

void Stand_ACS::start() {
    ACS.autoMidPoint();
    Serial.print("ACS MidPoint = ");
    Serial.println(ACS.getMidPoint());
   
}

float Stand_ACS::get_current_instant() {
    return get_current_averaged(10);
}

float Stand_ACS::get_current_averaged(int sampling_size) {

    long sum_mA = 0;

    for (uint8_t i = 0; i < sampling_size; i++) {
        sum_mA += ACS.mA_DC();  // mA, can be slightly ± around 0
        delay(2);
    }

    float avg_mA = sum_mA / (float)sampling_size;
    float avg_A  = avg_mA / 1000.0;  // to Amps

    // Make sure it's positive (motor DC should not go negative)
    if (avg_A < 0) avg_A = -avg_A;

    // Small noise floor: kill anything below ~0.05 A
    if (avg_A < 0.05) avg_A = 0.0;

    if (max_current < avg_A) max_current = avg_A;

    return avg_A;

}

float Stand_ACS::get_max_current() {
    return max_current;
}

float Stand_ACS::reset_when_entered(char r, char inByte) {
   if (inByte == r) {
     max_current = 0.0;
     Serial.println("Max current reset.");
   }
    
}

// ============== PID RELATED ============== //




    



