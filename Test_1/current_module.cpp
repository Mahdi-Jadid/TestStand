#include <Arduino.h>
#include "current_module.h"

Stand_ACS::Stand_ACS(uint8_t analog_pin, float type)
    : ANALOG_PIN{analog_pin},
      TYPE{type},
      max_current(0.0f),
      ACS(ANALOG_PIN, V_REF_UNO, ADC_STEPS_UNO, TYPE) {}

void Stand_ACS::start() {
    // A appeler a courant nul (moteur arrete)
    ACS.autoMidPoint();
    Serial.print("ACS MidPoint = ");
    Serial.println(ACS.getMidPoint());
}

float Stand_ACS::get_current_instant() {
    return get_current_averaged(10);
}

float Stand_ACS::get_current_averaged(int sampling_size) {

    if (sampling_size <= 0) sampling_size = 1;  // evite la division par zero

    long sum_mA = 0;

    for (int i = 0; i < sampling_size; i++) {
        sum_mA += ACS.mA_DC();  // mA, peut etre legerement +/- autour de 0
        delay(2);
    }

    float avg_mA = sum_mA / (float)sampling_size;
    float avg_A  = avg_mA / 1000.0f;  // en Amperes

    // Le courant DC du moteur ne doit pas etre negatif
    if (avg_A < 0) avg_A = -avg_A;

    // Seuil de bruit : un pas ADC vaut ~0.78 A avec cette pince (6.25 mV/A sur Uno)
    if (avg_A < NOISE_FLOOR_A) avg_A = 0.0f;

    if (max_current < avg_A) max_current = avg_A;

    return avg_A;
}

float Stand_ACS::get_max_current() {
    return max_current;
}

void Stand_ACS::reset_when_entered(char r, char inByte) {
    if (inByte == r) {
        max_current = 0.0f;
        Serial.println("Max current reset.");
    }
}

// ============== PID RELATED ============== //
