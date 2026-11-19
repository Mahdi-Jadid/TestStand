#pragma once

#include <Arduino.h>
#include <Servo.h>

// Les bornes sont maintenant en microsecondes (avant : angles 80 à 180)
#define ESC_MIN_ANGLE 1000
#define ESC_MAX_ANGLE 2000

// NOUVEAU : pas d'incrément/décrément (≈ 10° d'avant) et pas de rampe, en µs
#define THROTTLE_STEP_US 50
#define RAMP_STEP_US 5

// A garder identique à ta valeur actuelle
#define RAMP_DELAY_MS 20

class Stand_ESC {

    public:
        Stand_ESC(int pin = 9);

        void start(bool calibration = false);
        void write(int angle);
        int get_throttle_angle();
        float get_throttle_percentage();
        void increment_throttle_by(int increment);
        void increment_throttle_when_entered(char i, char inByte);
        void decrement_throttle_when_entered(char d, char inByte);
        void stop_throttle_when_entered(char s, char inByte);

    private:
        // NOUVEAU : envoie directement une largeur d'impulsion en microsecondes à l'ESC
        void send_us(int us) { ESC.writeMicroseconds(us); }

        const int ATTACHMENT_PIN;
        int esc_angle;      // désormais en microsecondes
        bool started;
        Servo ESC;
};
