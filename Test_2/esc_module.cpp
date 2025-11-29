#include "esc_module.h"



Stand_ESC::Stand_ESC(int pin = 9) : ATTACHMENT_PIN{pin}, esc_angle(ESC_MIN_ANGLE), started(false)  {}

void Stand_ESC::start(bool calibration = false) {

    ESC.attach(ATTACHMENT_PIN, 1000, 2000);

     // Immediately send minimum throttle
     write(ESC_MIN_ANGLE);          // 1000 µs
     delay(700);

   

    if (!started)
        ESC.write(esc_angle);

    started = true;
}

void Stand_ESC::write(int angle) {

    if (angle > ESC_MAX_ANGLE) angle = ESC_MAX_ANGLE;
    if (angle < ESC_MIN_ANGLE) angle = ESC_MIN_ANGLE;
    esc_angle = angle;
    ESC.write(esc_angle);
}

int Stand_ESC::get_throttle_angle() {
    return esc_angle;
}

float Stand_ESC::get_throttle_percentage() {

  float pct = (float)((esc_angle - ESC_MIN_ANGLE) * 100.0 / (ESC_MAX_ANGLE - ESC_MIN_ANGLE));
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;

  return pct;
}

void Stand_ESC::increment_throttle_by(int increment) {
  esc_angle += increment;                          // coarse step
  if (esc_angle > ESC_MAX_ANGLE) esc_angle = ESC_MAX_ANGLE;
  write(esc_angle);
}

void Stand_ESC::increment_throttle_when_entered(char i, char inByte) {

    if (inByte == i) {
      // increase throttle
      esc_angle += 10;
      if (esc_angle > 180) esc_angle = 180;
      for (int i = esc_angle - 9; i <= esc_angle; i++) {
        if (i < 80) continue;
        ESC.write(i);
        delay(RAMP_DELAY_MS);
      }

      Serial.println(esc_angle);
    }
  
}   

void Stand_ESC::decrement_throttle_when_entered(char d, char inByte) {

    if (inByte == d) {
      // decrease throttle
     esc_angle -= 10; 
      if (esc_angle < 80) esc_angle= 80; 
      for (int i = esc_angle+ 9; i >= esc_angle; i--) {
        if (i < 80) break;
        ESC.write(i);
        delay(RAMP_DELAY_MS);
      }

      Serial.println(esc_angle);
    } 
}

void Stand_ESC::stop_throttle_when_entered(char s, char inByte) {

    if (inByte == s) {
      // smooth stop back to idle (80)
      for (int i = esc_angle; i >= 80; i--) {
        ESC.write(i);
        delay(RAMP_DELAY_MS);
      }
     esc_angle= 80;

     Serial.println(esc_angle);
    }
}
