#include <Servo.h>

#define ESC_MAX_ANGLE 180
#define ESC_MIN_ANGLE 80
#define RAMP_DELAY_MS 50

class Stand_ESC {

    public:

        // ESC object wrapper for our stand
        Stand_ESC(int pin);

        // Setup the ESC with the option to calibrate (false by default)
        void start(bool calibration = false);

        // Write an angle to the ESC
        void write(int angle);

        // Get the raw angle value for ESC
        int get_throttle_angle();

        // Get the percentage of throttle on ESC
        float get_throttle_percentage();

        void increment_throttle_by(int increment = 2);

        // Increment angle value smoothly when the specified char (default 'i') is entered into the Serial Monitor
        void increment_throttle_when_entered(char i = 'i', char inByte);

        // Decrement angle value smoothly when the specified char (default 'd') is entered into the Serial Monitor
        void decrement_throttle_when_entered(char d = 'd', char inByte);

        // Reduce the esc angle to minimum smoothly when the specified char (default 's') is entered into the Serial Monitor
        void stop_throttle_when_entered(char s = 's', char inByte);
            
    private:

        bool started;
        int ATTACHMENT_PIN;
        Servo ESC;
        int esc_angle;

 };
