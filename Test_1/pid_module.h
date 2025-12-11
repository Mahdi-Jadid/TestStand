#pragma once

#include <PID_v1.h>
#include "current_module.h"
#include "esc_module.h"
#include "battery_module.h"
#include "load_cell_module.h"


class Stand_PID {

    public: 

        Stand_PID(double Kp, double Ki, double Kd);

        void start(Stand_ACS* stand_acs);

        void set_setpoint(double setpoint);

        void set_setpoint_from_serial();

        void monitor_and_compute (Stand_ESC* stand_esc, Stand_ACS* stand_acs, Stand_Loadcell* stand_loadcell);

        bool is_locked();

        void csv_log(unsigned long max_log_count = 0);

        void csv_log(Stand_Battery * battery);

        float update_windowed_average(float new_value);

        float current_windowed_average = 0.0f;

        bool log_finished = false;    
        unsigned long led_last_toggle = 0;
        const unsigned long LED_BLINK_INTERVAL = 200; // ms
        const int LED_PIN;

        private:

            void reset_bools();

            int nearCount;
            int farCount;
            const int FAR_REQUIRED;

            void update_current_stats(Stand_ACS* stand_acs);

            double current_input; 
            double current_output;
            double current_setpoint;

            PID current_pid;

           // ---------- Moving-average filter (0.5 s at 10 Hz = 5 samples) ----------
                static constexpr uint8_t WINDOW_SIZE = 5;
                float values_in_window[WINDOW_SIZE] = {};   // zero-initialize
                uint8_t window_index = 0;
                uint8_t window_count = 0;
                float window_sum = 0.0f;

                // ---------- Locking (value seeker) ----------
                bool pwmLocked = false;
                int lockedAngle = ESC_MIN_ANGLE;

                // fast coarse seek before PID
                bool preSeek = true;    

                unsigned long now_ms = 0;

                // ---------- Penalty integration ----------
                double penalty_integral = 0.0;
                double penalty_points = 0.0;
                const float PENALTY_COEFFICIENT;
                const float LOOP_DT;   // 10 Hz loop (100 ms)

                float current_instant = 0.0f;
                
                float current_long_time_average = 0.0f;

                float loadcell_thrust = 0.0f;

                unsigned long log_count = 0;

                unsigned long max_log = 0;
};

