#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    private:
        //
    protected:
        // pins that won't be changed:
        // DIR DIR PWM pins
        uint8_t
            dir1_pin,
            dir2_pin,
            pwm_pin;
        uint8_t reverse_spd;
        // the current speed of the motor
        int16_t curr_spd;
    public:
        // copy constructor
        Motor ();
        // configures the settings of the pins
        void begin (const uint8_t init_dir1_pin, const uint8_t init_dir2_pin, const uint8_t init_pwm_pin, const uint8_t init_reverse_spd = 0);
        // stops the motor
        void stop ();
        // sets the speed of the motor [-255 : 255]
        int16_t set_spd (int16_t spd);
        //
        int16_t operator= (const int16_t spd);
        //
        int16_t operator+= (const int16_t spd_inc);
        //
        int16_t operator-= (const int16_t spd_dec);
        //
        int16_t operator++ ();
        //
        int16_t operator-- ();
        //
        int16_t get_spd ();
        //
        operator int16_t ();
};

#endif // #ifndef MOTOR_H