#ifndef KOJAY_H
#define KOJAY_H

#include <Arduino.h>
#include <EEPROM.h>

#include "Motor/Motor.h"
#include "CompoI/CompoI.h"
#include "Uts/Uts.h"
#include "Cmpas/Cmpas.h"

#ifndef __AVR_ATmega2560__
#error Arduino Mega not selected
#endif // #ifndef __AVR_ATmega2560__

class Kojay {
    private:
        //
    protected:
        //
    public:
        //
        Motor mtrs [4];
        //
        uint8_t gryscls [4][3];
        //
        CompoI eyes [2];
        //
        Uts uts [4];
        //
        Cmpas cmpas;
        //
        uint8_t buttons [3];
        //
        uint16_t gryscls_thresholds [4][3];
        //
        Kojay ();
        //
        void begin ();
        //
        void begin (const uint8_t m1, const uint8_t m2, const uint8_t m3, const uint8_t m4, const bool m1r, const bool m2r, const bool m3r, const bool m4r, const uint8_t gs1, const uint8_t gs2, const uint8_t gs3, const uint8_t gs4);
        //
        void set_motor (const uint8_t idx, const int16_t spd);
        //
        void polar_ctrl (int16_t angle, int16_t spd, int16_t rotation);
        //
        void rect_ctrl (int16_t spd_x, int16_t spd_y, int16_t rotation);
        //
        int16_t get_gryscl (const uint8_t side, const uint8_t idx);
        //
        bool gryscl_touch_white (const uint8_t side, const uint8_t idx);
        //
        bool side_touch_white (const uint8_t side);
        //
        void cal_gryscl (const uint16_t cal_time_in_millis);
        //
        int16_t max_ir_val ();
        //
        int8_t max_ir_idx ();
        //
        int16_t get_uts_dist (const uint8_t side);
        //
        int16_t get_heading ();
        //
        void reset_heading ();
        //
        void cal_compass ();
        //
        bool read_button (const uint8_t idx);
};

Kojay robot;

enum {
    front = 0,
    left = 1,
    right = 2,
    back = 3
};

#endif // #ifndef KOJAY_H