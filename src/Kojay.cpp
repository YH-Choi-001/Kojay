#ifndef KOJAY_CPP
#define KOJAY_CPP

#include "Kojay.h"

Kojay::Kojay ()
{
    //
}

void Kojay::begin () {
    // motors
    mtrs[0].begin(4, 8, 12);
    mtrs[1].begin(5, 9, 13);
    mtrs[2].begin(6, 10, 14);
    mtrs[3].begin(7, 11, 15);
    // grayscales
    gryscls[front][0] = A0;
    gryscls[front][1] = A1;
    gryscls[front][2] = A2;
    gryscls[left][0] = A3;
    gryscls[left][1] = A4;
    gryscls[left][2] = A5;
    gryscls[right][0] = A6;
    gryscls[right][1] = A7;
    gryscls[right][2] = A8;
    gryscls[back][0] = A9;
    gryscls[back][1] = A10;
    gryscls[back][2] = A11;
    for (uint8_t side = 0; side < 4; side++) {
        for (uint8_t idx = 0; idx < 3; idx++) {
            pinMode(gryscls[side][idx], INPUT);
        }
    }
    // compound eyes
    eyes[0].begin(0x01);
    eyes[1].begin(0x02);
    // ultrasounds
    uts[0].begin(31, 27);
    uts[1].begin(32, 28);
    uts[2].begin(33, 29);
    uts[3].begin(34, 30);
    // compass
    cmpas.begin();
    // buttons
    buttons[0] = 38;
    buttons[1] = 39;
    buttons[2] = 40;
    for (uint8_t idx = 0; idx < 3; idx++) {
        pinMode(buttons[idx], INPUT_PULLUP);
    }
}

void Kojay::set_motor (const uint8_t idx, const int16_t spd) {
    mtrs[idx].set_spd(spd);
}

void Kojay::polar_ctrl (int16_t angle, int16_t spd, int16_t rotation) {
    if (spd > 255) {
        spd = 255;
    } else if (spd < -255) {
        spd = -255;
    }
    int mtr0_spd = sin((angle + 45) * DEG_TO_RAD) * spd;
    int mtr1_spd = sin((angle + 135) * DEG_TO_RAD) * spd;
    mtrs[0].set_spd(mtr0_spd + rotation);
    mtrs[1].set_spd(mtr1_spd + rotation);
    mtrs[2].set_spd(-mtr0_spd + rotation);
    mtrs[3].set_spd(-mtr1_spd + rotation);
}

void Kojay::rect_ctrl (int16_t spd_x, int16_t spd_y, int16_t rotation) {
    int mtr0_spd = (spd_x + spd_y) * 0.707;
    int mtr1_spd = (spd_x - spd_y) * 0.707;
    mtrs[0].set_spd(mtr0_spd + rotation);
    mtrs[1].set_spd(mtr1_spd + rotation);
    mtrs[2].set_spd(-mtr0_spd + rotation);
    mtrs[3].set_spd(-mtr1_spd + rotation);
}

int16_t Kojay::get_gryscl (const uint8_t side, const uint8_t idx) {
    return analogRead(gryscls[side][idx]);
}

bool Kojay::side_touch_white (const uint8_t side) {
    bool touch_white = false;
    for (uint8_t i = 0; i < 3; i++) {
        if (gryscl_touch_white(side, i)) {
            touch_white = true;
            break;
        }
    }
    return touch_white;
}

#endif // #ifndef KOJAY_CPP