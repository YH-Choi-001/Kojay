#ifndef KOJAY_CPP
#define KOJAY_CPP

#include "Kojay.h"

Kojay::Kojay ()
{
    //
}

void Kojay::begin () {
    begin(2, 1, 0, 3, false, false, true, true, front, left, right, back); // Chloe settings on 28/04/2023
}

void Kojay::begin (const uint8_t m1, const uint8_t m2, const uint8_t m3, const uint8_t m4, const bool m1r, const bool m2r, const bool m3r, const bool m4r, const uint8_t gs1, const uint8_t gs2, const uint8_t gs3, const uint8_t gs4) {
    // motors
    mtrs[m1].begin(4, 8, 12, m1r);  // hardware M1
    mtrs[m2].begin(5, 9, 13, m2r);  // hardware M2
    mtrs[m3].begin(6, 10, 14, m3r); // hardware M3
    mtrs[m4].begin(7, 11, 15, m4r); // hardware M4
    // grayscales
    gryscls[gs1][0] = A0;
    gryscls[gs1][1] = A1;
    gryscls[gs1][2] = A2;
    gryscls[gs2][0] = A3;
    gryscls[gs2][1] = A4;
    gryscls[gs2][2] = A5;
    gryscls[gs3][0] = A6;
    gryscls[gs3][1] = A7;
    gryscls[gs3][2] = A8;
    gryscls[gs4][0] = A9;
    gryscls[gs4][1] = A10;
    gryscls[gs4][2] = A11;
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
    // tobe done: EEPROM retrieve data for compass cal and re-zero and gryscls_thresholds
    // EEPROM[0x00]: [7 : 4   unused, always write 0 (legacy motors reversed order)] [3 : 0   robot no.]
    // EEPROM[0x01 - 0x03]: reserved
    // re-zero:
    // EEPROM[0x04]: re-zero HIGH
    // EEPROM[0x05]: re-zero LOW
    // compass cal:
    // minimums:
    // EEPROM[0x06]: min_x HIGH
    // EEPROM[0x07]: min_x LOW
    // EEPROM[0x08]: min_y HIGH
    // EEPROM[0x09]: min_y LOW
    // EEPROM[0x0a]: min_z HIGH
    // EEPROM[0x0b]: min_z LOW
    // ranges:
    // EEPROM[0x0c]: range_x HIGH
    // EEPROM[0x0d]: range_x LOW
    // EEPROM[0x0e]: range_y HIGH
    // EEPROM[0x0f]: range_y LOW
    // EEPROM[0x10]: range_z HIGH
    // EEPROM[0x11]: range_z LOW
    // grayscale thresholds:
    // EEPROM[0x12]: grayscale[0][0] HIGH
    // EEPROM[0x13]: grayscale[0][0] LOW
    // EEPROM[0x14]: grayscale[0][1] HIGH
    // EEPROM[0x15]: grayscale[0][1] LOW
    // EEPROM[0x16]: grayscale[0][2] HIGH
    // EEPROM[0x17]: grayscale[0][2] LOW
    // EEPROM[0x18]: grayscale[1][0] HIGH
    // EEPROM[0x19]: grayscale[1][0] LOW
    // EEPROM[0x1a]: grayscale[1][1] HIGH
    // EEPROM[0x1b]: grayscale[1][1] LOW
    // EEPROM[0x1c]: grayscale[1][2] HIGH
    // EEPROM[0x1d]: grayscale[1][2] LOW
    // EEPROM[0x1e]: grayscale[2][0] HIGH
    // EEPROM[0x1f]: grayscale[2][0] LOW
    // EEPROM[0x20]: grayscale[2][1] HIGH
    // EEPROM[0x21]: grayscale[2][1] LOW
    // EEPROM[0x22]: grayscale[2][2] HIGH
    // EEPROM[0x23]: grayscale[2][2] LOW
    // EEPROM[0x24]: grayscale[3][0] HIGH
    // EEPROM[0x25]: grayscale[3][0] LOW
    // EEPROM[0x26]: grayscale[3][1] HIGH
    // EEPROM[0x27]: grayscale[3][1] LOW
    // EEPROM[0x28]: grayscale[3][2] HIGH
    // EEPROM[0x29]: grayscale[3][2] LOW
    uint8_t eeprom_ptr = 0x04;
    cmpas.re_zero_heading = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.min_x = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.min_y = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.min_z = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.range_x = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.range_y = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.range_z = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    for (uint8_t side = 0; side < 4; side++) {
        for (uint8_t idx = 0; idx < 3; idx++) {
            gryscls_thresholds[side][idx] = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
        }
    }
    Serial.begin(9600);
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
    int mtr1_spd = sin((angle - 45) * DEG_TO_RAD) * spd;
    mtrs[0].set_spd(mtr0_spd + rotation);
    mtrs[1].set_spd(mtr1_spd + rotation);
    mtrs[2].set_spd(-mtr0_spd + rotation);
    mtrs[3].set_spd(-mtr1_spd + rotation);
}

void Kojay::rect_ctrl (int16_t spd_x, int16_t spd_y, int16_t rotation) {
    int mtr0_spd = (spd_x + spd_y) * 0.707; // sin(45 degrees) is roughly equal to 0.707
    int mtr1_spd = (spd_x - spd_y) * 0.707;
    mtrs[0].set_spd(mtr0_spd + rotation);
    mtrs[1].set_spd(mtr1_spd + rotation);
    mtrs[2].set_spd(-mtr0_spd + rotation);
    mtrs[3].set_spd(-mtr1_spd + rotation);
}

int16_t Kojay::get_gryscl (const uint8_t side, const uint8_t idx) {
    return analogRead(gryscls[side][idx]);
}

bool Kojay::gryscl_touch_white (const uint8_t side, const uint8_t idx) {
    return get_gryscl(side, idx) > gryscls_thresholds[side][idx];
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

void Kojay::cal_gryscl (const uint16_t cal_time_in_millis) {
    // tobe done: cal-gryscl and save to EEPROM
    uint16_t maxs [4][3];
    uint16_t mins [4][3];
    for (uint8_t side = 0; side < 4; side++) {
        for (uint8_t idx = 0; idx < 3; idx++) {
            maxs[side][idx] = mins[side][idx] = get_gryscl(side, idx);
        }
    }
    unsigned long prev_millis = millis();
    while ((millis() - prev_millis) < cal_time_in_millis) {
        for (uint8_t side = 0; side < 4; side++) {
            for (uint8_t idx = 0; idx < 3; idx++) {
                const uint16_t curr_val = get_gryscl(side, idx);
                if (curr_val > maxs[side][idx]) {
                    maxs[side][idx] = curr_val;
                }
                if (curr_val < mins[side][idx]) {
                    mins[side][idx] = curr_val;
                }
            }
        }
    }
    uint8_t eeprom_ptr = 0x12;
    for (uint8_t side = 0; side < 4; side++) {
        for (uint8_t idx = 0; idx < 3; idx++) {
            const int16_t threshold = (maxs[side][idx] + mins[side][idx]) / 2;
            gryscls_thresholds[side][idx] = threshold;
            // gryscls_thresholds[side][idx] = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
            EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(threshold >> 8));
            EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(threshold & 0xff));
        }
    }
}

int16_t Kojay::max_ir_val () {
    const int16_t val0 = eyes[0].get_max_val();
    const int16_t val1 = eyes[1].get_max_val();
    if (val0 > val1) {
        return val0;
    } else {
        return val1;
    }
}

int8_t Kojay::max_ir_idx () {
    const int16_t val0 = eyes[0].get_max_val();
    const int16_t val1 = eyes[1].get_max_val();
    int8_t idx = 0;
    if (val0 > val1) {
        idx = eyes[0].get_max_idx() - 4;
    } else {
        idx = eyes[1].get_max_idx() + 2;
    }
    if (idx < 0) {
        idx += 12;
    }
    return idx;
}

int16_t Kojay::get_uts_dist (const uint8_t side) {
    return uts[side].read_dist_cm();
}

int16_t Kojay::get_heading () {
    return cmpas.get_heading();
}

void Kojay::reset_heading () {
    cmpas.reset_heading();
    EEPROM.update(0x04, static_cast<uint8_t>(cmpas.re_zero_heading >> 8));
    EEPROM.update(0x05, static_cast<uint8_t>(cmpas.re_zero_heading & 0xff));
}

void Kojay::cal_compass () {
    for (uint8_t idx = 0; idx < 4; idx++) {
        set_motor(idx, 50);
    }
    cmpas.compass_cal();
    uint8_t eeprom_ptr = 0x06;
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.min_x >> 8));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.min_x & 0xff));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.min_y >> 8));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.min_y & 0xff));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.min_z >> 8));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.min_z & 0xff));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.range_x >> 8));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.range_x & 0xff));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.range_y >> 8));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.range_y & 0xff));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.range_z >> 8));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.range_z & 0xff));
    for (uint8_t idx = 0; idx < 4; idx++) {
        set_motor(idx, 0);
    }
}

bool Kojay::read_button (const uint8_t idx) {
    return digitalRead(buttons[idx]);
}

#endif // #ifndef KOJAY_CPP