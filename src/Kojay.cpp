#ifndef KOJAY_CPP
#define KOJAY_CPP

#include "Kojay.h"

#define center_to_circum(x,y,r,angle,color) { \
    display.drawLine(x,y,x+r*sin(angle*DEG_TO_RAD),y-r*cos(angle*DEG_TO_RAD), color);}

#define print_degree(x,y) { \
    display.drawLine(x+1, y, x+2, y, SSD1306_WHITE); \
    display.drawLine(x, y+1, x, y+2, SSD1306_WHITE); \
    display.drawLine(x+1, y+3, x+2, y+3, SSD1306_WHITE); \
    display.drawLine(x+3, y+1, x+3, y+2, SSD1306_WHITE); \
}

Kojay robot;

Kojay::Kojay () :
    display(128, 64, &Wire, -1),
    display_debug_info(0)
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
    Wire.beginTransmission(SCREEN_ADDRESS);
    if (!Wire.endTransmission()) {
        display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
        display.display();
        display.setTextSize(1);      // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE); // Draw white text
        display.setCursor(0, 0);     // Start at top-left corner
        display.cp437(true);         // Use full 256 char 'Code Page 437' font
    }

    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    // debug info basic structure display

    // eye
    display.drawCircle(15, 15, 15, SSD1306_WHITE);
    // center_to_circum(15,15,15,eye_angle,SSD1306_WHITE);
    display.drawRect(34, 0, 8, 31, SSD1306_WHITE);
    display.setCursor(0, 34);
    display.write("CMPOeye");
    // display.setCursor(0, 43);
    // display.write(((eye_angle / 100) % 10) + '0');
    // display.write(((eye_angle / 10) % 10) + '0');
    // display.write((eye_angle % 10) + '0');
    print_degree(18, 43);
    // display.setCursor(0, 52);
    // display.write((eye_intensity / 1000) % 10 + '0');
    // display.write((eye_intensity / 100) % 10 + '0');
    // display.write((eye_intensity / 10) % 10 + '0');
    // display.write(eye_intensity % 10 + '0');

    // vertical div
    display.drawLine(45, 0, 45, 64, SSD1306_WHITE);

    // // utx
    // display.drawRect(67, 15, 5, 16, SSD1306_WHITE);
    // display.drawRect(67, 33, 5, 16, SSD1306_WHITE);
    // display.drawRect(49, 30, 16, 5, SSD1306_WHITE);
    // display.drawRect(74, 30, 16, 5, SSD1306_WHITE);
    // // display.fillRect(68, 31 - 15 * uts[0] / 1024, 3, 15 * uts[0] / 1024, SSD1306_WHITE);
    // // display.fillRect(68, 33, 3, 15 * uts[3] / 1024, SSD1306_WHITE);
    // // display.fillRect(64 - 15 * uts[1] / 1024, 31, 15 * uts[1] / 1024, 3, SSD1306_WHITE);
    // // display.fillRect(74, 31, 15 * uts[2] / 1024, 3, SSD1306_WHITE);

    // grayscale

    // box border
    // front
    display.drawRect(66, 10, 7, 7, SSD1306_WHITE);
    // left
    display.drawRect(49, 34, 7, 7, SSD1306_WHITE);
    // right
    display.drawRect(83, 23, 7, 7, SSD1306_WHITE);
    // back
    display.drawRect(66, 47, 7, 7, SSD1306_WHITE);
    // // box border
    // // front
    // display.drawRect(59, 0, 5, 5, SSD1306_WHITE);
    // display.drawRect(67, 7, 5, 5, SSD1306_WHITE);
    // display.drawRect(75, 0, 5, 5, SSD1306_WHITE);
    // // back
    // display.drawRect(59, 59, 5, 5, SSD1306_WHITE);
    // display.drawRect(67, 52, 5, 5, SSD1306_WHITE);
    // display.drawRect(75, 59, 5, 5, SSD1306_WHITE);
    // // left
    // display.drawRect(49, 17, 5, 5, SSD1306_WHITE);
    // display.drawRect(55, 24, 5, 5, SSD1306_WHITE);
    // display.drawRect(55, 36, 5, 5, SSD1306_WHITE);
    // display.drawRect(49, 43, 5, 5, SSD1306_WHITE);
    // // right
    // display.drawRect(85, 17, 5, 5, SSD1306_WHITE);
    // display.drawRect(79, 24, 5, 5, SSD1306_WHITE);
    // display.drawRect(79, 36, 5, 5, SSD1306_WHITE);
    // display.drawRect(85, 43, 5, 5, SSD1306_WHITE);
    // // fill-in
    // // front
    // // display.drawRect(60, 1, 3, 3, touch_white[0] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(68, 8, 3, 3, touch_white[1] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(76, 1, 3, 3, touch_white[2] ? SSD1306_WHITE : SSD1306_BLACK);
    // // // back
    // // display.drawRect(60, 60, 3, 3, touch_white[3] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(68, 53, 3, 3, touch_white[4] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(76, 60, 3, 3, touch_white[5] ? SSD1306_WHITE : SSD1306_BLACK);
    // // // left
    // // display.drawRect(50, 18, 3, 3, touch_white[6] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(56, 25, 3, 3, touch_white[7] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(56, 37, 3, 3, touch_white[8] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(50, 44, 3, 3, touch_white[9] ? SSD1306_WHITE : SSD1306_BLACK);
    // // // right
    // // display.drawRect(86, 18, 3, 3, touch_white[10] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(80, 25, 3, 3, touch_white[11] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(80, 37, 3, 3, touch_white[12] ? SSD1306_WHITE : SSD1306_BLACK);
    // // display.drawRect(86, 44, 3, 3, touch_white[13] ? SSD1306_WHITE : SSD1306_BLACK);

    // vertical div
    display.drawLine(93, 0, 93, 64, SSD1306_WHITE);

    // compass
    display.setCursor(98, 34);
    display.print("CMPAS");
    display.drawCircle(112,15,15, SSD1306_WHITE);
    // center_to_circum(112,15,15,(360-yaw_int),SSD1306_WHITE);
    // display.setCursor(101, 43);
    // display.print((yaw_int / 100) % 10);
    // display.print((yaw_int / 10) % 10);
    // display.print(yaw_int % 10);
    print_degree(119, 43);

    display.display();

    }
    #endif // #if DISPLAY_DEBUG_INFO

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
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    switch (side) {
        case front:
            display.fillRect(67, 11, 5, 5, touch_white ? SSD1306_WHITE : SSD1306_BLACK);
            break;
        case left:
            display.fillRect(50, 35, 5, 5, touch_white ? SSD1306_WHITE : SSD1306_BLACK);
            break;
        case right:
            display.fillRect(84, 24, 5, 5, touch_white ? SSD1306_WHITE : SSD1306_BLACK);
            break;
        case back:
            display.fillRect(67, 48, 5, 5, touch_white ? SSD1306_WHITE : SSD1306_BLACK);
            break;
    }
    }
    #endif // #if DISPLAY_DEBUG_INFO
    return touch_white;
}

void Kojay::cal_gryscl (const uint16_t cal_time_in_millis) {
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    int x_coor[4] = {67, 50, 84, 67};
    int y_coor[4] = {11, 35, 24, 48};
    for (uint8_t i = 0; i < 4; i++) {
        // draw a cross on the grayscale
        display.writePixel(x_coor[i] + 0, y_coor[i] + 0, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 1, y_coor[i] + 1, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 2, y_coor[i] + 2, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 3, y_coor[i] + 3, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 4, y_coor[i] + 4, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 0, y_coor[i] + 4, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 1, y_coor[i] + 3, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 3, y_coor[i] + 1, SSD1306_WHITE);
        display.writePixel(x_coor[i] + 4, y_coor[i] + 0, SSD1306_WHITE);
    }
    }
    #endif // #if DISPLAY_DEBUG_INFO
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
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    display.fillRect(67, 11, 5, 5, SSD1306_BLACK);
    display.fillRect(50, 35, 5, 5, SSD1306_BLACK);
    display.fillRect(84, 24, 5, 5, SSD1306_BLACK);
    display.fillRect(67, 48, 5, 5, SSD1306_BLACK);
    }
    #endif // #if DISPLAY_DEBUG_INFO
}

int16_t Kojay::max_ir_val () {
    const int16_t val0 = eyes[0].get_max_val();
    const int16_t val1 = eyes[1].get_max_val();
    const int16_t max_val = ((val0 > val1) ? val0 : val1);
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    // display.drawRect(34, 0, 8, 31, SSD1306_WHITE);
    display.fillRect(35, 1, 6, 29, SSD1306_BLACK);
    display.fillRect(35, 31 - 30UL * max_val / 160, 6, 30UL * max_val / 160, SSD1306_WHITE);
    display.fillRect(0, 52, 23, 7, SSD1306_BLACK);
    display.setCursor(0, 52);
    display.write((max_val / 1000) % 10 + '0');
    display.write((max_val / 100) % 10 + '0');
    display.write((max_val / 10) % 10 + '0');
    display.write(max_val % 10 + '0');
    }
    #endif // #if DISPLAY_DEBUG_INFO
    return max_val;
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
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    const int16_t ball_angle = idx * 30;
    display.fillCircle(15, 15, 15, SSD1306_BLACK);
    display.drawCircle(15, 15, 15, SSD1306_WHITE);
    center_to_circum(15,15,15,ball_angle,SSD1306_WHITE);
    display.fillRect(0, 43, 17, 7, SSD1306_BLACK);
    display.setCursor(0, 43);
    display.write(((ball_angle / 100) % 10) + '0');
    display.write(((ball_angle / 10) % 10) + '0');
    display.write((ball_angle % 10) + '0');
    }
    #endif // #if DISPLAY_DEBUG_INFO
    return idx;
}

int16_t Kojay::get_uts_dist (const uint8_t side) {
    const int16_t dist = uts[side].read_dist_cm();
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    switch (side) {
        case front:
            display.fillRect(58, 0, 23, 7, SSD1306_BLACK);
            display.setCursor(58, 0);
            break;
        case left:
            display.fillRect(49, 23, 23, 7, SSD1306_BLACK);
            display.setCursor(49, 23);
            break;
        case right:
            display.fillRect(67, 34, 23, 7, SSD1306_BLACK);
            display.setCursor(67, 34);
            break;
        case back:
            display.fillRect(58, 57, 23, 7, SSD1306_BLACK);
            display.setCursor(58, 57);
            break;
    }
    display.write((dist / 1000) % 10 + '0');
    display.write((dist / 100) % 10 + '0');
    display.write((dist / 10) % 10 + '0');
    display.write((dist) % 10 + '0');
    }
    #endif // #if DISPLAY_DEBUG_INFO
    return dist;
}

int16_t Kojay::get_heading () {
    int16_t heading = cmpas.get_heading();
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    display.fillCircle(112,15,15, SSD1306_BLACK);
    display.drawCircle(112,15,15, SSD1306_WHITE);
    center_to_circum(112,15,15,(360-heading),SSD1306_WHITE);
    display.fillRect(101, 43, 23, 7, SSD1306_BLACK);
    display.setCursor(101, 43);
    display.print((heading / 100) % 10);
    display.print((heading / 10) % 10);
    display.print(heading % 10);
    print_degree(119, 43);
    }
    #endif // #if DISPLAY_DEBUG_INFO
    return heading;
}

void Kojay::reset_heading () {
    cmpas.reset_heading();
    EEPROM.update(0x04, static_cast<uint8_t>(cmpas.re_zero_heading >> 8));
    EEPROM.update(0x05, static_cast<uint8_t>(cmpas.re_zero_heading & 0xff));
}

void Kojay::cal_compass () {
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    display.fillCircle(112,15,15, SSD1306_WHITE);
    display.fillRect(101, 43, 23, 7, SSD1306_BLACK);
    display.setCursor(101, 43);
    display.print("cali");
    display.display();
    }
    #endif // #if DISPLAY_DEBUG_INFO

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

    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    display.fillCircle(112,15,15, SSD1306_BLACK);
    display.drawCircle(112,15,15, SSD1306_WHITE);
    center_to_circum(112,15,15,0,SSD1306_WHITE);
    display.fillRect(101, 43, 23, 7, SSD1306_BLACK);
    display.setCursor(101, 43);
    display.print("000");
    print_degree(119, 43);
    display.display();
    }
    #endif // #if DISPLAY_DEBUG_INFO
}

bool Kojay::button_pressed (const uint8_t idx) {
    return !digitalRead(buttons[idx]);
}

void Kojay::clear_mon () {
    robot.display.clearDisplay();
}

void Kojay::set_cursor (int16_t x, int16_t y) {
    robot.display.setCursor(x, y);
}



size_t Kojay::print(const __FlashStringHelper *x) {
    return robot.display.print(x);
}

size_t Kojay::print(const String &x) {
    return robot.display.print(x);
}

size_t Kojay::print(const char x[]) {
    return robot.display.print(x);
}

size_t Kojay::print(char x) {
    return robot.display.print(x);
}

size_t Kojay::print(unsigned char x, int y) {
    return robot.display.print(x, y);
}

size_t Kojay::print(int x, int y) {
    return robot.display.print(x, y);
}

size_t Kojay::print(unsigned int x, int y) {
    return robot.display.print(x, y);
}

size_t Kojay::print(long x, int y) {
    return robot.display.print(x, y);
}

size_t Kojay::print(unsigned long x, int y) {
    return robot.display.print(x, y);
}

size_t Kojay::print(double x, int y) {
    return robot.display.print(x, y);
}

size_t Kojay::print(const Printable& x) {
    return robot.display.print(x);
}



void Kojay::update_all_data () {
    const bool old_display_debug_info = display_debug_info;
    display_debug_info = true;
    for (uint8_t i = 0; i < 4; i++) {
        side_touch_white(i);
        get_uts_dist(i);
    }
    max_ir_val();
    max_ir_idx();
    get_heading();
    if (button_pressed(0)) {
        cal_gryscl(20000);
        while (button_pressed(0)) {}
    } else if (button_pressed(1)) {
        cal_compass();
        while (button_pressed(1)) {}
    } else if (button_pressed(2)) {
        reset_heading();
        while (button_pressed(2)) {}
    }
    display_debug_info = old_display_debug_info;
}

#endif // #ifndef KOJAY_CPP