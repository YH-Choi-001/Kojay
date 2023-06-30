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

#define STRINGS(...) F(__VA_ARGS__)

Kojay robot;

Kojay::Kojay () :
    uts_dists{888, 888, 888, 888},
    display(128, 64, &Wire, -1),
    display_debug_info(0),
    prev_uts_time(0)
{
    //
}

void Kojay::begin () {
    const uint8_t eeprom_adr0 = EEPROM.read(0x00);
    mtrs_idxs = EEPROM.read(0x01);
    begin((mtrs_idxs >> 0) & 0b11, (mtrs_idxs >> 2) & 0b11, (mtrs_idxs >> 4) & 0b11, (mtrs_idxs >> 6) & 0b11, (eeprom_adr0 & (1 << 4)) ? true : false, (eeprom_adr0 & (1 << 5)) ? true : false, (eeprom_adr0 & (1 << 6)) ? true : false, (eeprom_adr0 & (1 << 7)) ? true : false);
    // begin(2, 1, 0, 3, false, false, true, true, front, left, right, back); // Chloe settings on 28/04/2023
}

void Kojay::begin (uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, const bool m1r, const bool m2r, const bool m3r, const bool m4r) {
    // Serial monitor debug
    Serial.begin(9600);
    // I2C bus
    Wire.begin();
    Wire.setClock(400000);
    // I2C oled monitor
    Wire.beginTransmission(SCREEN_ADDRESS);
    if (!Wire.endTransmission()) {
        display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
        clear_mon();
        display.setTextSize(1);      // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE); // Draw white text
        display.setCursor(0, 0);     // Start at top-left corner
        display.cp437(true);         // Use full 256 char 'Code Page 437' font
    }
    {
        uint8_t slaves_found = 0x00;
        for (uint8_t i = 1; i <= 127; i++) {
            Wire.beginTransmission(i);
            if (!Wire.endTransmission()) {
                Serial.print(STRINGS("I2C slave found at 0x"));
                Serial.print(i, HEX);
                switch (i) {
                    case 0x01:
                        Serial.print(STRINGS(",\twhich is IR-ring 1"));
                        slaves_found |= (1 << 0);
                        break;
                    case 0x02:
                        Serial.print(STRINGS(",\twhich is IR-ring 2"));
                        slaves_found |= (1 << 1);
                        break;
                    case 0x0D:
                        Serial.print(STRINGS(",\twhich is QMC5883L"));
                        slaves_found |= (1 << 2);
                        break;
                    case 0x3C:
                        Serial.print(STRINGS(",\twhich is OLED monitor"));
                        slaves_found |= (1 << 3);
                        break;
                    case 0x68:
                        Serial.print(STRINGS(",\twhich is MPU6050"));
                        slaves_found |= (1 << 4);
                        break;
                    default:
                        Serial.print(STRINGS(",\twhich is unidentified"));
                        break;
                }
                Serial.print('\n');
            }
        }
        {
            int y = 0;
            if (!(slaves_found & (1 << 0))) {
                set_cursor(0, y++);
                print("IR-ring 1 not found");
                delay(2000);
            }
            if (!(slaves_found & (1 << 1))) {
                set_cursor(0, y++);
                print("IR-ring 2 not found");
                delay(2000);
            }
            if (!(slaves_found & (1 << 2)) && !(slaves_found & (1 << 4))) {
                set_cursor(0, y++);
                print("no headings found");
                delay(2000);
            }
            clear_mon();
        }
    }
    // motors
    m1 &= 0b11;
    m2 &= 0b11;
    m3 &= 0b11;
    m4 &= 0b11;
    mtrs_idxs = (m4 << 6) | (m3 << 4) | (m2 << 2) | (m1 << 0);
    mtrs[m1].begin(4, 8, 12, m1r);  // hardware M1
    mtrs[m2].begin(5, 9, 13, m2r);  // hardware M2
    mtrs[m3].begin(6, 10, 14, m3r); // hardware M3
    mtrs[m4].begin(7, 11, 15, m4r); // hardware M4
    // grayscales
    gryscls[0] = A0;
    gryscls[1] = A1;
    gryscls[2] = A2;
    gryscls[3] = A3;
    gryscls[4] = A4;
    gryscls[5] = A5;
    gryscls[6] = A6;
    gryscls[7] = A7;
    gryscls[8] = A8;
    gryscls[9] = A9;
    gryscls[10] = A10;
    gryscls[11] = A11;
    for (uint8_t idx = 0; idx < 12; idx++) {
        pinMode(gryscls[idx], INPUT);
    }
    // compound eyes
    if (EEPROM.read(0x00) & (1 << 3)) {
        eyes[0].begin(0x02);
        eyes[1].begin(0x01);
    } else {
        eyes[0].begin(0x01);
        eyes[1].begin(0x02);
    }
    // ultrasounds
    uts[0].begin(31, 27);
    uts[1].begin(32, 28);
    uts[2].begin(33, 29);
    uts[3].begin(34, 30);
    // compass
    use_gyro_instead_cmpas = false;
    if (cmpas.is_cmpas_present()) {
        cmpas.begin();
    } else if (gyro.is_gyro_present()) {
        use_gyro_instead_cmpas = true;
        gyro.begin();
    }
    // buttons
    buttons[0] = 38;
    buttons[1] = 39;
    buttons[2] = 40;
    for (uint8_t idx = 0; idx < 3; idx++) {
        pinMode(buttons[idx], INPUT_PULLUP);
    }
    // tobe done: EEPROM retrieve data for compass cal and re-zero and gryscls_thresholds
    // EEPROM[0x00]: [7 : 4   motors reversed direction] [3   compoI reversed direction] [2 : 0   robot no.]
    // EEPROM[0x01]: [7 : 6 M4_idx] [5 : 4 M3_idx] [3 : 2 M2_idx] [1 : 0 M1_idx]
    // EEPROM[0x02 - 0x03]: reserved
    // compass:
    // re-zero:
    // EEPROM[0x04]: re-zero HIGH
    // EEPROM[0x05]: re-zero LOW
    // compass cal:
    // minimums:
    // EEPROM[0x06]: base_x HIGH
    // EEPROM[0x07]: base_x LOW
    // EEPROM[0x08]: base_y HIGH
    // EEPROM[0x09]: base_y LOW
    // EEPROM[0x0a]: base_z HIGH
    // EEPROM[0x0b]: base_z LOW
    // ranges:
    // EEPROM[0x0c]: range_x HIGH
    // EEPROM[0x0d]: range_x LOW
    // EEPROM[0x0e]: range_y HIGH
    // EEPROM[0x0f]: range_y LOW
    // EEPROM[0x10]: range_z HIGH
    // EEPROM[0x11]: range_z LOW
    // EEPROM[0x12 - 0x2f]: reserved
    // gyro:
    // EEPROM[0x30]: gyro cal_x HIGH
    // EEPROM[0x31]: gyro cal_x LOW
    // EEPROM[0x32]: gyro cal_y HIGH
    // EEPROM[0x33]: gyro cal_y LOW
    // EEPROM[0x34]: gyro cal_z HIGH
    // EEPROM[0x35]: gyro cal_z LOW
    // grayscale thresholds:
    // EEPROM[0x40]: grayscale [0] HIGH
    // EEPROM[0x41]: grayscale [0] LOW
    // EEPROM[0x42]: grayscale [1] HIGH
    // EEPROM[0x43]: grayscale [1] LOW
    // EEPROM[0x44]: grayscale [2] HIGH
    // EEPROM[0x45]: grayscale [2] LOW
    // EEPROM[0x46]: grayscale [3] HIGH
    // EEPROM[0x47]: grayscale [3] LOW
    // EEPROM[0x48]: grayscale [4] HIGH
    // EEPROM[0x49]: grayscale [4] LOW
    // EEPROM[0x4a]: grayscale [5] HIGH
    // EEPROM[0x4b]: grayscale [5] LOW
    // EEPROM[0x4c]: grayscale [6] HIGH
    // EEPROM[0x4d]: grayscale [6] LOW
    // EEPROM[0x4e]: grayscale [7] HIGH
    // EEPROM[0x4f]: grayscale [7] LOW
    // EEPROM[0x50]: grayscale [8] HIGH
    // EEPROM[0x51]: grayscale [8] LOW
    // EEPROM[0x52]: grayscale [9] HIGH
    // EEPROM[0x53]: grayscale [9] LOW
    // EEPROM[0x54]: grayscale [10] HIGH
    // EEPROM[0x55]: grayscale [10] LOW
    // EEPROM[0x56]: grayscale [11] HIGH
    // EEPROM[0x57]: grayscale [11] LOW
    // EEPROM[0x58]: grayscale inverse logic [11 : 8]
    // EEPROM[0x59]: grayscale inverse logic [7 : 0]
    // EEPROM[0x5a - 0x5f]: reserved
    //grayscale mapping:
    // EEPROM[0x60]: grayscale [0][0]
    // EEPROM[0x61]: grayscale [0][1]
    // EEPROM[0x62]: grayscale [0][2]
    // EEPROM[0x63]: grayscale [1][0]
    // EEPROM[0x64]: grayscale [1][1]
    // EEPROM[0x65]: grayscale [1][2]
    // EEPROM[0x66]: grayscale [2][0]
    // EEPROM[0x67]: grayscale [2][1]
    // EEPROM[0x68]: grayscale [2][2]
    // EEPROM[0x69]: grayscale [3][0]
    // EEPROM[0x6a]: grayscale [3][1]
    // EEPROM[0x6b]: grayscale [3][2]
    uint8_t eeprom_ptr = 0x04;
    cmpas.re_zero_heading = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.base_x = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.base_y = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.base_z = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.range_x = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.range_y = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    cmpas.range_z = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    eeprom_ptr = 0x40;
    for (uint8_t idx = 0; idx < 12; idx++) {
        gryscls_thresholds[idx] = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    }
    const uint16_t gs_inverse = (EEPROM.read(eeprom_ptr++) << 8) | EEPROM.read(eeprom_ptr++);
    for (uint8_t idx = 0; idx < 12; idx++) {
        gryscls_inverse_logic[idx] = ((gs_inverse & (1 << idx)) ? true : false);
    }
    eeprom_ptr = 0x60;
    for (uint8_t side = 0; side < 4; side++) {
        for (uint8_t idx = 0; idx < 3; idx++) {
            gryscl_map[side][idx] = EEPROM.read(eeprom_ptr++);
        }
    }

    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    // debug info basic structure display

    // eye
    display.drawCircle(15, 15, 15, SSD1306_WHITE);
    // center_to_circum(15,15,15,eye_angle,SSD1306_WHITE);
    display.drawRect(34, 0, 8, 31, SSD1306_WHITE);
    display.setCursor(0, 34);
    display.print(STRINGS("CMPOeye"));
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
    display.print(STRINGS("CMPAS"));
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

bool Kojay::move_to (const uint8_t side1, const int16_t dist1, const uint8_t side2, const int16_t dist2, const int16_t threshold, const int16_t spd, const double rotation_kp, const int16_t target_heading) {
    if ((side1 == side2) || (side1 == front && side2 == back) || (side1 == back && side2 == front) || (side1 == left && side2 == right) || (side1 == right && side2 == left)) {
        robot.polar_ctrl(0, 0, 0);
        return false;
    }
    int dist_to_x, dist_to_y;
    switch (side1) {
        case front:
            dist_to_y = robot.get_uts_dist(front) - dist1;
            break;
        case left:
            dist_to_x = dist1 - robot.get_uts_dist(left);
            break;
        case right:
            dist_to_x = robot.get_uts_dist(right) - dist1;
            break;
        case back:
            dist_to_y = dist1 - robot.get_uts_dist(back);
            break;
    }
    switch (side2) {
        case front:
            dist_to_y = robot.get_uts_dist(front) - dist2;
            break;
        case left:
            dist_to_x = dist2 - robot.get_uts_dist(left);
            break;
        case right:
            dist_to_x = robot.get_uts_dist(right) - dist2;
            break;
        case back:
            dist_to_y = dist2 - robot.get_uts_dist(back);
            break;
    }
    int relative_heading = robot.get_heading() - target_heading;
    while (relative_heading < -180) {
        relative_heading += 360;
    }
    while (relative_heading > 180) {
        relative_heading -= 360;
    }
    if ((dist_to_x <= threshold) && (dist_to_y <= threshold) && (relative_heading > -7) && (relative_heading < 7)) {
        robot.polar_ctrl(0, 0, 0);
        return true;
    }
    robot.polar_ctrl(atan2(dist_to_x, dist_to_y) * RAD_TO_DEG, spd, -relative_heading * rotation_kp);
    return false;
}

int16_t Kojay::get_raw_gryscl (const uint8_t idx) {
    if (idx >= 12) {
        return 8193;
    }
    return analogRead(gryscls[idx]);
}

bool Kojay::raw_gryscl_touch_white (const uint8_t idx) {
    if (idx >= 12) {
        return false;
    }
    const uint16_t val = get_raw_gryscl(idx);
    if (gryscls_inverse_logic[idx]) {
        return val < gryscls_thresholds[idx];
    } else {
        return val > gryscls_thresholds[idx];
    }
}

bool Kojay::raw_side_touch_white (const uint8_t side) {
    bool touch_white = false;
    uint8_t basic_val = side * 3;
    for (uint8_t i = 0; i < 3; i++, basic_val++) {
        if (raw_gryscl_touch_white(basic_val)) {
            touch_white = true;
            break;
        }
    }
    return touch_white;
}

int16_t Kojay::get_gryscl (const uint8_t side, const uint8_t idx) {
    return get_raw_gryscl(gryscl_map[side][idx]);
}

bool Kojay::gryscl_touch_white (const uint8_t side, const uint8_t idx) {
    return raw_gryscl_touch_white(gryscl_map[side][idx]);
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
    // box border
    // front
    display.drawRect(66, 10, 7, 7, SSD1306_WHITE);
    // left
    display.drawRect(49, 34, 7, 7, SSD1306_WHITE);
    // right
    display.drawRect(83, 23, 7, 7, SSD1306_WHITE);
    // back
    display.drawRect(66, 47, 7, 7, SSD1306_WHITE);
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

void Kojay::cal_gryscl () {
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
    display.display();
    }
    #endif // #if DISPLAY_DEBUG_INFO
    // tobe done: cal-gryscl and save to EEPROM
    uint16_t maxs [12];
    uint16_t mins [12];
    uint16_t initials [12];
    for (uint8_t idx = 0; idx < 12; idx++) {
        maxs[idx] = mins[idx] = initials[idx] = analogRead(gryscls[idx]);
    }
    unsigned long prev_millis = millis();
    while ((millis() - prev_millis) < 5000) {
        for (uint8_t idx = 0; idx < 12; idx++) {
            bool updated = false;
            const uint16_t curr_val = analogRead(gryscls[idx]);
            if (curr_val > maxs[idx]) {
                maxs[idx] = curr_val;
                updated = true;
            }
            if (curr_val < mins[idx]) {
                mins[idx] = curr_val;
                updated = true;
            }
            if (updated) {
                prev_millis = millis();
            }
        }
    }
    uint16_t gryscls_inverse_word = 0x00;
    uint8_t eeprom_ptr = 0x40;
    for (uint8_t idx = 0; idx < 12; idx++) {
        // threshold
        const int16_t threshold = (maxs[idx] + mins[idx]) / 2;
        gryscls_thresholds[idx] = threshold;
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(threshold >> 8));
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(threshold & 0xff));
        // inverse logic
        gryscls_inverse_logic[idx] = (initials[idx] > threshold); // if the reading in green is greater than the reading in white, the logic is inversed
        if (gryscls_inverse_logic[idx]) {
            gryscls_inverse_word |= (1 << idx);
        }
    }
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(gryscls_inverse_word >> 8));
    EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(gryscls_inverse_word & 0xff));
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

void Kojay::set_ir_addr_0x01 () {
    eyes[0].set_addr_0x01();
    eyes[1].set_addr_0x01();
}

void Kojay::set_ir_addr_0x02 () {
    eyes[0].set_addr_0x02();
    eyes[1].set_addr_0x02();
}

int16_t Kojay::get_uts_dist (const uint8_t side) {
    // int16_t dist = uts[side].read_dist_cm();
    int16_t dist;
    // if (micros() - prev_uts_time < 33333) {
    //     dist = uts_dists[side];
    // } else {
    //     uts_dists[side] = dist = uts[side].read_dist_cm();
    //     prev_uts_time = micros();
    // }
    while (micros() - prev_uts_time < 10000) {}
    uts_dists[side] = dist = uts[side].read_dist_cm();
    prev_uts_time = micros();
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

double Kojay::get_heading () {
    int16_t heading = 0;
    if (use_gyro_instead_cmpas) {
        heading = gyro.get_yaw();
    } else {
        heading = cmpas.get_heading();
    }
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
    if (use_gyro_instead_cmpas) {
        gyro.reset_yaw();
    } else {
        cmpas.reset_heading();
        EEPROM.update(0x04, static_cast<uint8_t>(cmpas.re_zero_heading >> 8));
        EEPROM.update(0x05, static_cast<uint8_t>(cmpas.re_zero_heading & 0xff));
    }
}

void Kojay::cal_compass () {
    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    display.fillCircle(112,15,15, SSD1306_WHITE);
    display.fillRect(101, 43, 23, 7, SSD1306_BLACK);
    display.setCursor(101, 43);
    display.print(STRINGS("cali"));
    }
    #endif // #if DISPLAY_DEBUG_INFO

    if (use_gyro_instead_cmpas) {
        #if DISPLAY_DEBUG_INFO
        if (display_debug_info) {
        display.setCursor(101, 52);
        display.print(STRINGS("gyro"));
        display.display();
        }
        #endif // #if DISPLAY_DEBUG_INFO
        for (uint8_t idx = 0; idx < 4; idx++) {
            set_motor(idx, 0);
        }
        gyro.cal();
    } else {
        #if DISPLAY_DEBUG_INFO
        if (display_debug_info) {
        display.setCursor(98, 52);
        display.print(STRINGS("magnt"));
        display.display();
        }
        #endif // #if DISPLAY_DEBUG_INFO
        for (uint8_t idx = 0; idx < 4; idx++) {
            set_motor(idx, 70);
        }

        cmpas.compass_cal();

        uint8_t eeprom_ptr = 0x06;
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.base_x >> 8));
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.base_x & 0xff));
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.base_y >> 8));
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.base_y & 0xff));
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.base_z >> 8));
        EEPROM.update(eeprom_ptr++, static_cast<uint8_t>(cmpas.base_z & 0xff));
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

    #if DISPLAY_DEBUG_INFO
    if (display_debug_info) {
    display.fillCircle(112,15,15, SSD1306_BLACK);
    display.drawCircle(112,15,15, SSD1306_WHITE);
    center_to_circum(112,15,15,0,SSD1306_WHITE);
    display.fillRect(101, 43, 23, 7, SSD1306_BLACK);
    display.fillRect(98, 52, 29, 7, SSD1306_BLACK);
    display.setCursor(101, 43);
    display.print(STRINGS("000"));
    print_degree(119, 43);
    display.display();
    }
    #endif // #if DISPLAY_DEBUG_INFO
}

int16_t Kojay::get_rotation_spd (int target_heading) {
    int16_t heading = get_heading() - target_heading;
    while (heading > 180) {
        heading -= 360;
    }
    while (heading < -180) {
        heading += 360;
    }
    return -heading;
}

bool Kojay::button_pressed (const uint8_t idx) {
    return !digitalRead(buttons[idx]);
}

void Kojay::clear_mon () {
    display.clearDisplay();
    display.display();
}

void Kojay::set_cursor (int16_t x, int16_t y) {
    display.setCursor(x * 6, y * 9);
}



size_t Kojay::print(const __FlashStringHelper *x) {
    const size_t r = display.print(x);
    display.display();
    return r;
}

size_t Kojay::print(const String &x) {
    const size_t r = display.print(x);
    display.display();
    return r;
}

size_t Kojay::print(const char x[]) {
    const size_t r = display.print(x);
    display.display();
    return r;
}

size_t Kojay::print(char x) {
    const size_t r = display.print(x);
    display.display();
    return r;
}

size_t Kojay::print(unsigned char x, int y) {
    const size_t r = display.print(x, y);
    display.display();
    return r;
}

size_t Kojay::print(int x, int y) {
    const size_t r = display.print(x, y);
    display.display();
    return r;
}

size_t Kojay::print(unsigned int x, int y) {
    const size_t r = display.print(x, y);
    display.display();
    return r;
}

size_t Kojay::print(long x, int y) {
    const size_t r = display.print(x, y);
    display.display();
    return r;
}

size_t Kojay::print(unsigned long x, int y) {
    const size_t r = display.print(x, y);
    display.display();
    return r;
}

size_t Kojay::print(double x, int y) {
    const size_t r = display.print(x, y);
    display.display();
    return r;
}

size_t Kojay::print(const Printable& x) {
    const size_t r = display.print(x);
    display.display();
    return r;
}



void Kojay::update_all_data () {
    const bool old_display_debug_info = display_debug_info;
    display_debug_info = true;
    // eye
    display.drawCircle(15, 15, 15, SSD1306_WHITE);
    // center_to_circum(15,15,15,eye_angle,SSD1306_WHITE);
    display.drawRect(34, 0, 8, 31, SSD1306_WHITE);
    display.setCursor(0, 34);
    display.print(STRINGS("CMPOeye"));
    print_degree(18, 43);

    // vertical div
    display.drawLine(45, 0, 45, 64, SSD1306_WHITE);

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

    // vertical div
    display.drawLine(93, 0, 93, 64, SSD1306_WHITE);

    // compass
    display.setCursor(98, 34);
    display.print(STRINGS("CMPAS"));
    display.drawCircle(112,15,15, SSD1306_WHITE);
    print_degree(119, 43);

    for (uint8_t i = 0; i < 4; i++) {
        side_touch_white(i);
        get_uts_dist(i);
    }
    max_ir_val();
    max_ir_idx();
    get_heading();
    robot.display.display();
    // mtrs[m1].begin(4, 8, 12, m1r);  // hardware M1
    // mtrs[m2].begin(5, 9, 13, m2r);  // hardware M2
    // mtrs[m3].begin(6, 10, 14, m3r); // hardware M3
    // mtrs[m4].begin(7, 11, 15, m4r); // hardware M4
    display_debug_info = old_display_debug_info;
}

bool Kojay::menu () {
    const bool old_display_debug_info = display_debug_info;
    display_debug_info = true;
    static uint8_t mode = 0;
    // 9, 4 page of raw-motors, 4 page of realloc-motors, 4 page of reassigned-motors, 5 page of raw-gryscl, 6 page of realloc-gryscl, 4 page of reassigned-gryscl, 1 page of all data, 3 page of cal gryscl, 3 page of cal compass
    static const uint8_t max_page_of_mode [] = {10, 4, 4, 4, 5, 6, 4, 1, 3, 3, 2};
    static uint8_t page = 0;

    static Motor raw_mtrs [4];
    raw_mtrs[0].begin(4, 8, 12, false);  // hardware M1
    raw_mtrs[1].begin(5, 9, 13, false);  // hardware M2
    raw_mtrs[2].begin(6, 10, 14, false); // hardware M3
    raw_mtrs[3].begin(7, 11, 15, false); // hardware M4
    switch (mode) {
        case 0:
            display.clearDisplay();
            display.setCursor(0, 0);
            switch (page) {
                case 0:
                    display.print(STRINGS("1. raw-motors"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("M1, M2, M3, M4"));
                    break;
                case 1:
                    display.print(STRINGS("2. realloc-motors"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("M1, M2, M3, M4"));
                    display.setCursor(0, 18);
                    display.print(STRINGS("[0], [1], [2], [3]"));
                    break;
                case 2:
                    display.print(STRINGS("3. reasgn-motors"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("[0], [1], [2], [3]"));
                    break;
                case 3:
                    display.print(STRINGS("4. raw-gryscls"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("0123 x [0],[1],[2]"));
                    break;
                case 4:
                    display.print(STRINGS("5. realloc-gryscls"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("F, L, R, B"));
                    display.setCursor(0, 18);
                    display.print(STRINGS("[0], [1], [2]"));
                    break;
                case 5:
                    display.print(STRINGS("6. reasgn-gryscls"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("FLRB x [0],[1],[2]"));
                    break;
                case 6:
                    display.print(STRINGS("7. disp all data"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("IR, UTS, CMPAS"));
                    display.setCursor(0, 18);
                    display.print(STRINGS("GRYSCLS"));
                    break;
                case 7:
                    display.print(STRINGS("8. cal gryscl"));
                    break;
                case 8:
                    display.print(STRINGS("9. cal compass"));
                    break;
                case 9:
                    display.print(STRINGS("10. start program"));
                    break;
                default:
                    page = 0;
                    break;
            }
            display.display();
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } else {
                            page = max_page_of_mode[mode] - 1;
                        }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    mode = page + 1;
                    page = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            break;
        case 1:
        {
            static bool inc = true;
            if (inc) {
                raw_mtrs[page]+=5;
            } else {
                raw_mtrs[page]-=5;
            }
            if (raw_mtrs[page] == 255) {
                inc = false;
            }
            if (raw_mtrs[page] == -255) {
                inc = true;
            }
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(STRINGS("raw: M"));
            display.print(page+1);
            display.print(':');
            if (raw_mtrs[page] > 0) {
                display.print('+');
            } else if (raw_mtrs[page] == 0) {
                display.print(' ');
            } else {
                display.print('-');
            }
            const uint16_t spd = abs(raw_mtrs[page]);
            if (spd < 100) {
                display.print('0');
            }
            if (spd < 10) {
                display.print('0');
            }
            display.print(spd);
            display.display();
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    raw_mtrs[page] = 0;
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } else {
                            page = max_page_of_mode[mode] - 1;
                        }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    raw_mtrs[page] = 0;
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
        }
            break;
        case 2:
        {
            // page indicates M1, M2, M3, M4
            // mtr_idx indicates the selected motor being [0], [1], [2], [3]
            static uint8_t mtr_idx = 0;
            static uint8_t mtr_reversed = 0;
            display.clearDisplay();
            switch (mtr_idx) {
                case 0:
                    center_to_circum(128/2, 64/2, 30, 225, SSD1306_BLACK);
                    center_to_circum(128/2, 64/2, 30, 315, SSD1306_WHITE);
                    break;
                case 1:
                    center_to_circum(128/2, 64/2, 30, 315, SSD1306_BLACK);
                    center_to_circum(128/2, 64/2, 30, 45, SSD1306_WHITE);
                    break;
                case 2:
                    center_to_circum(128/2, 64/2, 30, 45, SSD1306_BLACK);
                    center_to_circum(128/2, 64/2, 30, 135, SSD1306_WHITE);
                    break;
                case 3:
                    center_to_circum(128/2, 64/2, 30, 135, SSD1306_BLACK);
                    center_to_circum(128/2, 64/2, 30, 225, SSD1306_WHITE);
                    break;
            }
            display.setCursor(0, 0);
            display.print(STRINGS("motor ["));
            display.print(mtr_idx);
            display.print(STRINGS("] is at:"));
            raw_mtrs[page].set_spd((mtr_reversed & (1 << page)) ? -80 : 80);
            display.display();
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    raw_mtrs[page] = 0;
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        mtr_reversed ^= (1 << page);
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    raw_mtrs[page] = 0;
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= 4) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    if (mtr_idx < 4) {
                        // considering which motor M (M == page) carries the index mtr_idx
                        // if mtr_idx == 0, page == 2, M3 carries the index 0
                        mtrs_idxs &= ~((0b11) << ((page & 0b11) * 2));
                        mtrs_idxs |= (mtr_idx & 0b11) << ((page & 0b11) * 2);
                        mtr_idx++;
                    }
                    if (mtr_idx >= 4) {
                        display.clearDisplay();
                        display.setCursor(0, 0);
                        display.print(STRINGS("all motors reasgned"));
                        display.display();
                        // EEPROM operations
                        uint8_t eeprom_adr0 = EEPROM.read(0x00);
                        // mtrs_idxs = EEPROM.read(0x01);
                        eeprom_adr0 &= 0x0f;
                        eeprom_adr0 |= (mtr_reversed << 4);
                        EEPROM.update(0x00, eeprom_adr0);
                        EEPROM.update(0x01, mtrs_idxs);
                        mtrs[(mtrs_idxs >> 0) & 0b11].begin(4, 8, 12, (eeprom_adr0 & (1 << 4)) ? true : false);  // hardware M1
                        mtrs[(mtrs_idxs >> 2) & 0b11].begin(5, 9, 13, (eeprom_adr0 & (1 << 5)) ? true : false);  // hardware M2
                        mtrs[(mtrs_idxs >> 4) & 0b11].begin(6, 10, 14, (eeprom_adr0 & (1 << 6)) ? true : false); // hardware M3
                        mtrs[(mtrs_idxs >> 6) & 0b11].begin(7, 11, 15, (eeprom_adr0 & (1 << 7)) ? true : false); // hardware M4
                        display.setCursor(0, 9);
                        display.print(STRINGS("mtrs config saved"));
                        display.display();
                        delay(1500);
                        mtr_idx = 0;
                        mtr_reversed = 0;
                        page = mode - 1;
                        mode = 0;
                    }
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
        }
            break;
        case 3:
        {
            static bool inc = true;
            if (inc) {
                mtrs[page]+=5;
            } else {
                mtrs[page]-=5;
            }
            if (mtrs[page] == 255) {
                inc = false;
            }
            if (mtrs[page] == -255) {
                inc = true;
            }
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(STRINGS("reasgn: ["));
            display.print(page);
            display.print(STRINGS("]:"));
            if (mtrs[page] > 0) {
                display.print('+');
            } else if (mtrs[page] == 0) {
                display.print(' ');
            } else {
                display.print('-');
            }
            const uint16_t spd = abs(mtrs[page]);
            if (spd < 100) {
                display.print('0');
            }
            if (spd < 10) {
                display.print('0');
            }
            display.print(spd);
            display.display();
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    mtrs[page] = 0;
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } else {
                            page = max_page_of_mode[mode] - 1;
                        }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    mtrs[page] = 0;
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
        }
            break;
        case 4:
        {
            if (page == 0) {
                display.clearDisplay();
                for (uint8_t i = 0; i < 12; i++) {
                    display.drawRect(i * 10, 0, 8, 64, SSD1306_WHITE);
                    const uint8_t height = (analogRead(gryscls[i]) * 64) / 1024;
                    display.fillRect(i * 10 + 1, 64 - height, 6, height, SSD1306_WHITE);
                    const uint16_t threshold_height = 64 - (gryscls_thresholds[gryscls[i]] * 64) / 1024;
                    display.drawLine(i * 10 + 1, threshold_height, i * 10 + 6, threshold_height, SSD1306_WHITE);
                }
                display.display();
            } else {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.print(STRINGS("gryscl no.: "));
                const char arr [] = {'0', '1', '2', '3'};
                display.print(arr[page - 1]);
                display.print(':');
                display.setCursor(0, 9);
                display.print(STRINGS("idx   raw  thrs"));
                //             [i]  anlR  thrs
                uint8_t y_coor = 18;
                for (uint8_t idx = 0; idx < 3; idx++) {
                    display.setCursor(0, y_coor);
                    display.print('[');
                    display.print(idx);
                    display.print(STRINGS("]  "));
                    const int16_t val = get_raw_gryscl((page-1) * 3 + idx);
                    if (val < 1000) {
                        display.print('0');
                    }
                    if (val < 100) {
                        display.print('0');
                    }
                    if (val < 10) {
                        display.print('0');
                    }
                    display.print(val < 9999 ? val : 9999);
                    display.print("  ");
                    const uint16_t thrs = gryscls_thresholds[(page-1) * 3 + idx];
                    if (thrs < 1000) {
                        display.print('0');
                    }
                    if (thrs < 100) {
                        display.print('0');
                    }
                    if (thrs < 10) {
                        display.print('0');
                    }
                    display.print(thrs < 9999 ? thrs : 9999);
                    display.drawRect(121, y_coor, 7, 7, SSD1306_WHITE);
                    if (raw_gryscl_touch_white((page-1) * 3 + idx)) {
                        display.fillRect(121, y_coor, 7, 7, SSD1306_WHITE);
                    }
                    y_coor += 9;
                }
            }
            display.display();
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } else {
                            page = max_page_of_mode[mode] - 1;
                        }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            break;
        }
        case 5:
        {
            if (page == 0) {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.print(STRINGS("gryscl"));
                display.setCursor(0, 0);
                display.print(STRINGS("press to realloc"));
                display.display();
            } else if (page == 1) {
                for (; page <= 4; page++) {
                    // #define TOUCHING_WHITE_LINE(idx) (gryscls_inverse_logic[idx] ? (analogRead(gryscls[idx]) < gryscls_thresholds[idx]) : (analogRead(gryscls[idx]) > gryscls_thresholds[idx]))
                    display.clearDisplay();
                    display.setCursor(0, 0);
                    display.print(STRINGS("gryscl side "));
                    const char arr [] = {'F', 'L', 'R', 'B'};
                    display.print(arr[page - 1]);
                    display.print(':');
                    display.setCursor(0, 9);
                    display.print(STRINGS("[xx] [xx] [xx]")); // 1, 6, 11
                    display.display();
                    uint8_t indice [3] = {101, 101, 101};
                    {
                        uint8_t idx = 0;
                        unsigned long prev_millis = millis();
                        while ((idx < 3) && (indice[idx] == 101) && ((millis() - prev_millis) < 10000)) {
                            for (uint8_t i = 0; i < (sizeof(gryscls)/sizeof(gryscls[0])); i++) {
                                if ((i == indice[0]) || (i == indice[1]) || (i == indice[2])) {
                                    break;
                                }
                                if (raw_gryscl_touch_white(i)) {
                                    delay(250);
                                    if (raw_gryscl_touch_white(i)) {
                                        indice[idx] = i;
                                        display.setCursor((idx * 5) + 1, 9);
                                        if (i < 10) {
                                            display.print('0');
                                        }
                                        display.print((short)i);
                                        display.display();
                                        prev_millis = millis();
                                        idx++;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    display.setCursor(0, 18);
                    display.print(STRINGS("3 gs are located"));
                    display.display();
                    display.setCursor(0, 27);
                    display.print(STRINGS("saving to EEPROM"));
                    display.display();
                    {
                        // eeprom_ptr = 0x60;
                        // for (uint8_t side = 0; side < 4; side++) {
                        //     for (uint8_t idx = 0; idx < 3; idx++) {
                        //         gryscl_map[side][idx] = EEPROM.read(eeprom_ptr++);
                        //     }
                        // }
                        uint8_t eeprom_ptr = 0x60 + page * 4;
                        for (uint8_t i = 0; i < 3; i++) {
                            const uint8_t temp = indice[i];
                            gryscl_map[page][i] = temp;
                            EEPROM.write(eeprom_ptr++, temp);
                        }
                    }
                    display.setCursor(0, 27);
                    display.print(STRINGS("saved to EEPROM "));
                    display.display();
                }
            } else if (page == 5) {
                display.clearDisplay();
                display.setCursor(0, 0);
                display.print(STRINGS("all gryscls located"));
                display.setCursor(0, 0);
                display.print(STRINGS("press to realloc"));
                display.display();
            }
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        // if (page > 0) {
                        //     page--;
                        // } else {
                        //     page = max_page_of_mode[mode] - 1;
                        // }
                        page = 1;
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        // page++;
                        // if (page >= max_page_of_mode[mode]) {
                        //     page = 0;
                        // }
                        page = 1;
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            break;
        }
        case 6:
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(STRINGS("gryscl side "));
            const char arr [] = {'F', 'L', 'R', 'B'};
            display.print(arr[page]);
            display.print(':');
            display.setCursor(0, 9);
            display.print(STRINGS("idx   raw  thrs"));
            //             [i]  anlR  thrs
            uint8_t y_coor = 18;
            for (uint8_t idx = 0; idx < 3; idx++) {
                display.setCursor(0, y_coor);
                display.print('[');
                display.print(idx);
                display.print(STRINGS("]  "));
                const int16_t val = get_gryscl(page, idx);
                if (val < 1000) {
                    display.print('0');
                }
                if (val < 100) {
                    display.print('0');
                }
                if (val < 10) {
                    display.print('0');
                }
                display.print(val < 9999 ? val : 9999);
                display.print("  ");
                const uint16_t thrs = gryscls_thresholds[gryscl_map[page][idx]];
                if (thrs < 1000) {
                    display.print('0');
                }
                if (thrs < 100) {
                    display.print('0');
                }
                if (thrs < 10) {
                    display.print('0');
                }
                display.print(thrs < 9999 ? thrs : 9999);
                if (val > thrs) {
                    display.fillRect(121, y_coor, 7, 7, SSD1306_WHITE);
                } else {
                    display.drawRect(121, y_coor, 7, 7, SSD1306_WHITE);
                }
                y_coor += 9;
            }
            display.display();
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } else {
                            page = max_page_of_mode[mode] - 1;
                        }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            break;
        }
        case 7:
            update_all_data();
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        uint8_t eeprom_0x00 = EEPROM.read(0x00);
                        eeprom_0x00 ^= (1 << 3);
                        EEPROM.update(0x00, eeprom_0x00);
                        eyes[0].begin((eeprom_0x00 & (1 << 3)) ? 0x02 : 0x01);
                        eyes[1].begin((eeprom_0x00 & (1 << 3)) ? 0x01 : 0x02);
                        if (page > 0) {
                            page--;
                        } else {
                            page = max_page_of_mode[mode] - 1;
                        }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        reset_heading();
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            break;
        case 8:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(STRINGS("gryscl"));
            display.setCursor(0, 9);
            switch (page) {
                case 0:
                    display.print(STRINGS("press button to cal"));
                    display.display();
                    break;
                case 1:
                    display.print(STRINGS("caling"));
                    display.display();
                    cal_gryscl();
                    page++;
                    break;
                case 2:
                    display.print(STRINGS("cal done"));
                    display.setCursor(0, 18);
                    display.print(STRINGS("press to cal again"));
                    display.display();
                    break;
                default:
                    page = 0;
                    break;
            }
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } // else {
                        //     page = max_page_of_mode[mode] - 1;
                        // }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            // page = 0;
                            page = 1;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            break;
        case 9:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(STRINGS("cmpas"));
            display.setCursor(0, 9);
            switch (page) {
                case 0:
                    display.print(STRINGS("press button to cal"));
                    get_heading();
                    display.display();
                    break;
                case 1:
                    display.print(STRINGS("caling"));
                    display.display();
                    cal_compass();
                    page++;
                    break;
                case 2:
                    display.print(STRINGS("cal done"));
                    display.setCursor(0, 18);
                    display.print(STRINGS("press to cal again"));
                    get_heading();
                    display.display();
                    break;
                default:
                    page = 0;
                    break;
            }
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } // else {
                        //     page = max_page_of_mode[mode] - 1;
                        // }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            // page = 0;
                            page = 1;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            break;
        case 10:
            {
                bool both_pressed = false;
                if (button_pressed(0)) {
                    while (button_pressed(0)) {
                        if (button_pressed(1)) {
                            both_pressed = true;
                            while (button_pressed(1)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        if (page > 0) {
                            page--;
                        } else {
                            page = max_page_of_mode[mode] - 1;
                        }
                    }
                    display.clearDisplay();
                }
                if ((!both_pressed) && button_pressed(1)) {
                    while (button_pressed(1)) {
                        if (button_pressed(0)) {
                            both_pressed = true;
                            while (button_pressed(0)) {}
                            break;
                        }
                    }
                    if (!both_pressed) {
                        page++;
                        if (page >= max_page_of_mode[mode]) {
                            page = 0;
                        }
                    }
                    display.clearDisplay();
                }
                if (both_pressed) {
                    page = mode - 1;
                    mode = 0;
                    for (uint8_t i = 0; i < 4; i++) {
                        mtrs[i] = 0;
                    }
                }
            }
            display.clearDisplay();
            display.setCursor(0, 0);
            switch (page) {
                case 0:
                    display.print(STRINGS("press any button"));
                    display.setCursor(0, 9);
                    display.print(STRINGS("to proceed"));
                    break;
                case 1:
                    display.setTextSize(7);
                    display.print(STRINGS("GO!"));
                    display.setCursor(0, 0);
                    display.setTextSize(1);
                    break;
                default:
                    page = 0;
                    break;
            }
            display.display();
            break;
        default:
            mode = 0;
            page = 0;
            break;
    }
    display_debug_info = old_display_debug_info;
    return ((mode == 10) && (page == 1));
}

#endif // #ifndef KOJAY_CPP