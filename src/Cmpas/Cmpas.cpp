#ifndef CMPAS_CPP
#define CMPAS_CPP

#include "Cmpas.h"

Cmpas::Cmpas () :
    i2c_address(0x0D),
    heading(0)
{
    //
}

bool Cmpas::begin (const uint8_t init_i2c_address) {
    i2c_address = init_i2c_address;
    // init settings to the GY-271 module through I2C
    Wire.begin();
    Wire.setClock(400000); // set I2C to fast mode for faster communication
    Wire.beginTransmission(i2c_address); // address QMC5883L compass
    Wire.write(0x0B); // select register 0x0B - SET/RESET Period Register
    Wire.write(0x01); // write value of 0x01 (recommended by datasheet) to the register
    if (Wire.endTransmission()) {
        return 0;
    }
    Wire.beginTransmission(i2c_address); // address QMC5883L compass
    Wire.write(0x09); // select register 0x09 - Control Register 1
    Wire.write((0b00 << 6) | (0b01 << 4) | (0b11 << 2) | (0b01 << 0)); // write the selected configurations to the register
    if (Wire.endTransmission()) {
        return 0;
    }
    return 1;
}

bool Cmpas::soft_reset () {
    Wire.beginTransmission(i2c_address); // address QMC5883L compass
    Wire.write(0x0A); // select register 0x0A - Control Register 2
    Wire.write(1 << 7); // write SOFT_RST bit to high
    return !Wire.endTransmission();
}

bool Cmpas::update () {
    bool new_data_updated = false;
    Wire.beginTransmission(i2c_address); // address QMC5883L compass
    Wire.write(0x06); // select register 0x06 - Status Register
    Wire.endTransmission();
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1U));
    const uint8_t status = Wire.read();
    if (status & (1 << 2)) { // check if new data is ready
        if (!(status & (1 << 1))) { // check if overflow does not occur
            Wire.beginTransmission(i2c_address); // address QMC5883L compass
            Wire.write(0x00); // select register 0x00 - XOUT_L
            Wire.endTransmission();
            Wire.requestFrom(i2c_address, static_cast<uint8_t>(6U)); // request 6 bytes from compass
            const uint8_t xlow = Wire.available() ? Wire.read() : 0;
            const uint8_t xhigh = Wire.available() ? Wire.read() : 0;
            raw_data.x = (xhigh << 8) | xlow;
            const uint8_t ylow = Wire.available() ? Wire.read() : 0;
            const uint8_t yhigh = Wire.available() ? Wire.read() : 0;
            raw_data.y = (yhigh << 8) | ylow;
            const uint8_t zlow = Wire.available() ? Wire.read() : 0;
            const uint8_t zhigh = Wire.available() ? Wire.read() : 0;
            raw_data.z = (zhigh << 8) | zlow;
            new_data_updated = true;
        }
    }
    return new_data_updated;
}

bool Cmpas::compass_cal () {
    int16_t max [3];
    int16_t min [3];
    update();
    max[0] = min[0] = raw_data.x;
    max[1] = min[1] = raw_data.y;
    max[2] = min[2] = raw_data.z;
    unsigned long prev_update_time = millis();
    while (millis() - prev_update_time < 5000) {
        if(update()) {
            bool updated = false;
            if (raw_data.x > max[0]) {
                max[0] = raw_data.x;
                updated = true;
            }
            if (raw_data.y > max[1]) {
                max[1] = raw_data.y;
                updated = true;
            }
            if (raw_data.z > max[2]) {
                max[2] = raw_data.z;
                updated = true;
            }
            if (raw_data.x < min[0]) {
                min[0] = raw_data.x;
                updated = true;
            }
            if (raw_data.y < min[1]) {
                min[1] = raw_data.y;
                updated = true;
            }
            if (raw_data.z < min[2]) {
                min[2] = raw_data.z;
                updated = true;
            }
            if (updated) {
                prev_update_time = millis();
            }
        }
    }
    base_x = (max[0] + min[0]) / 2;
    range_x = max[0] - min[0];
    base_y = (max[1] + min[1]) / 2;
    range_y = max[1] - min[1];
    base_z = (max[2] + min[2]) / 2;
    range_z = max[2] - min[2];
    return true;
}

bool Cmpas::reset_heading () {
    const unsigned long starting_millis = millis();
    while (!update()) {
        // wait for it to be updated
        if (millis() - starting_millis > 100) {
            re_zero_heading = 0;
            return false;
        }
    }
    const double
        cal_x = static_cast<double>(raw_data.x - base_x) / range_x,
        cal_y = static_cast<double>(raw_data.y - base_y) / range_y;
    int16_t temp_heading = atan2(cal_y, cal_x) * RAD_TO_DEG;
    while (temp_heading < 0) {
        temp_heading += 360;
    }
    re_zero_heading = temp_heading;
    return true;
}

uint16_t Cmpas::get_heading () {
    if (update()) {
        const double
            cal_x = static_cast<double>(raw_data.x - base_x) / range_x,
            cal_y = static_cast<double>(raw_data.y - base_y) / range_y;
        int16_t temp_heading = atan2(cal_y, cal_x) * RAD_TO_DEG - re_zero_heading;
        while (temp_heading < 0) {
            temp_heading += 360;
        }
        while (temp_heading >= 360) {
            temp_heading -= 360;
        }
        heading = temp_heading;
    }
    return heading;
}

int16_t Cmpas::get_raw_temp () {
    Wire.beginTransmission(i2c_address); // address QMC5883L compass
    Wire.write(0x07); // select register 0x06 - TOUT_L
    Wire.endTransmission();
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(2U)); // request 2 bytes from compass
    uint8_t low = Wire.available() ? Wire.read() : 0;
    uint8_t high = Wire.available() ? Wire.read() : 0;
    return (high << 8) | low;
}

uint8_t Cmpas::id () {
    Wire.beginTransmission(i2c_address); // address QMC5883L compass
    Wire.write(0x0D); // select register 0x0D - Chip ID
    if (Wire.endTransmission()) {
        return 0;
    }
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1U)); // request 1 byte from compass
    return Wire.available() ? Wire.read() : 0;
}

bool Cmpas::is_cmpas_present () {
    return (id() == 0xff);
}

#endif // #ifndef CMPAS_CPP