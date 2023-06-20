#ifndef CMPAS_H
#define CMPAS_H

#include <Arduino.h>
#include <Wire.h>

struct Mag_field_raw_t {
    int16_t x, y, z;
};

class Cmpas {
    private:
        // the 7-bit I2C address of the chip [0x00:0x7f]
        uint8_t i2c_address;
        // stores the raw values of field strength in each axis
        Mag_field_raw_t raw_data;
        // stores the heading of the compass
        uint16_t heading;
    public:
        // hard iron calibration value
        int16_t base_x, base_y, base_z;
        // soft iron calibration value
        int16_t range_x, range_y, range_z;
        // the return-to-zero heading
        uint16_t re_zero_heading;
        //
        Cmpas ();
        // YOU MUST CALL ME IN void setup () FUNCTION TO USE THIS OBJECT PROPERLY
        // configures the settings of the I2C bus and the chip
        // @param init_i2c_address the 7-bit I2C address of the chip
        bool begin (const uint8_t init_i2c_address = 0x0D);

        // low-level functions

        // @brief Resets the QMC5883L in software.
        bool soft_reset ();
        // @brief Updates new data from the QMC5883L.
        bool update ();
        // @brief Calibrates the compass, and find the ranges of each axis respectively for compass usage.
        // ATTENTION: WHEN MAGNETOMETER CALIBRATION IS IN PROGRESS, SWING THE CHIP IN ALL DIRECTIONS OF X, Y AND Z.
        bool compass_cal ();
        // @brief Takes the current heading of compass as zero.
        bool reset_heading ();
        // @return The heading of compass in degrees [0:359].
        uint16_t get_heading ();
        // The gain is factory-calibrated, but not the offset,
        // so only the relative temperature is accurate.
        // @return The raw value of temperature sensor recorded by the chip in 2's complement (100 LSB / degree Celsius).
        int16_t get_raw_temp ();
        // reads the identification register
        uint8_t id ();
        // check for presence of the chip on the I2C bus
        bool is_cmpas_present ();
};

#endif // #ifndef CMPAS_H