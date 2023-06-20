#ifndef GYRO_H
#define GYRO_H

#include <EEPROM.h>

#include "I2Cdev/I2Cdev.h"

#include "MPU6050/MPU6050_6Axis_MotionApps612.h"

class Gyro {
    private:
        //
    protected:
        // the MPU6050 pointer to the object
        MPU6050 *mpu;
        // the 7-bit I2C address of the chip [0x00:0x7f]
        uint8_t i2c_address;
        // stores the yaw of the gyroscope
        double prev_yaw;
        // the return-to-zero yaw
        uint16_t re_zero_yaw;
    public:
        //
        Gyro ();
        //
        ~Gyro ();
        // YOU MUST CALL ME IN void setup () FUNCTION TO USE THIS OBJECT PROPERLY
        // configures the settings of the I2C bus and the chip
        // @param init_i2c_address the 7-bit I2C address of the chip
        bool begin (const uint8_t init_i2c_address = 0x68);
        // check for presence of the chip on the I2C bus
        bool is_gyro_present (const uint8_t check_i2c_address = 0x68);
        // @brief Calibrates the gyroscope in its steady state.
        void cal ();
        // @return The heading of gyroscope in degrees [0.00:359.99].
        double get_yaw ();
        // @brief Takes the current heading of gyroscope as zero.
        void reset_yaw ();

};

#endif // #ifndef GYRO_H