#ifndef GYRO_CPP
#define GYRO_CPP

#include "Gyro.h"

// MPU control/status vars
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Gyro::Gyro () :
    i2c_address(0x68),
    mpu(NULL),
    prev_yaw(0),
    re_zero_yaw(0)
{
    //
}

Gyro::~Gyro () {
    if (mpu != NULL) {
        delete mpu;
        mpu = NULL;
    }
}

bool Gyro::begin (const uint8_t init_i2c_address) {
    i2c_address = init_i2c_address;
    Wire.begin();
    Wire.setClock(400000);
    if (mpu != NULL) {
        delete mpu;
        mpu = NULL;
    }
    mpu = new MPU6050(i2c_address);
    mpu->initialize();
    // devStatus = mpu->dmpInitialize();
    // if (devStatus == 0) {
    if (mpu->dmpInitialize() == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        Wire.beginTransmission(i2c_address);
        Wire.write(0x13);
        Wire.write(EEPROM.read(0x30));
        Wire.write(EEPROM.read(0x31));
        Wire.write(EEPROM.read(0x32));
        Wire.write(EEPROM.read(0x33));
        Wire.write(EEPROM.read(0x34));
        Wire.write(EEPROM.read(0x35));
        Wire.endTransmission();
        mpu->setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        // packetSize = mpu->dmpGetFIFOPacketSize();
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        return false;
    }
}

bool Gyro::is_gyro_present (const uint8_t check_i2c_address) {
    Wire.beginTransmission(check_i2c_address);
    Wire.write(0x75);
    if (Wire.endTransmission()) {
        return false;
    }
    Wire.requestFrom(check_i2c_address, static_cast<uint8_t>(1U));
    if (Wire.available()) {
        const uint8_t who_am_i = Wire.read();
        return (who_am_i == 0x68) || (who_am_i == 0x70) || (who_am_i == 0x71);
    }
    return false;
}

void Gyro::cal () {
    mpu->CalibrateAccel(20);
    mpu->CalibrateGyro(20);
    Wire.beginTransmission(i2c_address);
    Wire.write(0x13);
    Wire.endTransmission();
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(6U));
    EEPROM.update(0x30,Wire.read());
    EEPROM.update(0x31,Wire.read());
    EEPROM.update(0x32,Wire.read());
    EEPROM.update(0x33,Wire.read());
    EEPROM.update(0x34,Wire.read());
    EEPROM.update(0x35,Wire.read());
}

double Gyro::get_yaw () {
    if (mpu->dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        // display Euler angles in degrees
        mpu->dmpGetQuaternion(&q, fifoBuffer);
        mpu->dmpGetGravity(&gravity, &q);
        mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
        prev_yaw = ypr[0] * 180 / M_PI - re_zero_yaw;
        while (prev_yaw < 0) {
            prev_yaw += 360;
        }
        while (prev_yaw >= 360) {
            prev_yaw -= 360;
        }
    }
    return prev_yaw;
}

void Gyro::reset_yaw () {
    while (!mpu->dmpGetCurrentFIFOPacket(fifoBuffer)) {} // Get the Latest packet
    {
        // display Euler angles in degrees
        mpu->dmpGetQuaternion(&q, fifoBuffer);
        mpu->dmpGetGravity(&gravity, &q);
        mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
        re_zero_yaw = ypr[0] * 180 / M_PI;
    }
}

#endif // #ifndef GYRO_CPP