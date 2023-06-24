#ifndef COMPOI_CPP
#define COMPOI_CPP

#include "CompoI.h"

uint8_t CompoI::command (const uint8_t cmd) {
    Wire.setClock(100000);
    Wire.beginTransmission(i2c_address);
    Wire.write(cmd);
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(1U));
    Wire.setClock(400000);
    return Wire.available() ? Wire.read() : 0;
}

CompoI::CompoI () :
    i2c_address(0x01)
{
    //
}

void CompoI::begin (const uint8_t init_i2c_address) {
    // init settings for I2C
    i2c_address = init_i2c_address;
    Wire.begin();
    Wire.setClock(400000); // set I2C to fast mode for faster communication
}

uint8_t CompoI::get_channel_val (const uint8_t channel) {
    return command(((channel < 5) ? (5) : (12)) - channel);
}

uint8_t CompoI::get_max_idx () {
    return 8 - command(8);
}

uint8_t CompoI::get_max_val () {
    return command(9);
}

uint8_t CompoI::get_min_idx () {
    return 8 - command(10);
}

uint8_t CompoI::get_min_val () {
    return command(11);
}

uint8_t CompoI::get_avg_val () {
    return command(12);
}

uint8_t CompoI::set_filter_on () {
    return command(14);
}

uint8_t CompoI::set_filter_off () {
    return command(13);
}

uint8_t CompoI::calibrate () {
    return command(15);
}

uint8_t CompoI::set_addr_0x01 () {
    return command(16);
}

uint8_t CompoI::set_addr_0x02 () {
    return command(17);
}

uint8_t CompoI::disable_vectors () {
    return command(18);
}

uint8_t CompoI::enable_vectors () {
    return command(19);
}

uint8_t CompoI::turn_off_leds () {
    return command(20);
}

uint8_t CompoI::turn_on_leds () {
    return command(21);
}

uint8_t CompoI::get_vector_dir () {
    Wire.beginTransmission(i2c_address);
    Wire.write(8);
    Wire.endTransmission();
    Wire.requestFrom(i2c_address, static_cast<uint8_t>(2U));
    if (Wire.available())
        Wire.read();
    return Wire.available() ? Wire.read() : 0;
}

#endif // #ifndef COMPOI_CPP