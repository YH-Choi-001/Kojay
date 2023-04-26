#ifndef MOTOR_CPP
#define MOTOR_CPP __DATE__ ", " __TIME__

#include "Motor.h"

Motor::Motor () :
    dir1_pin(0),
    dir2_pin(0),
    pwm_pin(0),
    reverse_spd(0),
    curr_spd(0)
{
    //
}

void Motor::begin (const uint8_t init_dir1_pin, const uint8_t init_dir2_pin, const uint8_t init_pwm_pin, const uint8_t init_reverse_spd = 0) {
    dir1_pin = init_dir1_pin;
    dir2_pin = init_dir2_pin;
    pwm_pin = init_pwm_pin;
    reverse_spd = init_reverse_spd;
    pinMode(pwm_pin, OUTPUT);
    digitalWrite(pwm_pin, LOW);
    pinMode(dir1_pin, OUTPUT);
    pinMode(dir2_pin, OUTPUT);
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, LOW);
}

void Motor::stop () {
    digitalWrite(pwm_pin, LOW);
    curr_spd = 0;
}

int16_t Motor::set_spd (int16_t spd) {
    if (spd < -255) {
        spd = -255;
    }
    if (spd > 255) {
        spd = 255;
    }
    curr_spd = spd;
    if (reverse_spd) {
        spd = -spd;
    }
    if (spd < 0) {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
        analogWrite(pwm_pin, -spd);
    } else {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
        analogWrite(pwm_pin, spd);
    }
    return curr_spd;
}

int16_t Motor::operator= (const int16_t spd) {
    return set_spd(spd);
}

int16_t Motor::operator+= (const int16_t spd_inc) {
    return set_spd(curr_spd + spd_inc);
}

int16_t Motor::operator-= (const int16_t spd_dec) {
    return set_spd(curr_spd - spd_dec);
}

int16_t Motor::operator++ () {
    return set_spd(curr_spd + 1);
}
int16_t Motor::operator-- () {
    return set_spd(curr_spd - 1);
}

int16_t Motor::get_spd () {
    return curr_spd;
}

Motor::operator int16_t () {
    return get_spd();
}

#endif // #ifndef MOTOR_CPP