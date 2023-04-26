#ifndef UTS_CPP
#define UTS_CPP

#include "Uts.h"

Uts::Uts () :
    trig_pin(0), echo_pin(0), max_waiting_time_in_us(15000)
{
    //
}

void Uts::begin (const uint8_t init_trig_pin, const uint8_t init_echo_pin) {
    trig_pin = init_trig_pin;
    echo_pin = init_echo_pin;
    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
}


void yh::rec::Hc_sr04::set_max_waiting_time_in_us (const unsigned long assign_max_waiting_time_in_us) {
    max_waiting_time_in_us = assign_max_waiting_time_in_us > 23530 ? 23530 : assign_max_waiting_time_in_us; // 23529.41 us for detecting 400 cm obstacle
}

void yh::rec::Hc_sr04::set_max_range_in_mm (const double max_range_in_mm) {
    max_waiting_time_in_us = (max_range_in_mm > 4000.0 ? 4000.0 : max_range_in_mm) * 5.8823; // it takes 5.8823 us to detect a 1 mm obstable
}

void yh::rec::Hc_sr04::set_max_range_in_cm (const double max_range_in_cm) {
    max_waiting_time_in_us = (max_range_in_cm > 400.0 ? 400.0 : max_range_in_cm) * 58.823; // it takes 58.823 us to detect a 1 cm obstable
}

uint16_t yh::rec::Hc_sr04::read_dist_mm () {
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    const unsigned long duration = pulseIn(echo_pin, HIGH, max_waiting_time_in_us);
    return duration ? duration * 0.17 : 8888;
}

uint16_t yh::rec::Hc_sr04::read_dist_cm () {
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    const unsigned long duration = pulseIn(echo_pin, HIGH, max_waiting_time_in_us);
    return duration ? duration * 0.017 : 888;
}

#endif // #ifndef UTS_CPP