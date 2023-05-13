#ifndef UTS_H
#define UTS_H

#include <Arduino.h>

class Uts {
    private:
        //
    protected:
        // pins that cannot be changed:
        // the pin to trigger a sound wave
        uint8_t trig_pin;
        // the pin to read the soundwave sensor
        uint8_t echo_pin;
        // the max waiting time for the sound pulse to return (unit is microseconds)
        unsigned long max_waiting_time_in_us;
        // the previous time the echo pulse is received
        unsigned long prev_echo_time;
        // the previous reading (unit is mm)
        uint16_t reading_mm;
    public:
        // constructor
        Uts ();
        // YOU MUST CALL ME IN void setup () FUNCTION TO USE THIS OBJECT PROPERLY
        // calls pinMode function and config the pin modes
        void begin (const uint8_t init_trig_pin, const uint8_t init_echo_pin);
        // sets the maximum time the program will pause and wait for the sound pulse to return (unit is microseconds)
        void set_max_waiting_time_in_us (const unsigned long assign_max_waiting_time_in_us);
        // sets the maximum range of the sensor (unit is mm)
        void set_max_range_in_mm (const double max_range_in_mm);
        // sets the maximum range of the sensor (unit is cm)
        void set_max_range_in_cm (const double max_range_in_cm);
        // reads the distance between this ultrasound sensor and the obstacle in front of it (unit is mm)
        // returns 8888 when the returning sound wave is undetectable
        // has better performance under noInterrupts() environment
        uint16_t read_dist_mm ();
        // reads the distance between this ultrasound sensor and the obstacle in front of it (unit is cm)
        // returns 888 when the returning sound wave is undetectable
        // has better performance under noInterrupts() environment
        uint16_t read_dist_cm ();
};

#endif // #ifndef UTS_H