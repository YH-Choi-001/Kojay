#ifndef COMPOI_H
#define COMPOI_H

#include <Arduino.h>
#include <Wire.h>

class CompoI {
    private:
        //
    protected:
        //
        uint8_t i2c_address;
        virtual uint8_t command (const uint8_t cmd);
    public:
        CompoI ();
        // YOU MUST CALL ME IN void setup () FUNCTION TO USE THIS OBJECT PROPERLY
        // configures the settings of the I2C bus and the chip
        virtual void begin (const uint8_t init_i2c_address = 0x01);
        // @param channel selects the channel of compound-eye to read its value
        // @return the IR intensity read by the photodiodes of the selected channel
        uint8_t get_channel_val (const uint8_t channel);
        // @return the no. of photodiode that reads the strongest IR intensity
        uint8_t get_max_idx ();
        // @return the strongest IR intensity among the readings of the photodiodes of all channels
        uint8_t get_max_val ();
        // @return the no. of photodiode that reads the weakest IR intensity
        uint8_t get_min_idx ();
        // @return the weakest IR intensity among the readings of the photodiodes of all channels
        uint8_t get_min_val ();
        // @return the mean average IR intensity among the readings of the photodiodes of all channels
        uint8_t get_avg_val ();
        // enters ball-filtering mode
        // environmental IR signals will be removed from readings of photodiodes
        uint8_t set_filter_on ();
        // quits ball-filtering mode
        // readings of photodiodes are unprocessed raw values
        uint8_t set_filter_off ();
        // initializes the hardware
        // completed by factory, no need for user to call this method
        uint8_t calibrate ();
        // sets the address of the compound eye as 0x01
        uint8_t set_addr_0x01 ();
        // sets the address of the compound eye as 0x02
        uint8_t set_addr_0x02 ();
};

#endif // #ifndef COMPOI_H