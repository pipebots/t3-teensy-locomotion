#ifndef encoder_h
#define encoder_h

#include "Arduino.h"

/*
* Class for an encoder.
* wheel_position keeps track so we know when to incriment total_revolutions
* ticks is the current total raw encoder value
* previous_ticks is the previous total raw encoder value. This is used
* to calcualte the speed.
* Using technique from here http://www.gammon.com.au/forum/?id=12983 to access
* ISR inside a class - this means the glue functions need extending for more
* than two instances of this class.
*/
class Encoder{
    int pin_A_, pin_B_, counts_per_revolution_;
    float inv_counts_per_revolution_;
    bool inverse;
    static int inst_counter; // count instances of class
    static Encoder * instances[2];
    static void count_glue_0();
    static void count_glue_1();
    static void count_glue_0_inv();
    static void count_glue_1_inv();

    void count();
    void count_inverse();

 public:
    Encoder(const char *,  const unsigned int, const unsigned int,
            const unsigned int, const char *, const bool);
    bool setup();
    float ticks_to_degrees();
    float ticks_to_rad();
    volatile int total_revolutions, wheel_position, ticks;
    const char *name;
    const char *hardware_id;
};

#endif
