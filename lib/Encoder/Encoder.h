#include "Arduino.h"

#ifndef __ENCODER_H__
#define __ENCODER_H__

class Encoder
{
private:
    volatile int _position = 1;
    volatile int8_t _oldState;
   
public:
    Encoder();

    volatile int transitionCount = 0;
    volatile int encoderPos = 1;
    int lastEncoded = 0;
};

extern Encoder encoder;

#endif
