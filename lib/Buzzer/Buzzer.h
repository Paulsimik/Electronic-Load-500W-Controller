#include <Arduino.h>

#ifndef __BUZZER_H__
#define __BUZZER_H__

enum Buzzer_Status
{
    BEEP_IDLE,
    BEEP_SHORT,
    BEEP_SINGLE,
    BEEP_LONG
};

class Buzzer
{
private:
    enum Buzzer_Status buzzerStatus = BEEP_IDLE;
    uint32_t _pin;
    unsigned long beepMillis;
    bool buzzerState = false;
    void InternalTone();
    void InternalNoTone();

public:
    void InitBuzzer(uint32_t pin);
    void Tick();
    void Single();
    void Short();
    void Long();
};

extern Buzzer BUZZER;

#endif