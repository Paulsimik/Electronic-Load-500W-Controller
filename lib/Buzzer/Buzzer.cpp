#include "Buzzer.h"

Buzzer BUZZER;

void Buzzer::InitBuzzer(uint32_t pin)
{
    _pin = pin;
    pinMode(pin, OUTPUT);
    InternalTone();
    delay(40);
    InternalNoTone();
    delay(40);
    InternalTone();
    delay(40);
    InternalNoTone();
    delay(40);
}

void Buzzer::Tick()
{
    switch (buzzerStatus)
    {
        case BEEP_IDLE: return;
        case BEEP_SHORT:
            if(millis() - beepMillis >= 15)
            {
                buzzerStatus = BEEP_IDLE;
                InternalNoTone();
            }
            break;
        case BEEP_SINGLE:
            if(millis() - beepMillis >= 50)
            {
                buzzerStatus = BEEP_IDLE;
                InternalNoTone();
            }
            break;
        case BEEP_LONG:
            if(millis() - beepMillis >= 200)
            {
                buzzerStatus = BEEP_IDLE;
                InternalNoTone();
            }
            break;
    }
}

void Buzzer::Short()
{
    if(buzzerStatus != BEEP_IDLE) 
        return;

    InternalTone();
    buzzerStatus = BEEP_SHORT; 
    beepMillis = millis();
}

void Buzzer::Single()
{
    if(buzzerStatus != BEEP_IDLE) 
        return;

    InternalTone();
    buzzerStatus = BEEP_SINGLE; 
    beepMillis = millis();
}

void Buzzer::Long()
{
    if(buzzerStatus != BEEP_IDLE) 
        return;

    InternalTone();
    buzzerStatus = BEEP_LONG; 
    beepMillis = millis();
}

void Buzzer::InternalTone()
{
    digitalWrite(_pin, HIGH);
}

void Buzzer::InternalNoTone()
{
    digitalWrite(_pin, LOW);
}