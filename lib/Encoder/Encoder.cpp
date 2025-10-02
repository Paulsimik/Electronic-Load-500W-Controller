#include "Encoder.h"

#define ENCODER_PIN_A       PA11
#define ENCODER_PIN_B       PA12

void UpdateEncoder(void);

Encoder::Encoder()
{
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), UpdateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), UpdateEncoder, CHANGE);
}

void UpdateEncoder() 
{
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (encoder.lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoder.transitionCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoder.transitionCount--;

  encoder.lastEncoded = encoded;

  if (encoder.transitionCount >= 4) {
    encoder.encoderPos++;
    encoder.transitionCount = 0;
  } else if (encoder.transitionCount <= -4) {
    encoder.encoderPos--;
    encoder.transitionCount = 0;
  }
}