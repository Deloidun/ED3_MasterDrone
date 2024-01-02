#include <Arduino.h>

void Initialize_Serial()
{
  Serial.begin(115200);
  while (!Serial);
}