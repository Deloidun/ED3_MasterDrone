#include <Arduino.h>
#include <Master_ESP_NOW.h>

void setup()
{
  // Set up UART 2 for Middle ESP32
  init_ESPNOW_Transmitter();
}

void loop()
{
  // Sending UART data from Master to Slave
  sendingData_throughESPNOW();
  // Print message to show value
  debug();
}