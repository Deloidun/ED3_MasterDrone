#include <Arduino.h>
#include <Master_ESP_NOW.h>
#include <mySSD1306.h>

void setup()
{
  // Set up UART 2 for Middle ESP32
  init_ESPNOW_Transmitter();
  SSD1306_Setup(); //Screen setup
}

void loop()
{
  // Sending UART data from Middle to Slave
  DrawMrSonBitMap(); //Draw Mr Son
  sendingData_throughESPNOW();
}

