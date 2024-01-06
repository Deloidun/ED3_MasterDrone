#include <Option_Button.h>
#include <mySSD1306.h>
#include <Master_Sender.h>
#include <Voltage_Sensor.h>

void setup(){
  Initialize_ESPNOW_Transmitter();
  Initialize_Button();
  Initialize_SSD1306();
}

void loop(){
  // voltage_sensor();
  Reading_Button();
  Switch_Case();
  SerialDataWrite();
  SendingPS5Data_Through_ESPNOW();
  PrintPS5();
}

