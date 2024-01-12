#include <Option_Button.h>
#include <mySSD1306.h>
#include <Master_Sender.h>
#include <Preferences.h>

void setup(){
  Initialize_ESPNOW_Transmitter();
  Initialize_Button();
  Initialize_SSD1306();
  Initialize_Timer();
}

void loop(){
  Reading_Button();
  Switch_Case();
  SerialDataWrite(); //Live PID tunning function
  SendingPS5Data_Through_ESPNOW();
  // PrintPID();
  MATLAB_Print();
}

