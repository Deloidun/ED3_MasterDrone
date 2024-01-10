#pragma once

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>


#define Potentionmeter_Pin 35
#define X_Joystick_Pin 32
#define Y_Joystick_Pin 33
#define RightButton_Pin 16
#define LeftButton_Pin 17

extern int P;
extern float KalmanAngleRoll, KalmanAnglePitch;
extern float VoltageValue;

void SerialDataWrite();
void Initialize_ESPNOW_Transmitter();
void SendingPS5Data_Through_ESPNOW();
void PrintPID();
void Initialize_Timer();


