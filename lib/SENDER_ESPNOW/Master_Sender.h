#pragma once

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

extern float KalmanAngleRoll, KalmanAnglePitch;
extern float VoltageValue;

typedef struct {
    int Potentionmeter_PWM;
    byte X_Joystick;
    byte Y_Joystick;
    byte RightButton;
    byte LeftButton;
} PS5_Data;

extern PS5_Data Transmitted_PS5_Data;

#define Potentionmeter_Pin 35
#define X_Joystick_Pin 32
#define Y_Joystick_Pin 33
#define RightButton_Pin 16
#define LeftButton_Pin 17

void Initialize_ESPNOW_Transmitter();
void SendingPS5Data_Through_ESPNOW();
// void SendingPIDData_Through_ESPNOW();
void PrintPS5();