#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define POT_PIN 35      // Pin 35 attached to the potentiometer

// MAC Address of the receiver - Slave ESP32
uint8_t slaveAddress[] = {0x48, 0xE7, 0x29, 0x96, 0x77, 0x44};

// PMK and LMK keys
static const char* PMK_KEY_STR = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STR = "_SON_DINH_VU_ED3";

typedef struct {

  int CtrlPWM;

  byte JSX;
  byte JSY;
 
  byte leftButton;
  byte rightButton;

} Controller_Data;

// Create a structured object
Controller_Data transmittedData;

///////////////////////////////////////////
float TimeCount;
float voltage;
float Roll;
float Pitch;

typedef struct {

  float time;
  float V;
  float k_angle_roll;
  float k_angle_pitch;

} Sensor_Data;

// Create a structured object
Sensor_Data receivedData;
////////////////////////////////////////////

//This function is used to map 0-4095 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Joystick values range from 0-4095. But its center value is not always 2047. It is little different.
//So we need to add some deadband to center value. in our case 1800-2200. Any value in this deadband range is mapped to center 127.
int mapAndAdjustJoystickDeadBandValues(int value, bool reverse)
{
  if (value >= 2200)
  {
    value = map(value, 2200, 4095, 127, 254);
  }
  else if (value <= 1800)
  {
    value = (value == 0 ? 0 : map(value, 1800, 0, 127, 0));
  }
  else
  {
    value = 127;
  }

  if (reverse)
  {
    value = 254 - value;
  }
  // Serial.println(value);
  return value;
}

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	// Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery SUCCESS!" : "Delivery FAIL...");
}

// Callback function called when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t * incomingData, int len){
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  TimeCount = receivedData.time;
  voltage = receivedData.V;
  Roll = receivedData.k_angle_roll;
  Pitch = receivedData.k_angle_pitch;
}

void init_ESPNOW_Transmitter()
{
  Serial.begin(115200);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Create a slave peer object
  esp_now_peer_info_t slavePeer;

  // Set the PMK key
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  // Register peer
  memset(&slavePeer, 0, sizeof(slavePeer));
  memcpy(slavePeer.peer_addr, slaveAddress, 6);
  slavePeer.channel = 0;

		///*** Set the middle device's LMK ***///
		for (uint8_t i = 0; i < 16; i++)
    {
      slavePeer.lmk[i] = LMK_KEY_STR[i];
    }

  slavePeer.encrypt = true;
  
  // Add peer        
  if (esp_now_add_peer(&slavePeer) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }


  pinMode(16,INPUT_PULLDOWN);
  pinMode(17,INPUT_PULLDOWN);
  
	// Register the callback for sent transmittedData
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}
 
void sendingData_throughESPNOW() 
{
  transmittedData.CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180);  // Read the pot, map the reading from [0, 4095] to [0, 180]
  // Serial.println(transmittedData.CtrlPWM);

  transmittedData.JSX = mapAndAdjustJoystickDeadBandValues(analogRead(32), false);
  transmittedData.JSY = mapAndAdjustJoystickDeadBandValues(analogRead(33), true);

  transmittedData.leftButton = digitalRead(17);
  transmittedData.rightButton = digitalRead(16);
  // Serial.println(transmittedData.leftButton);
  // Serial.println(transmittedData.rightButton);


  esp_err_t result = esp_now_send(slaveAddress, (uint8_t *) &transmittedData, sizeof(transmittedData));
  if (result == ESP_OK)
  {
    // Serial.println("Sent with success");
  }
  // else 
  // {
  //   Serial.println("Error sending the transmittedData");
  // }
}

void debug()
{
  Serial.print("[ ");
  Serial.printf("P: %3d, X: %3d, Y: %3d, LB: %3d, RB: %3d",
  transmittedData.CtrlPWM, transmittedData.JSX, transmittedData.JSY,
  transmittedData.leftButton, transmittedData.rightButton);
  Serial.print(" ]\t");

  Serial.print("[ ");
  Serial.printf("Time: %.3f, Roll: %.3f, Pitch: %.3f, Altitude: %.3f",
  TimeCount, voltage, Roll, Pitch);
  Serial.print(" ]\n");
}
// void SerialDataPrint()
// {
//   if (micros() - time_prev >= 20000)
//   {
//     time_prev = micros();-
//     Serial.print(millis());
//     Serial.print("\t");
//     Serial.println(transmittedData.CtrlPWM);
//   }
// }