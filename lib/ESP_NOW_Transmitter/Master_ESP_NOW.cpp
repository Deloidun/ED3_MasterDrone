#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define POT_PIN 35      // Pin 35 attached to the potentiometer

// MAC Address of the receiver - Slave ESP32
uint8_t slaveAddress[] = {0x48, 0xE7, 0x29, 0x96, 0x77, 0x44};

// PMK and LMK keys
static const char* PMK_KEY_STR = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STR = "_SON_DINH_VU_ED3";

float PRate;
float IRate;
float DRate;

float PAngle;
float IAngle;
float DAngle;

typedef struct {

  int CtrlPWM;

  byte JSX;
  byte JSY;
 
  byte leftButton;
  byte rightButton;

  float PR;
  float IR;
  float DR;

  float PA;
  float IA;
  float DA;

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

void SerialDataWrite()
{
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      switch (received_chars[0])
      {
        case 'p':
          {
            received_chars.remove(0, 1);
            PRate = received_chars.toFloat();
            break;
          }
        case 'i':
          {
            received_chars.remove(0, 1);
            IRate = received_chars.toFloat();
            break;
          }
        case 'd':
          {
            received_chars.remove(0, 1);
            DRate = received_chars.toFloat();
            break;
          }
        case 'P':
          {
            received_chars.remove(0, 1);
            PAngle = received_chars.toFloat();
            break;
          }
        case 'I':
          {
            received_chars.remove(0, 1);
            IAngle = received_chars.toFloat();
            break;
          }
        case 'D':
          {
            received_chars.remove(0, 1);
            DAngle = received_chars.toFloat();
            break;
          }
        // case 's':
        //   received_chars.remove(0, 1);
        //   float anglex_setpoint = received_chars.toFloat();
        default:
          break;
      }
      received_chars = "";
    }
  }
}

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

  transmittedData.PR = PRate;
  transmittedData.IR = IRate;
  transmittedData.DR = DRate;

  transmittedData.PA = PAngle;
  transmittedData.IA = IAngle;
  transmittedData.DA = DAngle;

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
  // Serial.print("[ ");
  // Serial.printf("P: %3d, X: %3d, Y: %3d, LB: %3d, RB: %3d",
  // transmittedData.CtrlPWM, transmittedData.JSX, transmittedData.JSY,
  // transmittedData.leftButton, transmittedData.rightButton);
  // Serial.print(" ]\t");

  // Serial.print("[ ");
  // Serial.printf("Time: %.3f, Roll: %.3f, Pitch: %.3f, Altitude: %.3f",
  // TimeCount, voltage, Roll, Pitch);
  // Serial.print(" ]\n");

  Serial.print("[ ");
  Serial.printf("PR: %.6d, IR: %.6d, DR: %.6d, PA: %.6d, IA: %.6d, DA: %.6d",
  transmittedData.PR, transmittedData.IR, transmittedData.DR,
  transmittedData.PA, transmittedData.IA, transmittedData.DA);
  Serial.print(" ]\t");
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