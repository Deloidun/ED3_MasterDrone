#include <Master_Sender.h>
#include <Option_Button.h>

///////////////////////////////////////////////////////////////////
//DECLARATION
///////////////////////////////////////////////////////////////////
uint8_t SlaveMacAddress[] = {0x48, 0xE7, 0x29, 0x96, 0x77, 0x44}; //MAC address of slave ESP32
uint8_t NewMasterMacAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0x3F, 0x44}; //Install new MAC address to master ESP32

float PRate = 0.16;
float IRate = 0.65; //MotorInput3Compensation
float DRate = 2.45;

float PAngle = 5.4;
float IAngle = 0.0;
float DAngle = 0.0;

int P;
float KalmanAngleRoll, KalmanAnglePitch;
float VoltageValue;
float Timer = 0;

static const char* PMK_KEY_STRING = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STRING = "_SON_DINH_VU_ED3";

unsigned long StartTime; //Variable to store time for tracking and unlocking I gain
bool PotentionmeterFlag = false; //Variable to track if P has exceeded 20

///////////////////////////////////////////////////////////////////
//CREATE STRUCT AND OBJECT
///////////////////////////////////////////////////////////////////
typedef struct {
    int Potentionmeter_PWM;
    byte X_Joystick;
    byte Y_Joystick;
    byte RightButton;
    byte LeftButton;

    float PR;
    float IR;
    float DR;

    float PA;
    float IA;
    float DA;
} PS5_Data;

//CREATE STRUCTURED OBJECTS
PS5_Data Transmitted_Data;

typedef struct {
    float Time;
    float Volt;
    float K_Angle_Roll;
    float K_Angle_Pitch;
} Sensor_Data;

//CREATE STRUCTURED OBJECTS
Sensor_Data Received_Sensor_Data;


///////////////////////////////////////////////////////////////////
//FUNCTIONS DECLARATION
///////////////////////////////////////////////////////////////////

void Initialize_Timer(){
    StartTime = millis();
}

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
        case 'q':
            received_chars.remove(0, 1);
            PRate = received_chars.toFloat();
            break;
        case 'w':
            received_chars.remove(0, 1);
            IRate = received_chars.toFloat();
            break;
        case 'e':
            received_chars.remove(0, 1);
            DRate = received_chars.toFloat();
            break;
        case 'a':
            received_chars.remove(0, 1);
            PAngle = received_chars.toFloat();
            break;
        case 's':
            received_chars.remove(0, 1);
            IAngle = received_chars.toFloat();
            break;
        case 'd':
            received_chars.remove(0, 1);
            DAngle = received_chars.toFloat();
            break;

        case 'r':
            PRate = IRate = DRate = PAngle = IAngle = DAngle = 0;
            break;

        // case 'u': //Save PID gains
        //     // //New best
        //     // PRate = 0.16;
        //     // IRate = 0.97; //Motorinput3Compensate
        //     // DRate = 1.5;
            
        //     // PAngle = 5.0;
        //     // IAngle = 0.00055;
        //     // DAngle = 0;

        //     //Best of the best
        //     PRate = 0.16;
        //     IRate = 0.97; //Motorinput3Compensate
        //     DRate = 1.5;
            
        //     PAngle = 5.33;
        //     IAngle = 0.0;
        //     DAngle = 0;
        //     break;
        default:
        break;

      }
      received_chars = "";
    }
  }

    //Logic for automatically turn on and off IAngle
    if (P < 20){ //PWM value < 20 will turn off IAngle and reset time
        StartTime = millis();
        IAngle = 0;
    }
    //PWM value > 20 and the condition of PotentionmeterFlag is false
    else if (P > 20 && !PotentionmeterFlag){
        StartTime = millis(); //Reset timer
        PotentionmeterFlag = true; //Turn flag to true so that the condition in else if below satisfy
    }
    
    //Three conditions must be satisfied to turn on the IAngle:
    //PotentionmeterFlag = true
    //Time larger than 5
    //IAngle = 0;
    else if (PotentionmeterFlag && (millis() - StartTime) > 2500 && IAngle == 0){
        IAngle = 0.00095;
    }
}


int Map_And_Adjust_Joystick_Dead_Band_Values(int Value, bool Reverse){
    if (Value >= 2200){
        Value = map(Value, 2200, 4095, 127, 254);
    }
    else if (Value <= 1800){
        Value = (Value == 0 ? 0 : map(Value, 1800, 0, 127, 0));
    }
    else{
        Value = 127;
    }

    if (Reverse){
        Value = 254 - Value;
    }
    return Value;
}

//CALLBACK FUNCTION WHEN DATA IS SENT
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t Status){
}

//CALLBACK FUNCTION CALLED WHEN DATA IS RECEIVED
void OnDataRecv(const uint8_t * mac, const uint8_t * IncomingData, int Len){
    memcpy (&Received_Sensor_Data, IncomingData, sizeof (Received_Sensor_Data));
    Timer = Received_Sensor_Data.Time;
    VoltageValue = Received_Sensor_Data.Volt;
    KalmanAngleRoll = Received_Sensor_Data.K_Angle_Roll;
    KalmanAnglePitch = Received_Sensor_Data.K_Angle_Pitch;
}

void Initialize_Serial()
{
  Serial.begin(115200);
  while (!Serial);
}

//INITIALIZE THE ESPNOW OF MASTER ESP32
void Initialize_ESPNOW_Transmitter()
{
    Initialize_Serial();
    WiFi.mode(WIFI_STA); //Set ESP32 as a WIFI station
    esp_wifi_set_mac (WIFI_IF_STA, NewMasterMacAddress); //Install a same address to any esp32
    
    //Initialize ESPNOW
    if (esp_now_init() != ESP_OK){
        Serial.println("Error initializing ESPNOW!");
        return;
    }

    //Create a slave peer object
    esp_now_peer_info_t SlavePeer;

    //Set the PMK key
    esp_now_set_pmk((uint8_t *) PMK_KEY_STRING);

    //Register peer
    memset(&SlavePeer, 0, sizeof(SlavePeer));
    memcpy(SlavePeer.peer_addr, SlaveMacAddress, 6);
    SlavePeer.channel = 0;

    //Set the device's LMK
    for (uint8_t i = 0; i < 16; i++){
        SlavePeer.lmk[i] = LMK_KEY_STRING[i];
    }
    SlavePeer.encrypt = true;

    //Add peer
    if (esp_now_add_peer(&SlavePeer) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    //Register the callback for sent Transmitted and Received Data
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

//ASSIGN BUTTONS AND SETUP CONTROLLERs
void Controller_Button_Mode()
{
    pinMode(RightButton_Pin, INPUT_PULLDOWN); //Right button
    pinMode(LeftButton_Pin, INPUT_PULLDOWN); //Left button
}

//SEND THE PS5 CONTROLLER DATA THROUGH ESPNOW
void SendingPS5Data_Through_ESPNOW()
{
    Controller_Button_Mode();

    Transmitted_Data.Potentionmeter_PWM = map(analogRead(Potentionmeter_Pin), 0, 4095, 0, 120); //Read the pot and map the reading from [0, 4095] to [0, 180]
    Transmitted_Data.X_Joystick = Map_And_Adjust_Joystick_Dead_Band_Values(analogRead(X_Joystick_Pin), false);
    Transmitted_Data.Y_Joystick = Map_And_Adjust_Joystick_Dead_Band_Values(analogRead(Y_Joystick_Pin), true);
    Transmitted_Data.RightButton = digitalRead(RightButton_Pin);
    Transmitted_Data.LeftButton = digitalRead(LeftButton_Pin);

    Transmitted_Data.PR = PRate;
    Transmitted_Data.IR = IRate;
    Transmitted_Data.DR = DRate;

    Transmitted_Data.PA = PAngle;
    Transmitted_Data.IA = IAngle;
    Transmitted_Data.DA = DAngle;

    P = Transmitted_Data.Potentionmeter_PWM;

    esp_err_t myResult = esp_now_send(SlaveMacAddress, (uint8_t *) &Transmitted_Data, sizeof(Transmitted_Data));
}

///////////////////////////////////////////////////////////////////
//PRINTING FOR DEBUGING
///////////////////////////////////////////////////////////////////
void PrintPID(){

    Serial.printf("\n[PR: %.5f, MT3: %.5f, DR: %.5f, PA: %.5f, IA: %.7f, DA: %.5f", 
    Transmitted_Data.PR, Transmitted_Data.IR, Transmitted_Data.DR,
    Transmitted_Data.PA, Transmitted_Data.IA, Transmitted_Data.DA);
    Serial.print(" ]");
}


void MATLAB_Print(){
  Serial.println(String(KalmanAnglePitch) + " " + String(KalmanAngleRoll) + " " + String(VoltageValue) + " " + String(P) + " " + String(Transmitted_Data.X_Joystick) + " " + String( Transmitted_Data.Y_Joystick) + " " + String(Transmitted_Data.LeftButton)  + " " + String(Transmitted_Data.RightButton));
}

void Print_PS5_Value(){
    Serial.print("\n[ ");
    Serial.printf("PWM: %.3d, XJS: %.3d, YJS: %.3d, RB: %.1d, LB: %.1d", 
    Transmitted_Data.Potentionmeter_PWM, Transmitted_Data.X_Joystick, Transmitted_Data.Y_Joystick,
    Transmitted_Data.RightButton, Transmitted_Data.LeftButton);
}