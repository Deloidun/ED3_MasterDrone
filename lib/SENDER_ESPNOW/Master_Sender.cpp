#include <Master_Sender.h>
#include <Option_Button.h>

///////////////////////////////////////////////////////////////////
//DECLARATION
///////////////////////////////////////////////////////////////////
uint8_t SlaveMacAddress[] = {0x48, 0xE7, 0x29, 0x96, 0x77, 0x44}; //MAC address of slave ESP32
uint8_t NewMasterMacAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0x3F, 0x44}; //Install new MAC address to master ESP32

unsigned long time_prev = 0;

// float PRate = 1, IRate = 1, DRate = 2; //PID gains of Rate
// float PAngle = 3, IAngle = 4, DAngle = 5; //PID gains of Angle

float KalmanAngleRoll, KalmanAnglePitch;
float VoltageValue;
float Timer = 0;

static const char* PMK_KEY_STRING = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STRING = "_SON_DINH_VU_ED3";


///////////////////////////////////////////////////////////////////
//CREATE STRUCT AND OBJECT
///////////////////////////////////////////////////////////////////
// typedef struct {
//     float PR; //PRate
//     float IR;
//     float DR;

//     float PA; //PAngle
//     float IA;
//     float DA;
// } PID_Data;

typedef struct {
    float Time;
    float Volt;
    float K_Angle_Roll;
    float K_Angle_Pitch;
} Sensor_Data;

//CREATE STRUCTURED OBJECTS
// PID_Data Transmitted_PID_Data;
Sensor_Data Received_Sensor_Data;


///////////////////////////////////////////////////////////////////
//CREATE FUNCTIONS
///////////////////////////////////////////////////////////////////
int MapAndAdjustJoystickDeadBandValues(int Value, bool Reverse){
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

//INITIALIZE THE ESPNOW OF MASTER ESP32
void Initialize_ESPNOW_Transmitter(){
    Serial.begin(115200);
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
void EmbedControllerData(){
    pinMode (RightButton_Pin, INPUT_PULLDOWN); //Right button
    pinMode (LeftButton_Pin, INPUT_PULLDOWN); //Left button
    Transmitted_PS5_Data.Potentionmeter_PWM = map(analogRead(Potentionmeter_Pin), 0, 4095, 0, 180); //Read the pot and map the reading from [0, 4095] to [0, 180]
    Transmitted_PS5_Data.X_Joystick = MapAndAdjustJoystickDeadBandValues(analogRead(X_Joystick_Pin), false);
    Transmitted_PS5_Data.Y_Joystick = MapAndAdjustJoystickDeadBandValues(analogRead(Y_Joystick_Pin), true);
    Transmitted_PS5_Data.RightButton = digitalRead(RightButton_Pin);
    Transmitted_PS5_Data.LeftButton = digitalRead(LeftButton_Pin); 
}


//ASSIGN PID VALUES TO BE SENT
// void EmbedPIDData(){
//     Transmitted_PID_Data.PR = PRate;
//     Transmitted_PID_Data.IR = IRate;
//     Transmitted_PID_Data.DR = DRate;

//     Transmitted_PID_Data.PA = PAngle;
//     Transmitted_PID_Data.IA = IAngle;
//     Transmitted_PID_Data.DA = DAngle;
// }


//SEND THE PS5 CONTROLLER DATA THROUGH ESPNOW
void SendingPS5Data_Through_ESPNOW(){
    EmbedControllerData();
    esp_err_t myResult = esp_now_send(SlaveMacAddress, (uint8_t *) &Transmitted_PS5_Data, sizeof(Transmitted_PS5_Data));
    if (myResult == ESP_OK){
    }
}


// //SENDING PID VALUES THROUGH ESPNOW
// void SendingPIDData_Through_ESPNOW(){
//     EmbedPIDData();
//     esp_err_t myResult2 = esp_now_send(SlaveMacAddress, (uint8_t *) &Transmitted_PID_Data, sizeof(Transmitted_PID_Data));
//     if (myResult2 = ESP_OK){
//     }
// }


///////////////////////////////////////////////////////////////////
//PRINTING FOR DEBUGING
///////////////////////////////////////////////////////////////////
void PrintPS5(){
    Serial.print("\n[ ");
    Serial.printf("PWM: %.3d, XJS: %.3d, YJS: %.3d, RB: %.1d, LB: %.1d, OB: %.1d", 
    Transmitted_PS5_Data.Potentionmeter_PWM, Transmitted_PS5_Data.X_Joystick, Transmitted_PS5_Data.Y_Joystick,
    Transmitted_PS5_Data.RightButton, Transmitted_PS5_Data.LeftButton, ButtonState);
    Serial.print(" ]");
}


