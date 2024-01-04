#include <mySSD1306.h>
#include <Master_Sender.h>
#include <Voltage_Sensor.h>

Adafruit_SSD1306 display (SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
PS5_Data Transmitted_PS5_Data;

int PWMValue = Transmitted_PS5_Data.Potentionmeter_PWM;

void Initialize_SSD1306(){
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){ //SSD1306_SWITCHAPVCC = generate display voltage from 3.3V internally
        Serial.println(F("SSD1306 allocation failed!"));
        for(;;); //Do not proceed, loop forever
    }
}

void DisplayDroneAngle(){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    display.print("DRONE'S ANGLE");
    display.print("\n\n  \t ROLL: ");
    display.printf("%.1f", KalmanAngleRoll);
    display.print(" deg");

    display.print("\n");

    display.print("\n  \t PITCH: ");
    display.printf("%.1f", KalmanAnglePitch);
    display.print(" deg");

    display.display();
}

void PercentDroneBattery(){
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(1,1);
    display.printf("\n%.2f", VoltageValue);
    display.printf("V");
    display.display();
}



void PercentControllerBattery(){ 
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.printf("\n%.2f", In_Voltage);
    display.display();
}


void DisplayDroneBattery(){
    display.clearDisplay();
    //Display text
    display.setTextSize(1.7); //Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); //Draw with white text
    display.setCursor(0, 0); //Start at the top left corner
    display.print(F("DRONE'S BATTERY"));
    display.display(); //Show the display buffer on the screen
    PercentDroneBattery();   
}


void DisplayControllerBattery(){
    display.clearDisplay();
    //Display text
    display.setTextSize(1.7); //Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); //Draw with white text
    display.setCursor(0, 0); //Start at the top left corner
    display.print(F("CONTROLLER'S BATTERY"));
    display.display(); //Show the display buffer on the screen
    PercentControllerBattery();
}


void PWM_Value(){
    display.setTextSize(4);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.printf("\n%.3d", Transmitted_PS5_Data.Potentionmeter_PWM);
    display.display();
}


void DisplayThrottle(){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("INPUT THROTTLE"));
    display.display();
    PWM_Value();
}

