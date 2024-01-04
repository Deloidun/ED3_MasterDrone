#include <Option_Button.h>
#include <mySSD1306.h>

const int ButtonPin = 34; //Define the pin where the button is connected
int ButtonState = HIGH; //Variable to store the current state of the button
int LastButtonState = HIGH; //Variable to store the last state of the button
unsigned long LastDebounceTime = 0; //The last time the output pin was toggled
unsigned long DebounceDelay = 50; //Debounce time;
int DisplayMode = 0; // 0 for Mode 1, 1 for Mode 2, 2 for Mode 3
bool ModeChanged = false;

void Initialize_Button(){
    pinMode(ButtonPin, INPUT);
}


void Reading_Button(){
    int Reading = digitalRead(ButtonPin); //Read the state of the button into a local variable
    
    //Check if the button state has changed
    if (Reading != LastButtonState){
        LastDebounceTime = millis();
    }

    if ((millis() - LastDebounceTime) > DebounceDelay){
        //Update the button state
        if (Reading != ButtonState && Reading == LOW){
            ButtonState = Reading;

            //Cycle through display modes
            DisplayMode = (DisplayMode + 1) % 4;
            ModeChanged = true;
        }
        ButtonState = Reading;
    }
    LastButtonState = Reading;
}

void Switch_Case(){
    switch(DisplayMode){
    // case 0:
    //     if (ModeChanged){
    //         DisplayControllerBattery();
    //         ModeChanged = false; //Reset the flag after updating
    //     }
    //     break;

    case 0:
        DisplayDroneBattery();
        break;

    case 1:
        DisplayDroneAngle();
        break;

    case 2:
        DisplayThrottle();
        break;

    case 3:
        DisplayControllerBattery();
        break;
    }
}





// void Reading_Button(){
//     int Reading = digitalRead(ButtonPin); //Read the state of the button into a local variable
    
//     //Check if the button state has changed
//     if (Reading != LastButtonState){
//         LastDebounceTime = millis();
//     }

//     if ((millis() - LastDebounceTime) > DebounceDelay){
//         //Update the button state
//         if (Reading != ButtonState && Reading == LOW){
//             ButtonState = Reading;

//             //Cycle through display modes
//             DisplayMode = (DisplayMode + 1) % 3;
//             switch(DisplayMode){
//                 case 0:
//                     DisplayControllerBattery();
//                     break;
//                 case 1:
//                     DisplayDroneBattery();
//                     break;
//                 case 2:
//                     DisplayDroneAngle();
//                     break;
//             }
//         }
//         else{
//             ButtonState = Reading; //Update the butotn state
//         }
//     }
//     LastButtonState = Reading; //Save the reading for the next loop
// }

// void Reading_Button(){
//     int Reading = digitalRead(ButtonPin); //Read the state of the button into a local variable
    
//     //Check if the button state has changed
//     if (Reading != LastButtonState){
//         LastDebounceTime = millis();
//     }

//     if ((millis() - LastDebounceTime) > DebounceDelay){
//         //Update the button state
//         if (Reading != ButtonState && Reading == LOW){
//             ButtonState = Reading;

//             //Cycle through display modes
//             DisplayMode = (DisplayMode + 1) % 3;
//         }
//         ButtonState = Reading;
//     }
//     LastButtonState = Reading;
// }

// void Switch_Case(){
//     switch(DisplayMode){
//     case 0:
//         DisplayControllerBattery();
//         break;
//     case 1:
//         DisplayDroneBattery();
//         break;
//     case 2:
//         DisplayDroneAngle();
//         break;
//     }
// }



