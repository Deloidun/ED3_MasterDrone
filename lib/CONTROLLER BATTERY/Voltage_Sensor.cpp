#include <Arduino.h>

// Define analog input
#define VOLTAGE_SENSOR_PIN 39
 
// Floats for ADC voltage & Input voltage
float adc_Voltage = 0.0;
float In_Voltage = 0.0;
 
// Floats for resistor values in divider (in ohms)
float R1 = 3002.0;
float R2 = 7501.0; 
 
// Float for Reference Voltage
float Ref_Voltage = 5.0;
 
// Integer for ADC value
int adc_value = 0;
 
void voltage_sensor()
{
  // Read the Analog Input
  adc_value = analogRead(VOLTAGE_SENSOR_PIN);
  
  // Determine voltage at ADC input
  adc_Voltage  = (adc_value * Ref_Voltage) / 4095.0;
  
  // Calculate voltage at divider input
  In_Voltage = adc_Voltage/(R1/(R1+R2));
}