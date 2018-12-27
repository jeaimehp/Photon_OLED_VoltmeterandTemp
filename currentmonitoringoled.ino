// This #include statement was automatically added by the Particle IDE.
#include <ThingSpeak.h>

// This #include statement was automatically added by the Particle IDE.
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ArduinoJson.h>

/****************************************************
 * Program By: Je'aime Powell                       *
 * Purpose: Sensor to detect, and publish temp (C), *
 * voltage (v), and current (Amps)                  *
 * Publishes to Console, OLED and Google Drive      *
 *                                                  *
 * Pin connections on Particle Photon:              *
 *                                                  *
 *  A0 <== Max471 GY-471 Out                        *   
 *  A1 <== Voltage Divider Out                      *
 *  A6 <== Thermistor1 Signal                       *
 *  A7 <== Thermister2 Signal                       *
 *  D2 <== DS18B20 Signal                           *
 *  D5 <== Relay IN1 (Buck Converter)               *
 *                                                  *
 * *************************************************/
 
/////////////////////////
//       Libraries     //
////////////////////////

// Temperature Probe
// This #include statement was automatically added by the Particle IDE.
#include <DS18B20.h>


// Sparkfun Particle Photon OLED Shield
// This #include statement was automatically added by the Particle IDE.
#include <SparkFunMicroOLED.h>

// Additional Math Functions
#include "math.h"

// Current Sensor
//MAX471 Pin Definition 
#define max471out A0

// Voltage Sensor
// Voltage Divider to get Current
#define vdout A1
#define Arduino_Voltage 5.0


// Thermistor Sensor
    
#define THERMISTORPIN A6
#define SERIESRESISTOR 10000 //10K resistor on voltage
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
#define THERMISTORPIN2 A7
  
// Relay
#define RELAY1 D5


/*
Micro-OLED-Shield-Example.ino
SparkFun Micro OLED Library Hello World Example
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 22, 2015

This sketch prints a friendly, recognizable logo on the OLED Shield, then
  goes on to demo the Micro OLED library's functionality drawing pixels,
  lines, shapes, and text.

  Hardware Connections:
  This sketch was written specifically for the Photon Micro OLED Shield, which does all the wiring for you. If you have a Micro OLED breakout, use the following hardware setup:

    MicroOLED ------------- Photon
      GND ------------------- GND
      VDD ------------------- 3.3V (VCC)
    D1/MOSI ----------------- A5 (don't change)
    D0/SCK ------------------ A3 (don't change)
      D2
      D/C ------------------- D6 (can be any digital pin)
      RST ------------------- D7 (can be any digital pin)
      CS  ------------------- A2 (can be any digital pin)

  Development environment specifics:
    IDE: Particle Build
    Hardware Platform: Particle Photon
                       SparkFun Photon Micro OLED Shield

  This code is beerware; if you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!

  Distributed as-is; no warranty is given.
*/

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
// Declare a MicroOLED object. If no parameters are supplied, default pins are
// used, which will work for the Photon Micro OLED Shield (RST=D7, DC=D6, CS=A2)

MicroOLED oled;




/////////////
// DS18B20
// Pin D2 
////////////
const int      MAXRETRY          = 4;
const uint32_t msSAMPLE_INTERVAL = 2500;
const uint32_t msMETRIC_PUBLISH  = 5000;
const uint32_t msDATA_PUBLISH  = 30000;
uint32_t msLast_Data_Sample = 0; 

DS18B20  ds18b20(D2, true); //Sets Pin D2 for Water Temp Sensor and 
                            // this is the only sensor on bus
char     szInfo[64];
double   celsius;
double   fahrenheit;
uint32_t msLastMetric;
uint32_t msLastSample;
int relaystate = 0;

//Thingspeak API
TCPClient client;
unsigned int myChannelNumber = 663664;
const char * myWriteAPIKey = "XXXXXXXXXXXXX"; //<--Remove API for Github upload

///////////
// INA219
//////////


//Adafruit_INA219 ina219;



void setup()
{
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  delay(1000);     // Delay 1000 ms
  oled.clear(PAGE); // Clear the buffer.

  //randomSeed(analogRead(A0) + analogRead(A1));
  
  
  uint32_t currentFrequency;
  
     // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  //ina219.begin();
  
  ///MAX471 Pin Definition
  pinMode(max471out, INPUT);
  
  //voltage Divider Pin Definition
  pinMode(vdout, INPUT);
  
  //Themistor Pin Definition
  pinMode(THERMISTORPIN, INPUT); 
  
  //Themistor2 Pin Definition
  pinMode(THERMISTORPIN2, INPUT);
  
  //Pin Relay turn on buckconverter
  pinMode(RELAY1, OUTPUT);
  //Setup Function 
  Particle.function("buckconverter" , buckconverter);
  
  ThingSpeak.begin(client);
  
}

void loop()
{
 
    if (millis() - msLastSample >= msSAMPLE_INTERVAL){
    getTemp();
  }

  if (millis() - msLastMetric >= msMETRIC_PUBLISH){
    //Serial.println("Publishing now.");
    publishData();
  }
  
  
}


void printTitle(String title, int font)
{
  int middleX = oled.getLCDWidth() / 2;
  int middleY = oled.getLCDHeight() / 2;

  oled.clear(PAGE);
  oled.setFontType(font);
  // Try to set the cursor in the middle of the screen
  oled.setCursor(middleX - (oled.getFontWidth() * (title.length()/2)),
                 middleY - (oled.getFontWidth() / 2));
  // Print the title:
  oled.print(title);
  oled.display();
  delay(1500);
  oled.clear(PAGE);
}   
//////////////////////////////////////////////////////////////////////////////
//   Publish to OLED and Console Function
//
//////////////////////////////////////////////////////////////////////////////
void publishData(){
  //Create JSON string
  //ref: https://arduinojson.org/ and https://www.youtube.com/watch?v=z_rPvQhKTfY
  DynamicJsonBuffer jBuffer;
  JsonObject& root = jBuffer.createObject();
  
  sprintf(szInfo, "%2.2f", celsius);
  //Particle.publish("dsTmp", szInfo, PRIVATE);
  msLastMetric = millis();
  oled.clear(PAGE);     // Clear the screen
  oled.setFontType(1);  // Set font to type 0
  oled.setCursor(0, 0); // Set cursor to top-left
  oled.print("T=");          // Print "A0"
  oled.setFontType(1);         // 7-segment font
  oled.print(szInfo);

  float max471_c = analogRead(max471out);
  float vd_raw = analogRead(vdout);
  double voltage = (fmap(vd_raw,0.0,1023.0,0.0,3.7)  ); 
  
  //Voltage formula 3.22v(accross resistor network)/.768v (across 1K) = 4.19
  double current = (max471_c * 4.19) / 1024.0; //Amps detected
   
   
  //Thermistor Input  
  // Ref: https://learn.adafruit.com/thermistor/using-a-thermistor
  double thermistor_raw = analogRead(THERMISTORPIN);
  thermistor_raw = (thermistor_raw * 3.3)  / 4095.0;
  double thermistor_resistance = ( ( 3.3 * ( 10.0 / thermistor_raw ) ) - 10 ); // Resistance in kilo ohms
  thermistor_resistance = thermistor_resistance * 1000;
  //Convert resistance to Celsius with Steinhart Formula
  
  double therm_res_ln = log(thermistor_resistance);
  double temperature = ( 1 / ( 0.001129148 + ( 0.000234125 * therm_res_ln ) + ( 0.0000000876741 * therm_res_ln * therm_res_ln * therm_res_ln ) ) ); /* Temperature in Kelvin */
  temperature = temperature - 273.15; /* Temperature in degree Celsius */                  // convert to C
  float steinhart = temperature;
  
  //Particle.publish("Thermistor1" , String(steinhart));
  //Particle.publish("Thermistor1" , String(thermistor_raw));
  //Thermistor2 Input  
  // Ref: https://learn.adafruit.com/thermistor/using-a-thermistor
  double thermistor2_raw = analogRead(THERMISTORPIN2);
  thermistor2_raw = (thermistor2_raw * 3.3)  / 4095.0;
  double thermistor2_resistance = ( ( 3.3 * ( 10.0 / thermistor2_raw ) ) - 10 ); // Resistance in kilo ohms
  thermistor2_resistance = thermistor2_resistance * 1000;
  //Convert resistance to Celsius with Steinhart Formula
  
  double therm2_res_ln = log(thermistor2_resistance);
  double temperature2 = ( 1 / ( 0.001129148 + ( 0.000234125 * therm2_res_ln ) + ( 0.0000000876741 * therm2_res_ln * therm2_res_ln * therm2_res_ln ) ) ); /* Temperature in Kelvin */
  temperature2 = temperature2 - 273.15; /* Temperature in degree Celsius */                  // convert to C
  float steinhart2 = temperature2;
  
  //Particle.publish("Thermistor2" , String(steinhart2));
  //Particle.publish("Thermistor2" , String(thermistor2_raw));
  
  //JSON string creation
  root["BuckPower"] = relaystate;
  root["CurrentDraw"] = current;
  root["Voltage"] = voltage;
  root["DigitalTemp"] = celsius;
  root["Thermister1"] = temperature;
  root["Thermister2"] = temperature2;
  
  
  String jsonout;
  root.printTo(jsonout);
  if (millis() - msLast_Data_Sample >= msDATA_PUBLISH){
    //Serial.println("Publishing now.");
    Particle.publish("Data", jsonout);
    ThingSpeak.setField(1,root.get<int>("BuckPower"));
    ThingSpeak.setField(2,root.get<float>("CurrentDraw"));
    ThingSpeak.setField(3,root.get<float>("Voltage"));
    ThingSpeak.setField(4,root.get<float>("DigitalTemp"));
    ThingSpeak.setField(5,root.get<float>("Thermister1"));
    ThingSpeak.setField(6,root.get<float>("Thermister2"));
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);  
    
    //Particle.publish("ThingSpeak", jsonout , 60 , PRIVATE);
    msLast_Data_Sample = millis();
  }
 
  
  
  //Particle.publish("Data" , jsonout);
  
  
  
  oled.setCursor(0, 16);
  oled.print("I=");          // Print "A0"
  oled.setFontType(1);         // 7-segment font
  oled.print(current);
  oled.setCursor(0, 32);
  oled.print("V=");          // Print "A0"
  oled.setFontType(1);         // 7-segment font
  oled.print(voltage);
  oled.display();
  delay(10);
  
  
}

//THIS FUNCTION WILL MAP THE float VALUES IN THE GIVEN RANGE
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void getTemp(){
  float _temp;
  int   i = 0;

  do {
    _temp = ds18b20.getTemperature();
  } while (!ds18b20.crcCheck() && MAXRETRY > i++);

  if (i < MAXRETRY) {
    celsius = _temp;
    fahrenheit = ds18b20.convertToFahrenheit(_temp);
    Serial.println(celsius);
  }
  else {
    celsius = fahrenheit = NAN;
    Serial.println("Invalid reading");
  }
  msLastSample = millis();
}

int buckconverter(String power){
    //int state;
    if (power == "on"){
        digitalWrite(RELAY1, HIGH);
        Particle.publish("Buck Converter" , "On");
        relaystate = 1;
    }
    else{
        digitalWrite(RELAY1, LOW);
        Particle.publish("Buck Converter" , "OFF");
        relaystate = 0;
    }
    return relaystate;
}
  
  
