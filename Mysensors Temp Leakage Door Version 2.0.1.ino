/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 */

// Enable debug prints to serial monitor
#define MY_DEBUG
boolean debug = false;                            // Used for additional debugging lines in serial console;

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_DEFAULT_RX_LED_PIN  6                        // Receive led pin  GREEN LED
#define MY_DEFAULT_TX_LED_PIN  5                        // the PCB, on board YELLOW LED

#include <SPI.h>
#include <MySensors.h>  
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Bounce2.h>

// Necessary for the Dallas Temperature Sensor.

#define COMPARE_TEMP 1                                // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 3                                // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
#define NUMBER_OF_SWITCHES 2                          // Number of door switches
unsigned long sleepTimer = 60000;
unsigned long SLEEP_TIME = 60000;                     // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS);                        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);                  // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
boolean receivedConfig = false;
boolean metric = true; 

// Necessary for the Water Leakage Sensor / Temp and door Sensor
// #define NODE_ID 18                                     // or set to AUTO if you want gw to assign a NODE_ID for you.
#define CHILD_ID_DOOR AUTO                                // or set to AUTO if you want gw to assign a NODE_ID for you.

Bounce debouncer[NUMBER_OF_SWITCHES];                   // Set up debouncer (used for door sensors)
uint8_t oldValueContact[NUMBER_OF_SWITCHES];            //used to keep track of previous values contact sensor values

byte switchPin[NUMBER_OF_SWITCHES] = {7,8};             // what digital pin we're connected to Digital Pin 7 Door
                                                        // what digital pin we're connected to Digital Pin 8 Leakage

// Initialize door message, Initialize temperature message
MyMessage msgTemp(0,V_TEMP);
MyMessage msgDoor(CHILD_ID_DOOR, V_TRIPPED);

void before()
{
  // Startup up the OneWire library
  sensors.begin();

    for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    Bounce debouncer[i] = Bounce();
  }
}

void setup() { 
  sensors.setWaitForConversion(false);      // requestTemperatures() will not block current thread
   
  //Set up door contacts
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    pinMode(switchPin[i], INPUT);
    digitalWrite(switchPin[i], HIGH);
    oldValueContact[i] = 1;
//    Bounce debouncer[i] = Bounce();
    debouncer[i].attach(switchPin[i]);
    debouncer[i].interval(5);
  }
}


void presentation() {
  
  sendSketchInfo("CH Temp, Leakage, Door", "1.1");  // Send the sketch version information to the gateway and Controller

  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();
    
  // Present all temp sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     present(i, S_TEMP);
  }

  // Present all switches to controller
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    present(i, S_DOOR);
    delay(250);
  }
}



void loop()     
{     

  // check for switch updates
  for (int i = 0; i < NUMBER_OF_SWITCHES; i++)
  {
    debouncer[i].update();
    uint8_t ValueContact = debouncer[i].read();
    if (ValueContact != oldValueContact[i]) 
    {
      //send(msgDoor.setSensor(i).set(ValueContact == HIGH? true : false), false); 
      
      if (i==0) {
         send(msgDoor.setSensor(i).set(ValueContact==HIGH ? 0 : 1));  
      }

      if (i==1) {
         send(msgDoor.setSensor(i).set(ValueContact==HIGH ? 1 : 0));  
      }
      
      if (debug) {
         Serial.print("Sending doorvalue to controller: ");
         Serial.println(ValueContact);
       } 

       
    }
    oldValueContact[i] = ValueContact;
  }
  
  if (sleepTimer == SLEEP_TIME)
  {
      // handling of Temperature Sensors  
      sensors.requestTemperatures();                    // Fetch temperatures from Dallas sensors

      // query conversion time and sleep until conversion completed
      //int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
      int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());

      if (debug) {
        Serial.print("sleepTimer: ");
        Serial.println(" %\t");
      }
        
      // Read temperatures and send them to controller 
      for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
          // Fetch and round temperature to one decimal
          float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
    
          // Only send data if temperature has changed and no error
          #if COMPARE_TEMP == 1
          if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
          #else
              if (temperature != -127.00 && temperature != 85.00) {
              #endif 
              // Send in the new temperature
              send(msgTemp.setSensor(i).set(temperature,1));
              // Save new temperatures for next compare
              lastTemperature[i]=temperature;
            }
            sleepTimer = 0;  
          }
      }
      else sleepTimer++;
  
} // End of Loop


