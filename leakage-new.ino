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

#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Bounce2.h>

// Necessary for the Water Leakage Sensor
#define NODE_ID 18 // or set to AUTO if you want gw to assign a NODE_ID for you.


// Necessary for the Dallas Temperature Sensor.

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

// General Information
#define SKETCH_NAME "Water Leakage and Central Heating Monitor"
#define SKETCH_VERSION "1.0"


MySensor gw;

// Necessary for the Leakage sensor
 boolean metric = false;

//Set up debouncer (used for door sensors)
Bounce debouncer = {Bounce()};

//Make sure to match the order of doorPins to doorChildren.
//The pins on your Arduino
int doorPin = 5;
//The child ID that will be sent to your controller
int doorChild = 1;

//used to keep track of previous values contact sensor values
uint8_t oldValueContact = 1;

// Initialize door message
MyMessage doorMsg(0, V_TRIPPED);


// Necessary for Dallas sensors

float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
boolean receivedConfig = false;
//boolean metric = true; 

// Initialize temperature message
MyMessage msg(0,V_TEMP);


void setup()  
{ 
  // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);

  // Startup and initialize MySensors library. Set callback for incoming messages. 
  // gw.begin();
  //gw.begin(NULL, NODE_ID);
  gw.begin(NULL, NODE_ID, true);
 //gw.begin(NULL, AUTO, true);
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);

//Set up door contacts
  
  // Setup the pins
  pinMode(doorPin, INPUT);

  // Activate internal pull-up
  digitalWrite(doorPin, HIGH);

  // After setting up the button, setup debouncer
  debouncer.attach(doorPin);
  debouncer.interval(500); //This is set fairly high because when my door was shut hard it caused the other door to bounce slightly and trigger open.
  
  // Present door sensors to controller
  gw.present(doorChild, S_DOOR);

  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();

  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     gw.present(i, S_TEMP);
  }
}


void loop()     
{     
  // Process incoming messages (like config from server)
  gw.process(); 

  unsigned long currentMillis = millis();
  debouncer.update();
// Get the update value
    uint8_t value = debouncer.read();
    if (value != oldValueContact) {
      // Send in the new value
      gw.send(doorMsg.setSensor(doorChild).set(value == HIGH ? "0" : "1"));
      oldValueContact = value;
    }

  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  gw.wait(conversionTime);
  

  // Read temperatures and send them to controller 
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
 
    // Only send data if temperature has changed and no error
    #if COMPARE_TEMP == 1
    if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
    #else
    if (temperature != -127.00 && temperature != 85.00) {
    #endif
 
      // Send in the new temperature
      gw.send(msg.setSensor(i).set(temperature,1));
      // Save new temperatures for next compare
      lastTemperature[i]=temperature;
    }
  }
  gw.wait(SLEEP_TIME);
}
