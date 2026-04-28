/*
 * WebSocketServer.ino
 *
 *  Created on: 22.05.2015
 *
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

extern void sensor_setup();
extern void sensor_calibrate();
extern void wifi_setup();
extern void websocket_setup();

void setup()
{
    Serial.begin(115200);

    Serial.println("*** start ***");

    sensor_setup();
    sensor_calibrate();
    wifi_setup();
    websocket_setup(); 
}


void loop()
{
}
