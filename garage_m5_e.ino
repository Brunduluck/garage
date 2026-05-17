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
extern void task_start();
extern void wifi_setup();
extern void display_setup();

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("*** start ***");

    display_setup();
    sensor_setup();
    sensor_calibrate();
    wifi_setup();
    task_start();
}


void loop()
{
    ;
}
