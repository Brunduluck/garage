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
#include "step_data.h"

WebSocketsServer webSocket = WebSocketsServer(81);

extern void display_show();

#define USE_SERIAL Serial

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	USE_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			USE_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		USE_SERIAL.printf("%02X ", *src);
		src++;
	}
	USE_SERIAL.printf("\n");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
    switch(type) {
        case WStype_DISCONNECTED:
            USE_SERIAL.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        		// send message to client
		        webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            USE_SERIAL.printf("[%u] get Text: %s\n", num, payload);

            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
            USE_SERIAL.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);

            // send message to client
            // webSocket.sendBIN(num, payload, length);
            break;
	case WStype_ERROR:			
	case WStype_FRAGMENT_TEXT_START:
	case WStype_FRAGMENT_BIN_START:
	case WStype_FRAGMENT:
	case WStype_FRAGMENT_FIN:
	    break;
    }

}

void wifi_setup()
{;
    // USE_SERIAL.begin(921600);
    USE_SERIAL.begin(115200);

    // Serial.setDebugOutput(true);
    USE_SERIAL.println();
    USE_SERIAL.flush();

    WiFi.mode(WIFI_STA); // Set to Station mode
    WiFi.begin("point-2", "Password1");

    while(WiFi.status() != WL_CONNECTED) {
        vTaskDelay(100);
    }

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // Prints assigned IP
 
}

extern QueueHandle_t stepQueue;

void webSocketTask(void *pvParameters)
{
  // QMI8658_Data data;
  Step_t step;

  while(1)
  {
    webSocket.loop(); // Handle heartbeats/handshakes
    
    // Wait indefinitely for data from the sensor task
    // Serial.print("Checking Step queue. ");
    // Serial.println( uxQueueSpacesAvailable(stepQueue) );
    if (xQueueReceive(stepQueue, &step, portMAX_DELAY))
    {
        // SEND JSON RESPONSE
        JsonDocument response;
        // JsonArray sensors = response.createNestedArray("sinus"); 
        // sensors.add(Px);
        // sensors.add(Py);
        response["dt"] = step.dt_sec;
        response["Px"] = step.Px;
        response["Py"] = step.Py;
        response["Vx"] = step.Vx;
        response["Vy"] = step.Vy;
        response["Ax"] = step.Ax;
        response["Ay"] = step.Ay;
        response["N"] = step.N;
        response["th"] = step.th;
        response["dWz"] = step.dWz;
        response["angleZ"] = step.angleZ;
        String output;
        serializeJson(response, output);
        // webSocket.broadcastTXT(output);

        if( ((int)(step.N)%10) == 0 )
        {
          webSocket.broadcastTXT(output);
          // Serial.println(output);
        }

    }
  }
}
