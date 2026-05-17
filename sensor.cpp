#include <M5CoreS3.h>

// #include <Arduino.h>
#include "step_data.h"

// QueueHandle_t imuQueue;
QueueHandle_t stepQueue;
QueueHandle_t showQueue;

extern void webSocketTask(void *);
extern void sensorTask(void *);
extern void displayTask(void *);

void sensor_setup()
{
    // CoreS3.Imu.begin();              // Init IMU.
    auto cfg = M5.config();
    CoreS3.begin(cfg);

    stepQueue = xQueueCreate(20, sizeof(Step_t));
    Serial.println("Step queue created.");
   
    showQueue = xQueueCreate(5, sizeof(Step_t));
}

void task_start()
{
  xTaskCreatePinnedToCore(sensorTask, "IMU", 8192, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(webSocketTask, "WS", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "GR", 8192, NULL, 1, NULL, 1);
  Serial.println("Tasks started.");
}
