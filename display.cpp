#include <M5GFX.h>
M5GFX display;

//#include <M5UnitOLED.h>
//M5UnitOLED display; // default setting
//M5UnitOLED display ( 21, 22, 400000 ); // SDA, SCL, FREQ

//#include <M5UnitLCD.h>
//M5UnitLCD display;  // default setting
//M5UnitLCD display  ( 21, 22, 400000 ); // SDA, SCL, FREQ

//#include <M5UnitGLASS2.h>
//M5UnitGLASS2 display;  // default setting
//M5UnitGLASS2 display ( 21, 22, 400000 ); // SDA, SCL, FREQ

// #include <M5AtomDisplay.h>
// M5AtomDisplay display;

#include <M5Unified.h>
M5Canvas canvas(&display); // 1. Create a canvas linked to the display

#include "step_data.h"

void display_setup(void)
{
  display.init();
  display.fillScreen(TFT_BLACK);

    canvas.createSprite(200, 200);
}

extern double angleX, angleY, angleZ;
extern QueueHandle_t showQueue;

const int bufferSize = 128;
double bufX[bufferSize];
double bufY[bufferSize];
int indexP = 0;
float Xlast = 0, Ylast = 0;
double lastT = 0;

void pushP(float x, float y)
{
  bufX[indexP] = 10 * x;
  bufY[indexP] = 10 * y;
  indexP = (indexP + 1) % bufferSize; // Wraps back to 0 when it hits the end 
}

// float x0 = 100, y0 = 100;

void line(int x1, int y1, int x2, int y2, float th, int clr)
{
            float S = sin(th); 
            float C = cos(th);
            int X1 = 100 + x1 * C + y1 * S;
            int Y1 = 100 - x1 * S + y1 * C;
            int X2 = 100 + x2 * C + y2 * S;
            int Y2 = 100 - x2 * S + y2 * C;
            canvas.drawLine(X1, Y1, X2, Y2, clr); 
}

void displayTask(void *pvParameters)
{
    Step_t step;
    uint32_t lastTime = 0;
    float fps = 0;

    int old_x2 = 0, old_y2 = 0;
    for(int j=0; true; j++)
    {       
          if (xQueueReceive(showQueue, &step, portMAX_DELAY))
          {
            // DISCARD all but the newest message to kill the "Permanent Delay"
            Step_t latestStep = step;
            while (xQueueReceive(showQueue, &latestStep, 0)) {
              // Keep looping until the queue is empty, grabbing the newest one
            }
    
            // 0. Calculate FPS
            uint32_t now = millis();
            if (now > lastTime) {
                // Smooth the FPS value slightly so it's readable
                fps = (fps * 0.9f) + ((1000.0f / (now - lastTime)) * 0.1f);
            }
            lastTime = now;
            
            // 1. Work on the RAM buffer (No need for startWrite here)
            // *** canvas.setColorDepth(8); 
            canvas.fillSprite(TFT_BLACK); 

            // canvas.setTextColor(TFT_YELLOW);
            // canvas.printf("FPS: %.1f", fps);

            float z = step.angleZ; // Assuming this is in Radians
            // float x0 = 160, y0 = 120, r = 100;
            // Using 110, 110 as center for a 220x220 canvas
            float x0 = 100, y0 = 100;
            float r = 99;
 
            // Calculations

            // canvas.drawLine(x0, y0, 2*x2-old_x2, 2*y2-old_y2, TFT_GRAY);
            line(0, 0, 0, r, -step.angleZ, TFT_GRAY);
            line(-r, 0, r, 0, -step.angleZ, TFT_GRAY);
            line(0, 0, 0, -r, -step.angleZ, TFT_GREEN);

            canvas.fillCircle(x0, y0, 3, TFT_WHITE);
 
            int o = 1.0 * r;
            canvas.drawCircle(x0, y0, o, TFT_GRAY);
            canvas.drawCircle(x0, y0, o/2, TFT_GRAY);

            double kk = 10.0;

            // canvas.drawLine(x0+kk*step.Bx, y0-r, x0+kk*step.Bx, y0+r, TFT_BLUE);
            // canvas.drawLine(x0-r, y0-kk*step.By, x0+r, y0-kk*step.By, TFT_BLUE);
  
            canvas.drawLine(x0+kk*step.Rx, y0-r, x0+kk*step.Rx, y0+r, TFT_RED);
            canvas.drawLine(x0-r, y0-kk*step.Ry, x0+r, y0-kk*step.Ry, TFT_RED);
            
            kk = 100;
            canvas.drawLine(x0+kk*step.Vx, y0-r, x0+kk*step.Vx, y0+r, TFT_BLUE);
            canvas.drawLine(x0-r, y0-kk*step.Vy, x0+r, y0-kk*step.Vy, TFT_BLUE);

            pushP(step.Px, step.Py);
            float mPx = 0, mPy = 0;
            for(int i=0; i < bufferSize; i++)
            {
              mPx = mPx + bufX[i]; 
              mPy = mPy + bufY[i]; 
            }
            float Ox = x0 - mPx / bufferSize; 
            float Oy = y0 - mPy / bufferSize; 
            for(int i=1; i < bufferSize; i++)
            {
              int j0 = (indexP + bufferSize + i - 1) % bufferSize;
              int j1 = (indexP + bufferSize + i) % bufferSize;
              line(30*bufX[j0], 30*bufY[j0], 30*bufX[j1], 30*bufY[j1], -step.angleZ, canvas.color565(2*i+3, 2*i+3, 0) );
              if((j%100) == 1)
              {
                Serial.print("j0 = "); Serial.print(j0); Serial.print(", x = "); Serial.print(30*bufX[j], 6); 
                Serial.print(", clr ="); Serial.println( canvas.color565(4*i+3, 4*i+3, 0) );
              }
            }

            if((j%100) == 1)
            {
              Serial.print("Ux = "); Serial.print(step.Ux,6); Serial.print(", Uy = "); Serial.print(step.Uy,6);
              Serial.print(", Px = "); Serial.print(100*bufX[indexP-1], 6); Serial.print(", Py = "); Serial.println(100*bufY[indexP-1], 6);
            }
            
            // 2. Push finished frame to physical hardware
            canvas.pushSprite(60, 20); // Push to center of CoreS3 screen
            
            // 3. Let the hardware finish
            // vTaskDelay(pdMS_TO_TICKS(10));
            display.waitDisplay(); 

            // Debugging
            // if( (j%10) == 0 ) Serial.printf("AngleZ: %.2f\n", z * 180.0 / M_PI);
        }
        if( (j%1000) == 0 ) Serial.println( fps ); 
        
        // vTaskDelay(pdMS_TO_TICKS(10)); // Don't forget to yield!        
    }
}
