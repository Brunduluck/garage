#include <M5CoreS3.h>

#include <QMI8658.h>
#include <WebSocketsServer.h>
#include "step_data.h"

double G = 9.80665;
int N200 = 500;

double accX_offset = 0, accY_offset = 0, accZ_offset = 0;
double angleX_offset = 0, angleY_offset = 0, angleZ_offset = 0;

extern  QueueHandle_t stepQueue;
extern  QueueHandle_t showQueue;

QMI8658_Data data0;
QMI8658_Data data1;

Kalman kalmanRoll, kalmanPitch;
KalmanKinematic velX, velY;
 
void sensorTask(void * dummy)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Define 10ms period based on the RTOS tick rate
  const TickType_t xFrequency = pdMS_TO_TICKS(10); 

  double time0 = 0.0, time1 = 0.0;
  double angleX = 0, angleY = 0, angleZ = 0;
  double Ax = 0, Ay = 0;
  double WorldVx = 0, WorldVy = 0;
  double Px = 0, Py = 0;
  double Ux = 0, Uy = 0;
  double smoothAx = 0, smoothAy = 0;
  double smoothKx = 0, smoothKy = 0;
  double vT = 0;

  Serial.println("Testing Accelerometer");
  vTaskDelay(10);

  bool ok = false;
  for (int i = 0; true; i++)
  {
      Step_t step;

      // webSocket.loop();

      // byte good = mpu.getEvent(&a1, &g1, &temp1);

      auto imu_update = M5.Imu.update();
      if(imu_update == 0) continue;
      auto data = M5.Imu.getImuData();

        // The data obtained by getImuData can be used as follows.
        data1.accelX = (double)data.accel.x * G - accX_offset;      // accel x-axis value.
        data1.accelY = (double)data.accel.y * G - accY_offset;      // accel y-axis value.
        data1.accelZ = (double)data.accel.z * G - accZ_offset;      // - G; // accel z-axis value.

        data1.gyroX = (double)data.gyro.x / 180.0 * M_PI - angleX_offset;      // gyro x-axis value.
        data1.gyroY = (double)data.gyro.y / 180.0 * M_PI - angleY_offset;      // gyro y-axis value.
        data1.gyroZ = (double)data.gyro.z / 180.0 * M_PI - angleZ_offset;      // gyro z-axis value.

        // data.mag.x;      // mag x-axis value.
        // data.mag.y;      // mag y-axis value.
        // data.mag.z;      // mag z-axis value.

        ok = imu_update;

        time1 = micros();

        if( ok && (time0 != 0.0) )
        { 

            double dt_sec = (time1 - time0) / 1000000.0; 
            vT = vT + dt_sec;

            // 1. Gyro Calculations (Trapezoidal Rule)
            double Wz = (data0.gyroZ + data1.gyroZ) / 2.0;
            double dWz = Wz * dt_sec;
            angleZ += dWz; 
            if(angleZ < 0.0) angleZ += 2.0 * M_PI;
            if(angleZ > 2.0 * M_PI) angleZ -= 2.0 * M_PI;

            // 2. Cross-Axis Gyro Integrators 
            double dWx = (data0.gyroX + data1.gyroX) / 2.0 * dt_sec;
            double dWy = (data0.gyroY + data1.gyroY) / 2.0 * dt_sec;
            angleX = angleX + dWx;
            angleY = angleY + dWy;

            // 3. Raw Accelerometer Filter Buffer
            smoothAx = (0.25f * data1.accelX) + (0.75f * smoothAx);
            smoothAy = (0.25f * data1.accelY) + (0.75f * smoothAy);

            // ==========================================
            // 4. BLUE LINES (Manual Integration - Matrix Realigned)
            // ==========================================
            // Swapped: angleY now corrects X-accel, angleX now corrects Y-accel
            // Signs changed to '+' to match the raw hardware outputs
            Ax = smoothAx + G * sin(angleY); 
            Ay = smoothAy - G * sin(angleX); 
            step.Bx = Ax; 
            step.By = Ay;

            // ==========================================
            // 5. Get Orientation (Kalman Filter Alignment)
            // ==========================================
            // Decoupled denominators preserve separate tracking loops
            float accRoll  = atan2(data1.accelY, data1.accelZ);
            float accPitch = atan2(-data1.accelX, data1.accelZ);

            // Gyros match their coupled accelerometer axis partners 
            float pitch = kalmanPitch.getAngle(accPitch, data1.gyroY, dt_sec); // gyroY drives Pitch
            float roll  = kalmanRoll.getAngle(accRoll, data1.gyroX, dt_sec);   // gyroX drives Roll

            // ==========================================
            // 6. Remove Gravity Component (RED LINES)
            // ==========================================
            // Realigned signs to mirror the balanced blue tracking equations
            float linAccelX = data1.accelX + G * sin(pitch);
            float linAccelY = data1.accelY - G * sin(roll);

            // Final isolation step passes clean trends to your canvas drawer
            smoothKx = (0.25f * linAccelX) + (0.75f * smoothKx);
            smoothKy = (0.25f * linAccelY) + (0.75f * smoothKy);
\
            // the result
            step.Rx = smoothKx;
            step.Ry = smoothKy;
                      
            // 7. Estimate Velocity (m/s if accel is in Gs, multiply by 9.81)
            float velocityX = velX.update(linAccelX, dt_sec);
            float velocityY = velY.update(linAccelY, dt_sec);

            // 8. Local velocities
            double dUx = smoothKx * dt_sec;
            double dUy = smoothKy * dt_sec;
            Ux = Ux + dUx;
            Uy = Uy + dUy;

            // 9. World velocities
            WorldVx = velocityX * cos(angleZ) - velocityY * sin(angleZ);
            WorldVy = velocityX * sin(angleZ) + velocityY * cos(angleZ);

            // 10. World position shift
            // Calculate absolute physical speed (magnitude of your world velocity)
            float physical_speed = sqrtf(WorldVx * WorldVx + WorldVy * WorldVy);

            // 11. Prevent division-by-zero if the device is perfectly still
            if (physical_speed > 0.0001f)
            {      
                // Extract the direction unit vector
                float dirX = WorldVx / physical_speed;
                float dirY = WorldVy / physical_speed;

                // --- CHOOSE ONE COMPRESSION METHOD BELOW ---

                // CHOICE A: Hyperbolic Tangent Speed Cap (Hard maximum limit 'C')
                float C = 10.0f; // Maximum allowable display velocity in m/s
                float compressed_speed = C * tanhf(physical_speed / C);

                // CHOICE B: Logarithmic Compression (Smooth sub-linear scaling)
                // float K = 0.5f; // Tuning multiplier for display bounds
                // float compressed_speed = K * log1pf(physical_speed); 

                // --------------------------------------------

                // 3. Compute the compressed displacement step
                float stepX = compressed_speed * dt_sec * dirX;
                float stepY = compressed_speed * dt_sec * dirY;

                // 4. Update display positions
                Px = Px + stepX;
                Py = Py + stepY;

            } else {
                // If speed is zero, positions do not change
                Px = Px; 
                Py = Py;
            }

            // 12. Apply your leaky integrator or bounding box limits
            Px = 0.99f * Px;
            Py = 0.99f * Py;

            step.Px = Px;
            step.Py = Py;
            
            // radius
            double k = sqrt( (Ux*Ux + Uy*Uy) / (WorldVx*WorldVx + WorldVy*WorldVy) );
            WorldVx *= k;
            WorldVy *= k;
            double R = Ux*Ux / Ay;

            step.th = angleZ / M_PI * 180.0;
            step.dt_sec = dt_sec;
            step.vT = vT;
            step.Wz = Wz;
            step.dWz = dWz;
            step.angleZ = angleZ; 
            step.Ax = smoothKx;
            step.Ay = smoothKy;
            step.Ux = Ux;
            step.Uy = Uy;
            step.Vx = velocityX;
            step.Vy = velocityY;
            step.Px = Px;
            step.Py = Py;
            step.N = i;
          
            // to put into the queue
            // Send to WebSocket Queue
            UBaseType_t waiting = uxQueueMessagesWaiting(showQueue);
            // Serial.printf("Items in queue: %d\n", waiting);
            xQueueSend(showQueue, &step, 0);
            xQueueSend(stepQueue, &step, 0);

            // if((i%100) == 0 )
            //     { Serial.print("data ");  Serial.print( i );  Serial.println(" placed."); }
            // Serial.println( uxQueueSpacesAvailable(stepQueue) );

            if((i%100) == 1 )
            {
              // Serial.println();
              Serial.print( "dt_sec = " );
              Serial.print( dt_sec * 1000, 6 );
              Serial.print( "\t" );
              Serial.print( angleZ * 180 / M_PI );
              Serial.print( "\t" );
              Serial.print( angleY * 180 / M_PI );
              Serial.print( "\t" );
              Serial.print( angleX * 180 / M_PI );
              Serial.print( ", Vx = " );
              Serial.print( 100 * step.Vx, 6 );
              Serial.print( ", Vy = " );
              Serial.print( 100 * step.Vy, 6);
              Serial.print( ", Ay = " );
              Serial.print( step.Ay, 6);
              Serial.print( ", data.accel.x = " );
              Serial.print( data.accel.x * G, 6);
              Serial.print( ", data.accel.y = " );
              Serial.print( data.accel.y * G, 6);
              Serial.print(", G * sin( angleX ) = ");
              Serial.print(G * sin( angleX ), 6);
              Serial.print(", G * sin( angleY ) = ");
              Serial.print(G * sin( angleY ), 6);
              Serial.print( ", Px = " );
              Serial.print( Px, 6 );
              Serial.print( ", Py = " );
              Serial.print( Py, 6 );
              Serial.print( ", R = " );
              Serial.println( R, 6 );
            }
        }

        data0.accelX = data1.accelX;
        data0.accelY = data1.accelY;
        data0.accelZ = data1.accelZ;
        
        data0.gyroX = data1.gyroX;
        data0.gyroY = data1.gyroY;
        data0.gyroZ = data1.gyroZ;
        
        time0 = time1;

        vTaskDelay(10);   
   }

}

void sensor_calibrate()
{
  double sumX = 0, sumY = 0, sumZ = 0;
  double sumWx = 0, sumWy = 0, sumWz = 0;
  double sumX2 = 0, sumY2 = 0, sumZ2 = 0;
  double sumWx2 = 0, sumWy2 = 0, sumWz2 = 0;
  double time0 = 0.0, time1 = 0.0;
  double Sdt = 0, Sdt2 = 0;
  int n = 0;

  Serial.println("Calibrating Accelerometer");
  vTaskDelay(200);

  bool ok = false;
  for (int i = 0; i < N200+1; i++)
  {
    {
      // byte good = mpu.getEvent(&a1, &g1, &temp1);

      auto imu_update = M5.Imu.update();
      if(imu_update == 0) Serial.println("Data Not Ready");
      auto data = M5.Imu.getImuData();

        // The data obtained by getImuData can be used as follows.
        data1.accelX = (double)data.accel.x * G;      // accel x-axis value.
        data1.accelY = (double)data.accel.y * G;      // accel y-axis value.
        data1.accelZ = (double)data.accel.z * G - G;  // accel z-axis value.

        data1.gyroX = (double)data.gyro.x / 180.0 * M_PI ;      // gyro x-axis value.
        data1.gyroY = (double)data.gyro.y / 180.0 * M_PI ;      // gyro y-axis value.
        data1.gyroZ = (double)data.gyro.z / 180.0 * M_PI ;      // gyro z-axis value.
 
        // data.mag.x;      // mag x-axis value.
        // data.mag.y;      // mag y-axis value.
        // data.mag.z;      // mag z-axis value.

      if(i == 10)
      {
            Serial.print("data.accel.z = ");
            Serial.println(data.accel.z);
      }
      
      ok = (imu_update != 0);
      // Serial.println(data1.gyroZ);
    }

    // data1.accelZ += G;

    time1 = micros();

    if( ok && (time0 != 0.0) )
    {
      double dt_sec = (time1 - time0) / 1000000.0;
      Sdt = Sdt + dt_sec;
      Sdt2 = Sdt2 + dt_sec*dt_sec;
      n = n + 1;

      sumX += data1.accelX;
      sumY += data1.accelY;
      sumZ += (data1.accelZ); // + G);

      sumX2 += data1.accelX*data1.accelX;
      sumY2 += data1.accelY*data1.accelY;
      sumZ2 += (data1.accelZ)*(data1.accelZ);

      sumWx += data1.gyroX;
      sumWy += data1.gyroY;
      sumWz += data1.gyroZ;

      sumWx2 += data1.gyroX*data1.gyroX;
      sumWy2 += data1.gyroY*data1.gyroY;
      sumWz2 += data1.gyroZ*data1.gyroZ;

      vTaskDelay(10);
    }

    time0 = time1;

  }

  accX_offset = sumX / n;
  accY_offset = sumY / n;
  accZ_offset = sumZ / n; //  + G;

  angleX_offset = sumWx / n;
  angleY_offset = sumWy / n;
  angleZ_offset = sumWz / n;

  time0 = 0;

  Serial.println("Accelerometer Calibration Done:");
  double VarianceT = ( Sdt2 - Sdt*Sdt / n ) / n ;
  VarianceT = sqrt( VarianceT );
  Serial.print("Dt = "); Serial.print( Sdt / n * 1000, 8);
  Serial.print(" ms, Deviation = "); Serial.println( VarianceT, 6 );

  double VarianceX = ( sumX2 - sumX*sumX / n ) / n ;
  VarianceX = sqrt( VarianceX );
  Serial.print("X Offset = "); Serial.print(accX_offset, 8);
  Serial.print(" +/- "); Serial.println( VarianceX, 6 );

  double VarianceY = ( sumY2 - sumY*sumY / n ) / n ;
  VarianceY = sqrt( VarianceY );
  Serial.print("Y Offset = "); Serial.print(accY_offset, 8);
  Serial.print(" +/- "); Serial.println( VarianceY, 6 );

  double VarianceZ = ( sumZ2 - sumZ*sumZ / n ) / n ;
  VarianceZ = sqrt( VarianceZ );
  Serial.print("Z Offset = "); Serial.print(accZ_offset, 8);
  Serial.print(" +/- "); Serial.println( VarianceZ, 6 );

  double VarianceWx = ( sumWx2 - sumWx*sumWx / n ) / n ;
  VarianceWx = sqrt( VarianceWx );
  Serial.print("WX Offset = "); Serial.print(angleX_offset, 8);  
  Serial.print(" +/- "); Serial.println( VarianceWx, 6 );

  double VarianceWy = ( sumWy2 - sumWy*sumWy / n ) / n ;
  VarianceWy = sqrt( VarianceWy );
  Serial.print("WY Offset = "); Serial.print(angleY_offset, 8);
  Serial.print(" +/- "); Serial.println( VarianceWy, 6 );

  double VarianceWz = ( sumWz2 - sumWz*sumWz / n ) / n ;
  VarianceWz = sqrt( VarianceWz );
  Serial.print("WZ Offset = "); Serial.print(angleZ_offset, 8);
  Serial.print(" +/- "); Serial.println( VarianceWz, 6 );

  Serial.print("n = "); Serial.println( n );
}
