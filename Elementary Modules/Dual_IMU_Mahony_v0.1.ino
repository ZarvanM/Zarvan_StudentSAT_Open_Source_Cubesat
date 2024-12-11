//
// Thanks to S.J. Remington for initial implementation of Mahony filters 

#include "Wire.h"

unsigned long loop_timer;
int MPU1_addr = 0x68, MPU2_addr = 0x68;
int cal_gyro = 1;  
int cal_count= 900;
float accelWeight = 0.002; // Adjust between 0.0 and 1.0 to control influence
  
  int16_t ax1, ay1, az1, ax2, ay2, az2;
  int16_t gx1, gy1, gz1, gx2, gy2, gz2;
  int16_t Tmp1, Tmp2; 


float A_cal1[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; 
float A_cal2[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; 
float G_off1[3] = {0, 0, 0}; 
float G_off2[3] = {0, 0, 0}; 

#define gscale ((2000./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s
#define IMU1Pin 29
// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//26.86, -0.99, -32.72

float q[4] = {1.0, 0.0, 0.0, 0.0};
float Kp = 30.0;
float Ki = 0.0;
unsigned long now_ms, last_ms = 0; //millis() timers
int wooftimer = 500;

unsigned long print_ms = 50; //print angles every "print_ms" milliseconds
float yaw, pitch, roll; //Euler angle output
 
  static unsigned int i = 0; //loop counter
  static float deltat,deltatms, cyclefrq = 0;  //loop time in seconds
  static unsigned long now = 0, last = 0; //micros() timers
  static long gsum1[3] = {0};
  static long gsum2[3] = {0};

  float Axyz[3];
  float Gxyz[3];

int looptimes[30];
int cycles = 0;




void setup() 
{
 // rp2040.wdt_begin(wooftimer);

pinMode(IMU1Pin, OUTPUT);
digitalWrite(IMU1Pin, LOW);
delay(1500);
digitalWrite(IMU1Pin, HIGH);
delay(1000);

Wire.setSDA(4);
Wire.setSCL(5);
Wire.begin();
Wire.setClock(100000);
Wire1.setSDA(2);
Wire1.setSCL(3);
Wire1.begin();
Wire1.setClock(100000);

delay(1000);
  
  Serial.begin(460800);  
    while (!Serial)
    {
    delay(1); // Avoids WEIRD serial garbage and bugginess. Remove SERIAL usage once done debugging. No one has USB in space...
    }
  
  Serial.println("starting");

IMU1Init();
IMU2Init();

delay(5000);
loop_timer = micros(); 
last = micros();
}

// AHRS loop

void loop()
{
  cycles++;


  IMU1Read();
  IMU2Read();

  IMUCalib();


Axyz[0] = ((float)ax1 + (float)ax2) / 2.0;
Axyz[1] = ((float)ay1 + (float)ay2) / 2.0;
Axyz[2] = ((float)az1 + (float)az2) / 2.0;


Gxyz[0] = (((float)gx1 - G_off1[0]) + ((float)gx2 - G_off2[0])) / 2.0 * gscale;
Gxyz[1] = (((float)gy1 - G_off1[1]) + ((float)gy2 - G_off2[1])) / 2.0 * gscale;
Gxyz[2] = (((float)gz1 - G_off1[2]) + ((float)gz2 - G_off2[2])) / 2.0 * gscale;

     if (cal_gyro && i < cal_count) 
    { 
        Gxyz[0]=0;
        Gxyz[1]=0;
        Gxyz[2]=0;
    }

    now      = micros();
    deltat   = (now - last) * 1.0e-6; //seconds since last update
    deltatms = deltat*1.0e6;
    cyclefrq = 1/deltat;
    last = now;

    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
    EulerPrep();
    IMU1Out();




  while(micros() - loop_timer < 4400)
  {
      //aux_work(); 
  }           
  
  rp2040.wdt_reset();   

  loop_timer = micros(); 
}

void IMUCalib()
{
  if (cal_gyro) 
  {
    i++;
    gsum1[0] += gx1; gsum1[1] += gy1; gsum1[2] += gz1;
    gsum2[0] += gx2; gsum2[1] += gy2; gsum2[2] += gz2;
    if (i == cal_count) 
      {
      cal_gyro = 0;  //turn off calibration and print results
      for (int k = 0; k < 3; k++) 
        {
        G_off1[k] = ((float) gsum1[k]) / (float)cal_count;
        G_off2[k] = ((float) gsum2[k]) / (float)cal_count;
        }
      }
  }
}










void IMU1Out()
{
      Serial.print(pitch);
      Serial.print(", ");
      Serial.print(yaw);
      Serial.print(", ");
      Serial.print(roll);
      Serial.print(", ");
      Serial.print(deltatms);
      Serial.print(", ");     
      Serial.print(cyclefrq);
      Serial.println(" ");

}
void IMU1Read()
{
  Wire.beginTransmission(MPU1_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU1_addr, 14); // request a total of 14 registers
  int t = Wire.read() << 8;
  ax1 = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  ay1 = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read() << 8;
  az1 = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read() << 8;
  Tmp1 = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  gx1 = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gy1 = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read() << 8;
  gz1 = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void IMU2Read()
{
  Wire1.beginTransmission(MPU2_addr);
  Wire1.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU2_addr, 14); // request a total of 14 registers
  int t = Wire1.read() << 8;
  ax2 = t | Wire1.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire1.read() << 8;
  ay2 = t | Wire1.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire1.read() << 8;
  az2 = t | Wire1.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire1.read() << 8;
  Tmp2 = t | Wire1.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire1.read() << 8;
  gx2 = t | Wire1.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire1.read() << 8;
  gy2 = t | Wire1.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire1.read() << 8;
  gz2 = t | Wire1.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void IMU1Init()
{
  Wire.beginTransmission(MPU1_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU1_addr);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);
}

void IMU2Init()
{
  Wire1.beginTransmission(MPU2_addr);
  Wire1.write(0x6B);  // PWR_MGMT_1 register
  Wire1.write(0);     // set to zero (wakes up the MPU-6050)
  Wire1.endTransmission(true);
  Wire1.beginTransmission(MPU2_addr);
  Wire1.write(0x1B);
  Wire1.write(0x18);
  Wire1.endTransmission(true);
}










void EulerPrep()
{
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    yaw   *= 180.0 / PI;
    if (yaw < 0) yaw += 360.0;
    
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;
}
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) 
{
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;

  // ignore accelerometer if false (tested OK, SJR)
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)

    ex = accelWeight * (ay * vz - az * vy);
    ey = accelWeight * (az * vx - ax * vz);
    ez = accelWeight * (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, given by gyro term
  // rate of change = current orientation quaternion (qmult) gyro rate

  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  //add qmult*delta_t to current orientation
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}
void CyclicCheck()
{
  if (cycles < 30) 
    {
    looptimes[cycles] = deltat*1.0e6; // Add current index as value
    }

    if (cycles>=30)
  {
      for (int z = 0; z < 30; z++) 
    {
    Serial.print(looptimes[z]); // Print the current element
    Serial.print(", ");
    }
      Serial.println(" ");
      delay(10000000);
  }
}