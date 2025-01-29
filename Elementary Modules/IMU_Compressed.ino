#include "ICM_20948.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

#define AD0_VAL 1
ICM_20948_I2C imu;
float Gscale = (M_PI / 180.0) * 0.06106870229;
float G_offset[3] = {-36, -25, -9};
float A_B[3] = {-107.51, -655.68, -415.95};
float A_Ainv[3][3] = {
  {1.02754, -0.00198, -0.00962},
  {-0.00198,  1.00942,  0.00091},
  {-0.00962,  0.00091,  1.00630}
};
float M_B[3] = {-137.87, -11.85, 762.52};
float M_Ainv[3][3] = {
  {1.59427, -0.02776,  0.00298},
  {-0.02776,  1.64724, -0.03979},
  {0.00298,  -0.03979,  1.43937}
};
float declination = -11;
float Kp = 0.25;

unsigned long nowTime = 0, lastTime = 0;
float deltat = 0;
#define PRINT_SPEED 50
unsigned long lastPrint = 0;
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll;

void setup() {
  Serial.begin(230400);
  while (!Serial);
  Wire.begin();
  Wire.setClock(400000);
  imu.begin(Wire, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println("ICM_90248 not detected");
    while (1);
  }
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm2;
  myFSS.g = dps2000;
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
}

void loop() {
  static int loop_counter = 0;
  static float Gxyz[3], Axyz[3], Mxyz[3];

  if (imu.dataReady()) {
    imu.getAGMT();
    loop_counter++;
    get_scaled_IMU(Gxyz, Axyz, Mxyz);
    Mxyz[1] = -Mxyz[1];
    Mxyz[2] = -Mxyz[2];
    nowTime = micros();
    deltat = (nowTime - lastTime) * 1.0e-6;
    lastTime = nowTime;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint > PRINT_SPEED) {
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5f - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0f * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5f - (q[2] * q[2] + q[3] * q[3]));
      yaw   *= 180.0f / PI;
      pitch *= 180.0f / PI;
      roll  *= 180.0f / PI;
      yaw    = -(yaw + declination);
      if (yaw < 0) yaw += 360.0f;
      if (yaw >= 360.0f) yaw -= 360.0f;
      int looptimer = 1 / deltat;
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.print(", ");
      Serial.println(looptimer);
      loop_counter = 0;
      lastPrint = millis();
    }
  }
}

void vector_normalize(float a[3]) {
  float m = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  a[0] /= m; a[1] /= m; a[2] /= m;
}

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  float temp[3];
  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);
  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  for (int i = 0; i < 3; i++) temp[i] = Axyz[i] - A_B[i];
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  for (int i = 0; i < 3; i++) temp[i] = Mxyz[i] - M_B[i];
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float d) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4;
  float q3q3 = q3 * q3, q3q4 = q3 * q4, q4q4 = q4 * q4;
  float hx = ay * mz - az * my;
  float hy = az * mx - ax * mz;
  float hz = ax * my - ay * mx;
  float norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return;
  norm = 1.0f / norm;
  hx *= norm; hy *= norm; hz *= norm;
  float ux = 2.0f * (q2q4 - q1q3);
  float uy = 2.0f * (q1q2 + q3q4);
  float uz = q1q1 - q2q2 - q3q3 + q4q4;
  float wx = 2.0f * (q2q3 + q1q4);
  float wy = q1q1 - q2q2 + q3q3 - q4q4;
  float wz = 2.0f * (q3q4 - q1q2);
  float ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  float ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  float ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
  gx += Kp * ex; gy += Kp * ey; gz += Kp * ez;
  gx *= 0.5f * d; gy *= 0.5f * d; gz *= 0.5f * d;
  float qa = q1, qb = q2, qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
