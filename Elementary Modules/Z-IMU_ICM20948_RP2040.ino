#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define AD0_VAL 1     

ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets. NOW SET TO 2000DPS FSS
float Gscale = (M_PI / 180.0) * 0.06106870229; //250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {-36, -25, -9};
//float G_offset[3] = {74.3, 153.8, -5.5};

//Accel scale: divide by 16604.0 to normalize
float A_B[3]
 { -107.51, -655.68, -415.95};

float A_Ainv[3][3]
  {{  1.02754, -0.00198, -0.00962},
  { -0.00198,  1.00942,  0.00091},
  { -0.00962,  0.00091,  1.00630}};

//Mag scale divide by 369.4 to normalize

 float M_B[3]
 { -137.87,  -11.85,  762.52};

 float M_Ainv[3][3]
  {{  1.59427, -0.02776,  0.00298},
  { -0.02776,  1.64724, -0.03979},
  {  0.00298, -0.03979,  1.43937}};

// local magnetic declination in degrees
float declination = -11;

// These are the free parameters in the Mahony filter and fusion scheme,

float Kp = 0.25;
float Ki = 0.0;

//TIMING VARIABLES
unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds
#define PRINT_SPEED 50 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output


void setup()
{
  Serial.begin(230400);
  while (!Serial); //wait for connection
  Wire.begin();
  Wire.setClock(400000); //Set I2C clock speed. Fast enough to be impressive, not fast enough to break things.
  imu.begin(Wire, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }

 // Set full-scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; 
  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e), Choices are: gpm2, gpm4, gpm8, gpm16.  "gpm" = G's, +/-
  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e), Choices are: dps250, dps500, dps1000, dps2000.  "dps" = Degrees/sec
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  }

void loop()
{

  static int loop_counter = 0; //sample & update loop counter
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data


  // Update the sensor values whenever new data is available
  if ( imu.dataReady() ) {

    imu.getAGMT();

    loop_counter++;
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

    Mxyz[1] = -Mxyz[1]; //reflect Y and Z
    Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint > PRINT_SPEED) 
    {
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      yaw = -(yaw + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      int looptimer = 1/deltat;
      //     Serial.print("ypr ");
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.print(", "); 
      Serial.print(looptimer);  //sample & update loops per print interval
      loop_counter = 0;
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }
  }
}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
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

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}