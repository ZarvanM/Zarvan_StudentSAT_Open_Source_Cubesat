#include "ICM_20948.h" 

// Configuration constants
#define AD0_VAL 1
#define PRINT_INTERVAL 50 // ms between angle prints
#define DECLINATION -11.0f // local magnetic declination in degrees
#define GYRO_SCALE ((M_PI / 180.0f) * 0.06106870229f) // For 2000dps setting

// Mahony filter parameters
#define KP 0.25f  // Proportional gain
#define KI 0.0f   // Integral gain (currently unused)

// Create ICM_20948_I2C object
ICM_20948_I2C imu;

// Calibration data - make const to save memory and improve performance
const float G_OFFSET[3] = {-36.0f, -25.0f, -9.0f};

const float A_B[3] = {-107.51f, -655.68f, -415.95f};
const float A_AINV[3][3] = {
  {1.02754f, -0.00198f, -0.00962f},
  {-0.00198f, 1.00942f, 0.00091f},
  {-0.00962f, 0.00091f, 1.00630f}
};

const float M_B[3] = {-137.87f, -11.85f, 762.52f};
const float M_AINV[3][3] = {
  {1.59427f, -0.02776f, 0.00298f},
  {-0.02776f, 1.64724f, -0.03979f},
  {0.00298f, -0.03979f, 1.43937f}
};

// Quaternion and Euler angles
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float yaw, pitch, roll;

// Timing variables
uint32_t now = 0, last_update = 0, last_print = 0;
float delta_t = 0.0f;

// Forward declarations
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
inline void vector_normalize(float v[3]);

void setup() {
  Serial.begin(230400);
  while (!Serial); // Wait for connection
  
  Wire.begin();
  Wire.setClock(400000); // Set I2C speed to 400kHz
  
  bool initialized = false;
  while (!initialized) {
    imu.begin(Wire, AD0_VAL);
    if (imu.status == ICM_20948_Stat_Ok) {
      initialized = true;
    } else {
      Serial.println(F("ICM-20948 not detected, retrying..."));
      delay(500);
    }
  }

  // Configure IMU settings
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm2;     // +/-2g accelerometer full scale
  myFSS.g = dps2000;  // +/-2000 degrees/second gyro full scale
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  
  // Set more optimal settings
  imu.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, ICM_20948_Sample_Mode_Continuous);
  
  // Initialize timestamps
  last_update = last_print = micros();
  
  Serial.println(F("IMU initialized"));
}

void loop() {
  // Update sensor values when new data is available
  if (imu.dataReady()) {
    imu.getAGMT(); // Read all sensor data
    
    // Calculate delta time (handle micros overflow)
    now = micros();
    delta_t = (now - last_update) * 1.0e-6f; // Convert to seconds
    last_update = now;
    
    // Skip unreasonable delta_t values (e.g., after long blocking operations)
    if (delta_t > 0.05f || delta_t <= 0.0f) {
      delta_t = 0.01f; // Use a reasonable default
    }
    
    // Get scaled and calibrated IMU data
    float Gxyz[3], Axyz[3], Mxyz[3];
    get_scaled_IMU(Gxyz, Axyz, Mxyz);
    
    // Align coordinate systems for magnetometer
    Mxyz[1] = -Mxyz[1]; // Reflect Y and Z axes
    Mxyz[2] = -Mxyz[2]; // Must be done after offsets & scales applied
    
    // Update orientation estimation
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], 
                          Gxyz[0], Gxyz[1], Gxyz[2], 
                          Mxyz[0], Mxyz[1], Mxyz[2], 
                          delta_t);
    
    // Print data at specified interval
    if ((now - last_print) >= (PRINT_INTERVAL * 1000)) {
      // Calculate Euler angles
      roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5f - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0f * (q[0] * q[2] - q[1] * q[3]));
      yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5f - (q[2] * q[2] + q[3] * q[3]));
      
      // Convert to degrees
      yaw *= 180.0f / M_PI;
      pitch *= 180.0f / M_PI;
      roll *= 180.0f / M_PI;
      
      // Apply declination correction to yaw
      yaw = -(yaw + DECLINATION);
      if (yaw < 0) yaw += 360.0f;
      if (yaw >= 360.0f) yaw -= 360.0f;
      
      // Calculate update frequency
      uint16_t looptimer = (uint16_t)(1.0f / delta_t);
      
      // Print data in comma-separated format
      Serial.print(yaw, 0);
      Serial.print(F(", "));
      Serial.print(pitch, 0);
      Serial.print(F(", "));
      Serial.print(roll, 0);
      Serial.print(F(", "));
      Serial.println(looptimer);
      
      last_print = now;
    }
  }
}

// Get scaled and calibrated IMU data
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  // Apply gyro scaling and offsets
  Gxyz[0] = GYRO_SCALE * (imu.agmt.gyr.axes.x - G_OFFSET[0]);
  Gxyz[1] = GYRO_SCALE * (imu.agmt.gyr.axes.y - G_OFFSET[1]);
  Gxyz[2] = GYRO_SCALE * (imu.agmt.gyr.axes.z - G_OFFSET[2]);
  
  // Get raw accelerometer and magnetometer data
  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;
  
  // Apply accelerometer calibration
  float temp[3];
  for (byte i = 0; i < 3; i++) {
    temp[i] = Axyz[i] - A_B[i];
  }
  
  Axyz[0] = A_AINV[0][0] * temp[0] + A_AINV[0][1] * temp[1] + A_AINV[0][2] * temp[2];
  Axyz[1] = A_AINV[1][0] * temp[0] + A_AINV[1][1] * temp[1] + A_AINV[1][2] * temp[2];
  Axyz[2] = A_AINV[2][0] * temp[0] + A_AINV[2][1] * temp[1] + A_AINV[2][2] * temp[2];
  vector_normalize(Axyz);
  
  // Apply magnetometer calibration
  for (byte i = 0; i < 3; i++) {
    temp[i] = Mxyz[i] - M_B[i];
  }
  
  Mxyz[0] = M_AINV[0][0] * temp[0] + M_AINV[0][1] * temp[1] + M_AINV[0][2] * temp[2];
  Mxyz[1] = M_AINV[1][0] * temp[0] + M_AINV[1][1] * temp[1] + M_AINV[1][2] * temp[2];
  Mxyz[2] = M_AINV[2][0] * temp[0] + M_AINV[2][1] * temp[1] + M_AINV[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Fast vector normalization - optimized version
inline void vector_normalize(float v[3]) {
  float reciprocal_norm = 1.0f / sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] *= reciprocal_norm;
  v[1] *= reciprocal_norm;
  v[2] *= reciprocal_norm;
}

// Optimized Mahony quaternion update
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  // Observed horizon vector (W = A×M)
  float ex, ey, ez;  // Error terms
  
  // Pre-compute quaternion products (these are used multiple times)
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
  
  // Calculate horizon vector = a × m (cross product in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  
  // Normalize horizon vector
  norm = sqrtf(hx * hx + hy * hy + hz * hz);
  if (norm < 1e-6f) {
    return; // Avoid division by zero
  }
  
  // Fast reciprocal multiplication
  float recip_norm = 1.0f / norm;
  hx *= recip_norm;
  hy *= recip_norm;
  hz *= recip_norm;
  
  // Estimated direction of Up reference vector (gravity)
  float ux = 2.0f * (q2q4 - q1q3);
  float uy = 2.0f * (q1q2 + q3q4);
  float uz = q1q1 - q2q2 - q3q3 + q4q4;
  
  // Estimated direction of horizon (West) reference vector
  float wx = 2.0f * (q2q3 + q1q4);
  float wy = q1q1 - q2q2 + q3q3 - q4q4;
  float wz = 2.0f * (q3q4 - q1q2);
  
  // Calculate error (cross product of estimated and measured directions)
  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
  
  // Apply proportional feedback
  gx += KP * ex;
  gy += KP * ey;
  gz += KP * ez;
  
  // Integrate rate of change of quaternion
  float half_dt = 0.5f * dt;
  gx *= half_dt;  // Pre-multiply common factors
  gy *= half_dt;
  gz *= half_dt;
  
  // Update quaternion with integrated rate of change
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);
  
  // Normalize quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  recip_norm = 1.0f / norm;
  q[0] = q1 * recip_norm;
  q[1] = q2 * recip_norm;
  q[2] = q3 * recip_norm;
  q[3] = q4 * recip_norm;
}
