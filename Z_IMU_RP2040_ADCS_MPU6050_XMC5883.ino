////////////////////////////////////////////////
////////////////////////////////////////////  
/*
Zarvan Movdawalla
Version 0.2
________________________________________
DEVELOPED AT MPSTME, INDIA.
PART OF ASLS (Air Sea Land Space) initiative by Zarvan Movdawalla.


MPU6050 , RP2040
CF + AHRS 
DUAL CORE COMPATIBLE

STRICTLY not for commercial use. 
STRICTLY not for safety-critical/unsafe/hazardous applications.

Licensed under: CC BY-NC-SA


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Special thanks to Joop Brokking (for IMU CF methodology), Earle F. Philhower III (for RP2040 Arduino Core). This ADCS is based off of their pioneering work.




*///////////////////////////////////////////////
////////////////////////////////////////////////

#include <Wire.h>
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);

int gyro_roll, gyro_pitch, gyro_yaw;
float acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float acc_x_cal, acc_y_cal, acc_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
int pitch, roll, yaw = 0;


float thrustaccel = 0;
float totalaccel  = 0;
float xaccelcorrect=0;
float yaccelcorrect=0;

float comp_x, comp_y, comp_z = 0;
float heading = 0;
float actual_compass_heading, compass_x_horizontal, compass_y_horizontal = 0;
int opfreq = 0;
float declination = 0;
float compoff_x, compoff_y, compoff_z = 0;
float compscl_x, compscl_y, compscl_z = 0;
int comp_cal_vals[6] = {-1100,1099,-863,1387,-1160,1050};


void setup() 
{
  Wire.begin();  
  Serial.begin(115200);  
    while (!Serial)
    {
    delay(1); // Avoids WEIRD serial garbage and bugginess. Remove SERIAL usage once done debugging. No one has USB in space...
    }



  
  opfreq = rp2040.f_cpu()/1000000;
  
  setup_mpu_6050_registers();  

  Serial.println("*ZAR-ADCS-MPSTME-ASLS        *");
  Serial.println("* FOR EVALUATION AND TESTING *");                                                    
  Serial.println("* SINGLE GYRO WITH MAGCOMPASS*");
  opfreq = rp2040.f_cpu()/1000000;
  Serial.println("RP2040 operating at: ");
  Serial.println(opfreq);
    
    while (!compass.begin())
  {
    Serial.println("COMPASS FAILURE: COMP NOINIT");
    delay(500);
  }
  /////////////////////////////////////////////////////////////// COMPASS CONFIG
  Serial.println("COMPASS INIT SUCCESS, PROCEED "); 

    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
    compass.setDeclinationAngle(0);

  Serial.println("COMPASS CONF SUCCESS, PROCEED "); 

  Serial.println("GYRO CAL INIT, NULLING BIASES "); 
  delay(500);

  
  for (int cal_int = 0; cal_int < 500 ; cal_int ++)
  {                 

    read_mpu_6050_data();                                              
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                             
    delayMicroseconds(3600);                                                          
  }
  gyro_roll_cal /= 500;                                                  
  gyro_pitch_cal /= 500;                                                  
  gyro_yaw_cal /= 500;                                                 



for (int cal_int = 0; cal_int < 500 ; cal_int ++)
  {                 

    read_mpu_6050_data();                                              
    acc_x_cal += acc_x;                                              
    acc_y_cal += acc_y;                                              
   // acc_z_cal += acc_z;   UNCOMMENT only and ONLY if you are actually in space. DO NOT UNCOMMENT ON EARTH. BAD THINGS HAPPEN!                                          
    delayMicroseconds(3600);                                                          
  }
  acc_x_cal /= 500;                                                  
  acc_y_cal /= 500;                                                  
  acc_z_cal /= 500;   

  Serial.println("GYRO CAL RESULTS: X, Y, Z");
  Serial.print(gyro_roll_cal);
  Serial.print(" ");
  Serial.print(gyro_pitch_cal);
  Serial.print(" ");
  Serial.println(gyro_yaw_cal);
  
  Serial.println("ACCEL CAL RESULTS: X, Y, Z");
  Serial.print(acc_x_cal);
  Serial.print(" ");
  Serial.print(acc_y_cal);
  Serial.print(" ");
  Serial.println(acc_z_cal);
  delay(4000);


  /////////////////////////////////////////////////////////////// COMPASS CONFIG

  Serial.println("COMPASS CAL BEGINS...");
  delay(2000);
  Serial.println("LESSGOOO...");
/*
for (int cal_int = 0; cal_int < 7000 ; cal_int ++)
  {                 
  sVector_t mag = compass.readRaw();
  comp_x = mag.XAxis*-1;
  comp_y = mag.YAxis*-1;
  comp_z = mag.ZAxis;

    if (comp_x < comp_cal_vals[0])comp_cal_vals[0] = comp_x;
    if (comp_x > comp_cal_vals[1])comp_cal_vals[1] = comp_x;
    if (comp_y < comp_cal_vals[2])comp_cal_vals[2] = comp_y;
    if (comp_y > comp_cal_vals[3])comp_cal_vals[3] = comp_y;
    if (comp_z < comp_cal_vals[4])comp_cal_vals[4] = comp_z;
    if (comp_z > comp_cal_vals[5])comp_cal_vals[5] = comp_z;



    delayMicroseconds(3600);                                                          
  }

*/
for(int i = 0; i < 6; i++)
{
  Serial.print(comp_cal_vals[i]);
  Serial.print(",");
}
delay(2000);
  Serial.println("__");

compscl_y = ((float)comp_cal_vals[1] - comp_cal_vals[0]) / (comp_cal_vals[3] - comp_cal_vals[2]);
compscl_z = ((float)comp_cal_vals[1] - comp_cal_vals[0]) / (comp_cal_vals[5] - comp_cal_vals[4]);

compoff_x = (comp_cal_vals[1] - comp_cal_vals[0]) / 2 - comp_cal_vals[1];
compoff_y = (((float)comp_cal_vals[3] - comp_cal_vals[2]) / 2 - comp_cal_vals[3]) * compscl_y;
compoff_z = (((float)comp_cal_vals[5] - comp_cal_vals[4]) / 2 - comp_cal_vals[5]) * compscl_z;


Serial.println("CALCULATED COMPASS CALIBRATION VALUES");
Serial.println("COMP SCALE: Y, Z ~ COMP OFFSETS: X, Y, Z");
Serial.print(compscl_y);
Serial.print(" ");
Serial.print(compscl_z);
Serial.print(" ");
Serial.print(compoff_x);
Serial.print("   ");
Serial.print(compoff_y);
Serial.print(" ");
Serial.println(compoff_z);

delay(15000);



  rp2040.wdt_begin(500);
  loop_timer = micros();                                               //Reset the loop timer
}


void loop(){

  read_mpu_6050_data();                                                

    gyro_roll -= gyro_roll_cal;                                  //Subtact the manual gyro roll calibration value.
    gyro_pitch -= gyro_pitch_cal;                                //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= gyro_yaw_cal;                                                

  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
  
  angle_yaw += (float)gyro_yaw * 0.0000611;                                    
    if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (angle_yaw >= 360) angle_yaw -= 360; 

  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);   
            
  
//Elimination of steady-state-offset (SSO) errors, done!  
acc_x -= acc_x_cal;
acc_y -= acc_y_cal;
//acc_z -= acc_x_cal; COMMENTED, DUE TO WEIRDNESS IN CALIB DUE TO EARTH'S GRAV

//Elimination of scaling errors, done!  
acc_x *= 1;
acc_y *= 1;
acc_z *= 1.0242; 

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  
  totalaccel   = acc_total_vector/1671;
  thrustaccel  = acc_z/1671;
  xaccelcorrect= acc_x/1671;
  yaccelcorrect= acc_y/1671;
   
  
  temperature = ((temperature+521)/340) + 35;


  while(micros() - loop_timer < 4000)
  {
      aux_work(); 
  }           
  
  rp2040.wdt_reset();                  
  loop_timer = micros(); 
                                               
}


void read_mpu_6050_data()
{                                             
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_y = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
  acc_x = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
  acc_z = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable
  temperature = (int16_t)(Wire.read()<<8|Wire.read());                            //Add the low and high byte to the temperature variable
  gyro_roll = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
  gyro_pitch = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
  gyro_yaw = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable
  gyro_pitch *= -1;                                            //Invert the direction of the axis.
  gyro_yaw *= -1;  
}

void aux_work()
{   

  //Serial.println(".");
  pitch = angle_pitch;
  roll  = angle_roll;
  yaw   = angle_yaw;

float declinationAngle = (0.0 + (0.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  comp_x = mag.XAxis*-1;
  comp_y = mag.YAxis*-1;
  comp_z = mag.ZAxis;

    comp_y += compoff_y;                              //Add the y-offset to the raw value.
    comp_y *= compscl_y;                               //Scale the y-value so it matches the other axis.
    comp_z += compoff_z;                              //Add the z-offset to the raw value.
    comp_z *= compscl_z;                               //Scale the z-value so it matches the other axis.
    comp_x += compoff_x;     

  compass_x_horizontal = (float)comp_x * cos(angle_pitch * -0.0174533) + (float)comp_y * sin(angle_roll * -0.0174533) * sin(angle_pitch * -0.0174533) - (float)comp_z * cos(angle_roll * -0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)comp_y * cos(angle_roll * -0.0174533) + (float)comp_z * sin(angle_roll * -0.0174533);

  //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
  //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
  if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
  if (actual_compass_heading < 0) actual_compass_heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360;






  Serial.print(actual_compass_heading);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(","); 
  Serial.print(roll);
  Serial.print(","); 
  Serial.println(yaw);
/*
  Serial.print(",");
  Serial.print(totalaccel,2);
  Serial.print(",");
  Serial.print(thrustaccel,2);
  Serial.print(",");

  Serial.print(comp_x,0);
  Serial.print(",");
  Serial.print(comp_y,0);
  Serial.print(",");
  Serial.println(comp_z,0);

*/


}



void setup_mpu_6050_registers(){
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}







