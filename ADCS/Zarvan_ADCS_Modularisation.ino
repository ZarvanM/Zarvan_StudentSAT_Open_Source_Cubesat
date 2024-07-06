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
#include <QMC5883LCompass.h>
#include "RTClib.h"


QMC5883LCompass compass;
RTC_DS1307 rtc;

float gyro_roll, gyro_pitch, gyro_yaw;
int raw_gyro_roll, raw_gyro_pitch, raw_gyro_yaw;
float acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float acc_x_cal, acc_y_cal, acc_z_cal;
unsigned long loop_timer;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
int pitch, roll, yaw = 0;
int wooftimer = 500; //watchdog

float gyro_yaw_scale = 1.044;
float gyro_pit_scale = 1.05;
float gyro_rol_scale = 1.0;

float thrustaccel = 0;
float totalaccel  = 0;
float xaccelcorrect=0;
float yaccelcorrect=0;

float comp_x, comp_y, comp_z = 0;

int opfreq = 0;

float compoff_x, compoff_y, compoff_z = 0;
float compscl_x, compscl_y, compscl_z = 0;
int comp_cal_vals[6] = {-1100,1099,-863,1387,-1160,1050};

unsigned long loop_count = 0;
int loop_ctr1 = 0;
int loopfreq  = 250; //GYRO/ACCEL primary loop loopfreq. DO NOT CHANGE unless certain. Will affect performance at extreme vals


void setup() 
{
    Serial.begin(115200);  
    while (!Serial)
    {
    delay(1); // Avoids WEIRD serial garbage and bugginess. Remove SERIAL usage once done debugging. No one has USB in space...
    }
  
  
  Wire.begin();
  rtc.begin();
 
  

  compass.init();
  setup_mpu_6050_registers();  



  opfreq = rp2040.f_cpu()/1000000;
  


  Serial.println("*ZAR-ADCS-MPSTME-ASLS        *");
  Serial.println("* FOR EVALUATION AND TESTING *");                                                    
  Serial.println("* SINGLE GYRO WITH MAGCOMPASS*");
  opfreq = rp2040.f_cpu()/1000000;
  Serial.println("RP2040 operating at: ");
  Serial.println(opfreq);
  




/////////////////////////////////////////////////////////////// GYRO CALIB

  
  for (int cal_int = 0; cal_int < 3500 ; cal_int ++)
  {                 

    read_mpu_6050_data();                                              
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                             
    delayMicroseconds(3550);                                                          
  }
  gyro_roll_cal /= 3500;                                                  
  gyro_pitch_cal /= 3500;                                                  
  gyro_yaw_cal /= 3500;                                                 
  
  Serial.println("GYRO");


/////////////////////////////////////////////////////////////// ACCELERO CALIB

for (int cal_int = 0; cal_int < 500 ; cal_int ++)
  {                 

    read_mpu_6050_data();                                              
    acc_x_cal += acc_x;                                              
    acc_y_cal += acc_y;                                              
   // acc_z_cal += acc_z;   UNCOMMENT only and ONLY if you are actually in space. DO NOT UNCOMMENT ON EARTH. BAD THINGS HAPPEN!                                          
    delayMicroseconds(3650);                                                          
  }
  acc_x_cal /= 500;                                                  
  acc_y_cal /= 500;                                                  
  acc_z_cal /= 500;   
/////////////////////////////////////////////////////////////// COMPASS CONFIG

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


/////////////////////////////////////////////////////////////// COMPASS CALIB

  Serial.println("COMPASS CAL BEGINS...");
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
  Serial.println("__");

compscl_y = ((float)comp_cal_vals[1] - comp_cal_vals[0]) / (comp_cal_vals[3] - comp_cal_vals[2]);
compscl_z = ((float)comp_cal_vals[1] - comp_cal_vals[0]) / (comp_cal_vals[5] - comp_cal_vals[4]);

compoff_x = ((float)comp_cal_vals[1] - comp_cal_vals[0]) / 2 - comp_cal_vals[1];
compoff_y = (((float)comp_cal_vals[3] - comp_cal_vals[2]) / 2 - comp_cal_vals[3]) * compscl_y;
compoff_z = (((float)comp_cal_vals[5] - comp_cal_vals[4]) / 2 - comp_cal_vals[5]) * compscl_z;


Serial.println("CALCULATED COMPASS CALIBRATION VALUES");
Serial.println("COMP SCALE: Y, Z ~ COMP OFFSETS: X, Y, Z");
Serial.print(compscl_y);
Serial.print(" ");
Serial.print(compscl_z);
Serial.print("  ");
Serial.print(compoff_x);
Serial.print(" ");
Serial.print(compoff_y);
Serial.print(" ");
Serial.println(compoff_z);

delay(1000);

/////////////////////////////////////////////////////////////// WATCHDOG CONFIG
  rp2040.wdt_begin(wooftimer);
  loop_timer = micros();                                               //Reset the loop timer
}



void loop()
{
loop_count ++;
loop_ctr1 ++;

  read_mpu_6050_data();
  read_mag_xmc1_data();                                                

    gyro_roll -= gyro_roll_cal;                                  //Subtact the manual gyro roll calibration value.
    gyro_pitch -= gyro_pitch_cal;                                //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= gyro_yaw_cal;                                                

  angle_pitch += (float)gyro_pitch;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll  += (float)gyro_roll;                               //Calculate the traveled roll angle and add this to the angle_roll variable.
  angle_yaw   += (float)gyro_yaw;  
  
                              
    if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (angle_yaw >= 360) angle_yaw -= 360; 



  
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
  
  pitch = angle_pitch;
  roll  = angle_roll;
  yaw   = angle_yaw;

  comp_x *= -1;
  comp_y *= -1;

    comp_y += compoff_y;                             
    comp_y *= compscl_y;                              
    comp_z += compoff_z;                              
    comp_z *= compscl_z;                               
    comp_x += compoff_x;    

  while(micros() - loop_timer < 4000)
  {
      aux_work(); 
  }           
  
  rp2040.wdt_reset();                  
  loop_timer = micros(); 
                                               
}




void read_mag_xmc1_data()
{
  compass.read();
  comp_x = compass.getX();
  comp_y = compass.getY();
  comp_z = compass.getZ();
  

  comp_x *= -1;
  comp_y *= -1;


     
    comp_y += compoff_y;                              //Add the y-offset to the raw value.
    comp_y *= compscl_y;                               //Scale the y-value so it matches the other axis.
    comp_z += compoff_z;                              //Add the z-offset to the raw value.
    comp_z *= compscl_z;                               //Scale the z-value so it matches the other axis.
    comp_x += compoff_x;     

}




void read_mpu_6050_data()
{                                             
  Wire.beginTransmission(0x69);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x69,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_y = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
  acc_x = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
  acc_z = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable
  temperature = (int16_t)(Wire.read()<<8|Wire.read());                            //Add the low and high byte to the temperature variable
  raw_gyro_roll = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
  raw_gyro_pitch = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
  raw_gyro_yaw = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable
  

  gyro_yaw   = (float)raw_gyro_yaw;
  gyro_pitch = (float)raw_gyro_pitch;
  gyro_roll  = (float)raw_gyro_roll;

  gyro_yaw   *= -1/(loopfreq*65.5);
  gyro_pitch *= -1/(loopfreq*65.5);
  gyro_roll  *=  1/(loopfreq*65.5);

  gyro_yaw   *= gyro_yaw_scale;
  gyro_pitch *= gyro_pit_scale;
  gyro_roll  *= gyro_rol_scale;

}

void aux_work()
{   
 


if(loop_ctr1 == 25)
{
  DateTime now = rtc.now();

  Serial.print(pitch);
  Serial.print(","); 
  Serial.print(roll);
  Serial.print(","); 
  Serial.print(yaw);
  Serial.print(" ** "); 
  Serial.print(comp_x,0);
  Serial.print(",");
  Serial.print(comp_y,0);
  Serial.print(",");
  Serial.print(comp_z,0);
  Serial.print(" ** "); 
  Serial.print(loop_count); 
  Serial.println("**");
  loop_ctr1 = 0;
}
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
  Wire.beginTransmission(0x69);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(0x69);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(0x69);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();
  Wire.beginTransmission(0x69);                                        //Start communicating with the MPU-6050
  Wire.write(0x1A);                                                    //Send the requested starting register
  Wire.write(0x03);                                                    //Set the requested starting register
  Wire.endTransmission();

}







