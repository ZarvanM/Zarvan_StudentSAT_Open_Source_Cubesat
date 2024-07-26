////////////////////////////////////////////////
////////////////////////////////////////////  
/*
Zarvan Movdawalla
Version 0.3 (ADCS+PRIMCOMP)
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


HARDWARE:
RP2040 (aboard Waveshare RP2040-Zero)
DS3231 (RTC) - Wire0 - 0x68
MPU6050(IMU) - Wire0 - 0x69
EXTERNAL I2C - Wire1 -

OSTAB RPI2W  - Wire1 - Roll/Pitch/Misc. Data Channels






TODO:
1. Find elapsed time!! (using RTCLib, maybe UNIX time with arduino time library???)
2. Alarms and stuff (SQW Pin?)

*///////////////////////////////////////////////
////////////////////////////////////////////////


#include <Wire.h>
#include <QMC5883LCompass.h>
#include "RTClib.h"
#include "SparkFun_External_EEPROM.h"

QMC5883LCompass compass;
ExternalEEPROM eeprom;
RTC_DS3231 rtc;


uint32_t loop_timer = 0;


int read1, read2 = 0;
int wooftimer = 4000;
float gyro_roll, gyro_pitch, gyro_yaw;
int raw_gyro_roll, raw_gyro_pitch, raw_gyro_yaw;
float acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float acc_x_cal, acc_y_cal, acc_z_cal;
int axis, degree = 0;
int POM, type, duration, param1,param2,param3,param4,param5 = 0;


int16_t MP1[20][10] = 
{ 


/* TYPES OF ACTIONS

1. End Of Mission
2. Disable   ADCS
3. Self Test ADCS
4. Calibrate ADCS
5. Reset     ADCS
6. Enable    ADCS
**
7. GYROMOTION(AXIS,DEG)
**

//[TYPE] [DURATION] [PARAM1] [PARAM2] [PARAM3] 
*/

  {1, 0, 0, 0, 1, 2, 3, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

void setup()
{
  rp2040.wdt_begin(wooftimer);
   startup();
   printMP1();
   BeginCOMS();
   EEPMEM();
   RTIMER();
   MPU1_Boot();
  rp2040.wdt_reset(); 
  loop_timer = micros(); 
}

void loop()
{
   MISSIONREAD();
                                            
}



void startup()
{
  Serial.begin(9600);  
    while (!Serial)
    {
    delay(1); // Avoids WEIRD serial garbage and bugginess. Remove SERIAL usage once done debugging. No one has USB in space...
    }
  Serial.println("**ADCS+PRMCOMP**");  
  Serial.println("**SERIAL INITT**");  
  Serial.println(" RP2040: 133MHz ");  
  Serial.println("WTCHDOG IS ALIVE");  
}

void printMP1()
{
  for (int i = 0; i < 20; i++) {
    String row = "";
    for (int j = 0; j < 10; j++) {
      row += String(MP1[i][j]) + " ";
    }
    Serial.println(row); 
  }
}

void BeginCOMS()
{
 eeprom.setMemoryType(32);
 Wire.setSDA(4);
 Wire.setSCL(5);
 Wire.begin(); //for sensors
 Wire1.setSDA(6);
 Wire1.setSCL(7);
 Wire1.begin(); 
 eeprom.begin(0b1010111, Wire);
 rtc.begin(&Wire);
}

void EEPMEM()
{
eeprom.put(0,  -9999);  //PHOENIX (MODE)
eeprom.put(10, 9999);  //COMPLETION STATUS

eeprom.get(0,  read1); //(location, data)
eeprom.get(10, read2); //(location, data)

Serial.println(read1);
Serial.println(read2);
}

void RTIMER()
{
    DateTime now = rtc.now();

    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print(" (");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println(")");
}


void IMU1READ()
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

  gyro_yaw   *= 1;
  gyro_pitch *= 1;
  gyro_roll  *= 1;

}



void MPU1_Boot()
{
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


void DEBUG_IMU1()
{
  Serial.print(gyro_pitch);
  Serial.print(" ");
  Serial.print(gyro_roll);
  Serial.print(" ");
  Serial.print(gyro_yaw);
  Serial.println(";");  
}

void GYROMOTION(int axis, int duration, int degree)
{
IMU1READ();
DEBUG_IMU1();




  while(micros() - loop_timer < 4000)
  {
    
  }           
  
  rp2040.wdt_reset();                  
  loop_timer = micros(); 
}

void MISSIONREAD()
{
for(POM=0; POM<20; POM++)
{
  MISSIONEXEC(type,duration,param1,param2,param3);
}
}

void MISSIONEXEC(int type, int duration,int param1,int param2,int param3)
{

switch (type) 
{
  case 2:
  //DISABLE   ADCS
    break;
  case 3:
  //SELFTEST  ADCS
    break;
  case 4:
  //CALIBRATE ADCS
    break;
  case 5:
  rp2040.reboot();
    break;
  case 6:
  //ENABLE    ADCS
    break;
  case 7:
  GYROMOTION(axis, duration, degree);

    break;
  default:
    break;
}

}







