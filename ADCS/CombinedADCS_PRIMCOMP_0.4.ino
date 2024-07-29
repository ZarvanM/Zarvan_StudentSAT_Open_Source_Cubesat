////////////////////////////////////////////////
////////////////////////////////////////////  
/*
Zarvan Movdawalla
Version 0.4 (ADCS+PRIMCOMP)
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

uint32_t loopTimer = 0;

int eepromRead1 = 0, eepromRead2 = 0;
int watchdogTimer = 4000;
float gyroRoll, gyroPitch, gyroYaw;
int rawGyroRoll, rawGyroPitch, rawGyroYaw;
float accelX, accelY, accelZ;
int temperature;
float gyroRollCal, gyroPitchCal, gyroYawCal;
float accelXCal, accelYCal, accelZCal;
int actionAxis, actionDegree = 0;
int pendingOperation, actionType, actionDuration, param1, param2, param3, param4, param5 = 0;

int16_t missionPlan[20][10] = {
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
  rp2040.wdt_begin(watchdogTimer);
  startup();
  printMissionPlan();
  beginCommunications();
  initializeEEPROM();
  setupRTC();
  initializeIMU();
  rp2040.wdt_reset(); 
  loopTimer = micros(); 
}

void loop()
{
  readMission();
}

void startup()
{
  Serial.begin(9600);  
  while (!Serial)
  {
    delay(1);
  }
  Serial.println("**ADCS+PRMCOMP**");  
  Serial.println("**SERIAL INIT**");  
  Serial.println(" RP2040: 133MHz ");  
  Serial.println("WATCHDOG IS ALIVE");  
}

void printMissionPlan()
{
  for (int i = 0; i < 20; i++) {
    String row = "";
    for (int j = 0; j < 10; j++) {
      row += String(missionPlan[i][j]) + " ";
    }
    Serial.println(row); 
  }
}

void beginCommunications()
{
  eeprom.setMemoryType(32);
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin(); 
  eeprom.begin(0b1010111, Wire);
  rtc.begin(&Wire);
}

void initializeEEPROM()
{
  eeprom.put(0,  -9999);
  eeprom.put(10, 9999);

  eeprom.get(0,  eepromRead1);
  eeprom.get(10, eepromRead2);

  Serial.println(eepromRead1);
  Serial.println(eepromRead2);
}

void setupRTC()
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

void readIMU()
{                                          
  Wire.beginTransmission(0x69);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x69,14);
  while(Wire.available() < 14);
  accelY = (int16_t)(Wire.read()<<8|Wire.read());
  accelX = (int16_t)(Wire.read()<<8|Wire.read());
  accelZ = (int16_t)(Wire.read()<<8|Wire.read());
  temperature = (int16_t)(Wire.read()<<8|Wire.read());
  rawGyroRoll = (int16_t)(Wire.read()<<8|Wire.read());
  rawGyroPitch = (int16_t)(Wire.read()<<8|Wire.read());
  rawGyroYaw = (int16_t)(Wire.read()<<8|Wire.read());
  
  gyroYaw   = (float)rawGyroYaw;
  gyroPitch = (float)rawGyroPitch;
  gyroRoll  = (float)rawGyroRoll;
}

void initializeIMU()
{
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x69);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x69);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x69);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

void debugIMU()
{
  Serial.print(gyroPitch);
  Serial.print(" ");
  Serial.print(gyroRoll);
  Serial.print(" ");
  Serial.print(gyroYaw);
  Serial.println(";");  
}

void executeGyroMotion(int axis, int duration, int degree)
{
  readIMU();
  debugIMU();
  while(micros() - loopTimer < 4000)
  {
  }           
  rp2040.wdt_reset();                  
  loopTimer = micros(); 
}

void readMission()
{
  for(pendingOperation = 0; pendingOperation < 20; pendingOperation++)
  {
    executeMission(actionType, actionDuration, param1, param2, param3);
  }
}

void executeMission(int tempType, int tempDuration, int tempParam1, int tempParam2, int tempParam3)
{
  switch (tempType) 
  {
    case 2:
      // DISABLE   ADCS
      break;
    case 3:
      // SELFTEST  ADCS
      break;
    case 4:
      // CALIBRATE ADCS
      break;
    case 5:
      rp2040.reboot();
      break;
    case 6:
      // ENABLE    ADCS
      break;
    case 7:
      executeGyroMotion(actionAxis, actionDuration, actionDegree);
      break;
    case 8:
      // PHOENIX
      break;
    case 9:
      // SAFE
      break;
    case 10:
      // NORMAL
      break;
    case 11:
      // OVRD
      break;
    default:
      break;
  }
}
