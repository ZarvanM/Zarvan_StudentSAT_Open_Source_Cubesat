////////////////////////////////////////////////
////////////////////////////////////////////  
/*
Zarvan Movdawalla
Version 0.2
________________________________________
DEVELOPED AT MPSTME, INDIA.
PART OF ASLS (Air Sea Land Space) initiative by Zarvan Movdawalla.


RP2040
PRIMARY COMPUTER
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


MISSION FORMAT (ZarSAT V1) **massively WIP**
[TYPE] [DURATION] [DATA1] [DATA2] [DATA3] [DATA4] [DATA5] 
[ 0  ] [   0    ] [  0  ] [  0  ] [  0  ] [  0  ] [  0  ]     //shutdown adcs
[ 1  ] [   0    ] [  0  ] [  0  ] [  0  ] [  0  ] [  0  ]     //disable  adcs
[ 2  ] [ seconds] [  0  ] [  0  ] [  0  ] [  0  ] [  0  ]     //adcs magneto-observer mode ON (return X,Y,Z and BDOT values)
[ 3  ] [   0    ] [move1] [move2] [move3] [move4] [move5]     //raw reaction wheel driver (P, Y, R), MOTIONS WILL BE CLEARLY ONE AFTER ANOTHER, NOT PARALLEL. NO OFFSET >90 Degrees
[ 4  ] [seconds ] [XAXIS] [YAXIS] [ZAXIS] [  0  ] [  0  ]     //raw magnetorquer driving (x,y,z) MAGNITUDE BASED

STATUS TABLE:
-1 = INVALID
 0 = PHOENIX MODE
 1 = SAFE
 2 = MISSION MODE

MISSION COMPLETION STATUS:
 0 = INCOMPLETE
 1 = COMPLETED

ONBOARD HEALTH STATUS:
OHS_ADCS:
0 = NO FAULT
1 = IMU FAIL
2 = TOTAL ADCS FAILURE

OHS_EPS:
0 = NO FAULT
1 = TOTAL EPS FAILURE


*///////////////////////////////////////////////
////////////////////////////////////////////////

#include <Wire.h>
#include "RTClib.h"
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
ExternalEEPROM eeprom;
RTC_DS3231 rtc;


int16_t missionplan1[7][7] = 
{ 
  {1, 0, 0, 0, 1, 2, 3},  //1
  {2, 0, 0, 0, 1, 2, 3},  //2
  {3, 0, 0, 0, 1, 2, 3},  //3
  {4, 0, 0, 0, 1, 2, 3},  //4
  {5, 0, 0, 0, 1, 2, 3},  //5
  {6, 0, 0, 0, 1, 2, 3},  //6
  {7, 0, 0, 0, 1, 2, 3}   //7

};


int16_t dataToSend[] = {0,0,0,0,0,0,0}; 
const int dataLength = sizeof(dataToSend) / sizeof(dataToSend[0]);

int pointofmission = 0;
int opfreq = 0;
int adcscommand =  14;
int adcsfinish =  15;
int loop_timer  =   0;
int wooftimer   = 1500; //watchdog
int done        = 0  ;
int adcsfinishState = 0;
int lastadcsfinishState=0;


int16_t type = 0;
int16_t duration = 0;
int16_t data1 = 0;
int16_t data2 = 0;
int16_t data3 = 0;




void setup() 
{
    Serial.begin(115200);  
    /* while (!Serial)
    {
    delay(1); // Avoids WEIRD serial garbage and bugginess. Remove SERIAL usage once done debugging. No one has USB in space...
    }
    */
  for (int row = 0; row < 7; row++) 
{
    for (int col = 0; col < 7; col++) {
      Serial.print(missionplan1[row][col]);
      Serial.print(" ");  // Print a space between elements
    }
    Serial.println();  // Move to the next line after each row
  }
  
  eeprom.setMemoryType(32);
  begini2c();
  eeprom.begin(0b1010111, Wire);
  Serial.println("Memory detected!");
  rtc.begin(&Wire);
  
  //PIN DECLARATION
  pinMode (0, OUTPUT);
  digitalWrite(0,HIGH);
  pinMode (25, OUTPUT);
  pinMode (adcscommand, OUTPUT);
  pinMode (adcsfinish, INPUT_PULLDOWN);

//EEPROM OPS (32Byte)

//EEPROM writing (PRE-LAUNCH)
  int16_t membankID = 1;
  int16_t status = -1;
  









  eeprom.put(0,  0);  //PHOENIX (MODE)
  eeprom.put(10, 1);  //COMPLETION STATUS

 // eeprom.get(0, read1); //(location, data)
 // eeprom.get(10, read2); //(location, data)





  rp2040.wdt_begin(wooftimer);
  loop_timer = millis();     
}




void loop()
{

  checkadcsstatus();  
  prepforADCS();
  sendtoADCS(dataToSend, 7);




 while(millis() - loop_timer < 100)
  {
      aux_work(); 
  }           
  
  rp2040.wdt_reset();                  
  loop_timer = millis(); 
}










void begini2c()
{
 Wire.setSDA(4);
 Wire.setSCL(5);
  Wire.begin(); //for sensors
 Wire1.setSDA(6);
 Wire1.setSCL(7);
  Wire1.begin(); //for adcs<->prim
}


void checkadcsstatus()
{
adcsfinishState = digitalRead(adcsfinish);

  if (adcsfinishState == HIGH && lastadcsfinishState == LOW && pointofmission < 7) 
  {
     pointofmission++;
     Serial.print(" ADCS SAYS FINISHED, POM = ");
     Serial.println(pointofmission);
  }
    lastadcsfinishState = adcsfinishState;
  
  if (pointofmission == 7)
  {
         Serial.println("DONE!");
         mission1done();
  }

}



void mission1done()
{
  delay(20000);
}
void aux_work()
{

}

void prepforADCS()
{
      DateTime now = rtc.now();

for(int i = 0; i < 7 ; i++)
{
  dataToSend[i] = missionplan1[pointofmission][i];
}
  Serial.print(":");
  Serial.print(pointofmission);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}






void sendtoADCS(int16_t *array, int length) 
{
    Wire1.beginTransmission(9);
    for (int i = 0; i < length; i++) {
        Wire1.write((uint8_t)(array[i] >> 8)); // Send the high byte
        Wire1.write((uint8_t)(array[i] & 0xFF));  // Send the low byte
    }
    Wire1.endTransmission();
  
  for (int i = 0; i < 7; i++) 
  {
    Serial.print(dataToSend[i]);
    Serial.print(",");
  }
  
  Serial.println(" ");

}













