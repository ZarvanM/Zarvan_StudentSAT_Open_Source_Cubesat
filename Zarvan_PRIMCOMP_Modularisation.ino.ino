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


MISSION FORMAT (ZarSAT V1)
[TYPE] [DURATION] [DATA1] [DATA2] [DATA3] 
[ 1  ] [   15   ] [  0  ] [  0  ] [  0  ]





*///////////////////////////////////////////////
////////////////////////////////////////////////

#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;

int16_t missionplan1[7][5] = 
{ 
  {1, 232, 3, 4, 5},  //1
  {2, 214, 3, 4, 5213},  //2
  {3, 2112, 3132, 4, 5}, //3
  {4, 24121, 3, 4, 5}, //4
  {5, 2412, 3, 4, 5}, //5
  {6, 22, 3, 4, 5}, //6
  {7, 21, 3, 4123, 5}  //7

};

int16_t missionplan2[7][5] = 
{ 
  {1, 2, 3, 4, 5},  //1
  {1, 2, 3, 4, 5},  //2
  {1, 2, 3, 4, 5}, //3
  {1, 2, 3, 4, 5}, //4
  {1, 2, 3, 4, 5}, //5
  {1, 2, 3, 4, 5}, //6
  {1, 2, 3, 4, 5}  //7

};

int16_t dataToSend[] = {0, 0, 0, 0, 0}; 
const int dataLength = sizeof(dataToSend) / sizeof(dataToSend[0]);

int pointofmission = 0;
int opfreq = 0;
int adcscommand =  14;
int adcsfinish =  15;
int loop_timer  =   0;
int wooftimer   = 500; //watchdog
int done        = 0  ;
int adcsfinishState = 0;
int lastadcsfinishState=0;
int mission1length = 6;

int16_t type = 0;
int16_t duration = 0;
int16_t data1 = 0;
int16_t data2 = 0;
int16_t data3 = 0;




void setup() 
{
    Serial.begin(115200);  
    while (!Serial)
    {
    delay(1); // Avoids WEIRD serial garbage and bugginess. Remove SERIAL usage once done debugging. No one has USB in space...
    }
  for (int row = 0; row < 7; row++) 
{
    for (int col = 0; col < 5; col++) {
      Serial.print(missionplan1[row][col]);
      Serial.print(" ");  // Print a space between elements
    }
    Serial.println();  // Move to the next line after each row
  }

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  rtc.begin();

  
  pinMode (0, OUTPUT);
  digitalWrite(0,HIGH);
  pinMode (25, OUTPUT);
  pinMode (adcscommand, OUTPUT);
  pinMode (adcsfinish, INPUT_PULLDOWN);




  Serial.println("*  PRIMARY COMPUTER BOOTING  *");
  Serial.println("* FOR EVALUATION AND TESTING *");                                                    
  Serial.println("*  i2cWIRE 1 TIED TO ADCS    *");
  opfreq = rp2040.f_cpu()/1000000;
  Serial.println("RP2040 operating at: ");
  Serial.println(opfreq);
  
  
  rp2040.wdt_begin(wooftimer);
  loop_timer = millis();     

}




void loop()
{
  checkadcsstatus();  
  prepforADCS();




 while(millis() - loop_timer < 100)
  {
      aux_work(); 
  }           
  
  rp2040.wdt_reset();                  
  loop_timer = millis(); 
}









void checkadcsstatus()
{
adcsfinishState = digitalRead(adcsfinish);

  if (adcsfinishState == HIGH && lastadcsfinishState == LOW && pointofmission < mission1length) 
  {
     pointofmission++;
     Serial.print(" ADCS SAYS FINISHED, POM = ");
     Serial.println(pointofmission);
  }
    lastadcsfinishState = adcsfinishState;
  
  if (pointofmission == mission1length)
  {
         Serial.println("DONE!");
         mission1done();
  }

}



void mission1done()
{

}
void aux_work()
{

}

void prepforADCS()
{


type = 0;
duration = 0;
data1 = 0;
data2 = 0;
data3 = 0;

type     = missionplan1[pointofmission][0];
duration = missionplan1[pointofmission][1];
data1    = missionplan1[pointofmission][2];
data2    = missionplan1[pointofmission][3];
data3    = missionplan1[pointofmission][4];



dataToSend[0] = type;
dataToSend[1] = duration;
dataToSend[2] = data1;
dataToSend[3] = data2;
dataToSend[4] = data3;
  for (int i = 0; i < 5; i++) 
  {
    Serial.print(dataToSend[i]);
    Serial.print("..");
  }
  Serial.println(" ");

}

void sendtoADCS()
{
  Wire1.beginTransmission(9);
  Wire1.write(dataLength);
  for (int i = 0; i < dataLength; i++) 
  {
    Wire1.write((byte)(dataToSend[i] >> 8)); // Send the high byte
    Wire1.write((byte)(dataToSend[i] & 0xFF)); // Send the low byte
  }

  Wire1.endTransmission();
}

















