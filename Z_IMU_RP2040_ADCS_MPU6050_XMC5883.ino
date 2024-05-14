////////////////////////////////////////////////
////////////////////////////////////////////  
/*
Zarvan Movdawalla
Version 0.2
MPU6050 , RP2040
CF + AHRS 
DUAL CORE COMPATIBLE

STRICTLY not for commercial use. 
STRICTLY not for safety-critical/unsafe/hazardous applications.

Licensed under: CC BY-NC-SA

Disclaimer: Software is provided as-is, we make absolutely no claim or warranty towards its safety and reliability. It is purely an evaluation tool for advanced computing applications with microcontrollers.
Users acknowledge and agree that the use of the Software involves inherent risks, including but not limited to the risk of hardware damage, injury, or loss of property. 
Users assume all risks associated with the use of ZarvanM's AHRS-PID algorithm and software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.






*///////////////////////////////////////////////
////////////////////////////////////////////////

#include <Wire.h>

int gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
float acc_x_cal, acc_y_cal, acc_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
int pitch, roll = 0;
float gyro_x_telem, gyro_y_telem = 0;


float thrustaccel = 0;
float totalaccel  = 0;
float xaccelcorrect=0;
float yaccelcorrect=0;

int opfreq = 0;

void setup() 
{
  Wire.begin();  
  Serial.begin(115200);  
    while (!Serial)
    {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens  
    }



  
  opfreq = rp2040.f_cpu()/1000000;
  
  setup_mpu_6050_registers();  

  Serial.println("*ZAR-ADCS-MPSTME-ASLS        *");
  Serial.println("* FOR EVALUATION AND TESTING *");                                                    
  Serial.println("* SINGLE GYRO, NO MAGNETO    *");
  opfreq = rp2040.f_cpu()/1000000;
  Serial.println("RP2040 operating at: ");
  Serial.println(opfreq);
  Serial.println("GYRO CAL INIT, NULLING BIASES"); 
  delay(500);

  
  for (int cal_int = 0; cal_int < 1500 ; cal_int ++)
  {                 

    read_mpu_6050_data();                                              
    gyro_x_cal += gyro_x;                                              
    gyro_y_cal += gyro_y;                                              
    gyro_z_cal += gyro_z;                                             
    delay(3);                                                          
  }
  gyro_x_cal /= 1500;                                                  
  gyro_y_cal /= 1500;                                                  
  gyro_z_cal /= 1500;                                                 



for (int cal_int = 0; cal_int < 1500 ; cal_int ++)
  {                 

    read_mpu_6050_data();                                              
    acc_x_cal += acc_x;                                              
    acc_y_cal += acc_y;                                              
   // acc_z_cal += acc_z;                                             
    delay(3);                                                          
  }
  acc_x_cal /= 1500;                                                  
  acc_y_cal /= 1500;                                                  
  acc_z_cal /= 1500;   

  Serial.println("GYRO CAL RESULTS: X, Y, Z");
  Serial.print(gyro_x_cal);
  Serial.print(" ");
  Serial.print(gyro_y_cal);
  Serial.print(" ");
  Serial.println(gyro_z_cal);
  
  Serial.println("ACCEL CAL RESULTS: X, Y, Z");
  Serial.print(acc_x_cal);
  Serial.print(" ");
  Serial.print(acc_y_cal);
  Serial.print(" ");
  Serial.println(acc_z_cal);
  delay(4000);


  rp2040.wdt_begin(500);
  loop_timer = micros();                                               //Reset the loop timer
}


void loop(){

  read_mpu_6050_data();                                                

  gyro_x -= gyro_x_cal;                                               
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                

  angle_pitch += gyro_x * 0.0000611;                                   
  angle_roll  += gyro_y * 0.0000611;                                   
  
  gyro_x_telem = gyro_x / 65.5; 
  gyro_y_telem = gyro_y / 65.5; 



  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);              
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);              
  
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


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_x variable
  acc_y = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_y variable
  acc_z = (int16_t)(Wire.read()<<8|Wire.read());                                  //Add the low and high byte to the acc_z variable
  temperature = (int16_t)(Wire.read()<<8|Wire.read());                            //Add the low and high byte to the temperature variable
  gyro_x = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_x variable
  gyro_y = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_y variable
  gyro_z = (int16_t)(Wire.read()<<8|Wire.read());                                 //Add the low and high byte to the gyro_z variable

}

void aux_work()
{   

  //Serial.println(".");
  pitch = angle_pitch_output;
  roll  = angle_roll_output;
  
  Serial.print(90);
  Serial.print(",");
  Serial.print(-90);   
  Serial.print(",");                                           
  Serial.print(angle_pitch_output,0);
  Serial.print(",");
  Serial.print(angle_roll_output,0);
  Serial.print(",");
  Serial.print(totalaccel,2);
  Serial.print(",");
  Serial.print(thrustaccel,2);
  Serial.print(",");
  Serial.print(xaccelcorrect,2);
  Serial.print(",");
  Serial.print(yaccelcorrect,2);
  Serial.print(",");
  Serial.println(temperature);


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








