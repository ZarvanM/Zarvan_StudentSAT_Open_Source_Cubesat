  int16_t channel_A0 = 0;
  int16_t channel_A1 = 0;
  int16_t channel_A2 = 0;
  int16_t channel_A3 = 0;

  int16_t channel_A0_mV = 0;
  int16_t channel_A1_mV = 0;
  int16_t channel_A2_mV = 0;
  int16_t channel_A3_mV = 0;












#include <SparkFun_ADS1015_Arduino_Library.h>
#include <Wire.h>

ADS1015 adcSensor;

void setup() {
    Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  Serial.begin(115200);
  if (adcSensor.begin(0x48) == true) // connect to device at address 0x49 (default is 0x48)
    // **note, you must cut a trace and close the "0x49" jumper for this to work.
  {
    Serial.println("Device found. I2C connections are good.");
  }
  else
  {
    Serial.println("Device not found. Check wiring.");
    while (1); // stall out forever
  }

  adcSensor.setSampleRate(ADS1015_CONFIG_RATE_3300HZ);
  adcSensor.setGain(ADS1015_CONFIG_PGA_4); // PGA gain set to 1
  adcSensor.useConversionReady(true);
}

void loop() {
  channel_A0 = adcSensor.getSingleEndedSigned(0);
  channel_A1 = adcSensor.getSingleEndedSigned(1);
  channel_A2 = adcSensor.getSingleEndedSigned(2);
  channel_A3 = adcSensor.getSingleEndedSigned(3);

  channel_A0_mV = adcSensor.getSingleEndedMillivolts(0)*1.358;
  channel_A1_mV = adcSensor.getSingleEndedMillivolts(1)*1.358;
  channel_A2_mV = adcSensor.getSingleEndedMillivolts(2)*1.358;
  channel_A3_mV = adcSensor.getSingleEndedMillivolts(3)*1.358;

  
  Serial.print("A0: ");
  Serial.print(channel_A0_mV);
  Serial.print("mA   ");
 
  Serial.print("A1: ");
  Serial.print(channel_A1_mV);
  Serial.print("mA   ");

  Serial.print("A2: ");
  Serial.print(channel_A2_mV);
  Serial.print("mA   ");
 
  Serial.print("A3: ");
  Serial.print(channel_A3_mV);
  Serial.println("mA");


}
