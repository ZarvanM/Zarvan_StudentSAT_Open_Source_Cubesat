void setup() 
{
pinMode(26,INPUT);
Serial.begin(9600);
analogReadResolution(12);
}

void loop() 
{
int data = analogRead(27);
float temp = analogReadTemp();

int LDR1_Value = analogRead(26);  // Read the value from LDR 1
int LDR2_Value = analogRead(27);  // Read the value from LDR 2

  // Normalize the readings if necessary
  float LDR1_Normalized = (float)LDR1_Value / 3800.0;
  float LDR2_Normalized = (float)LDR2_Value / 3800.0;
LDR1_Normalized = pow(LDR1_Normalized, 2);
LDR2_Normalized = pow(LDR2_Normalized, 2);
  // Calculate the angle
  float angle = atan2(LDR2_Normalized - LDR1_Normalized, 1.0) * (180.0 / PI);

if(LDR1_Value>1400 ||LDR2_Value>1400)
{

  Serial.print("TEMP: ");
  Serial.print(temp);
  Serial.print(" LDR1: ");
  Serial.print(LDR1_Value);
  Serial.print(" LDR2: ");
  Serial.print(LDR2_Value);
  Serial.print(" Angle: ");
  angle = angle+18;
  Serial.println(angle*2);
}
else
{
    Serial.println("OUTTA BOUNDS");

}
}
