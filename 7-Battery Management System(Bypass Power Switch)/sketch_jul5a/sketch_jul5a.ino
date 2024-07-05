
float Voltage;

void batteryVoltage(void)
{
  voltage = (float)analogRead(A2) / 62;
}

void setup() {
  
  if(voltage > 7.5)
  {
    digitalWrite(7,HIGH);
  }
  else
  {
    digitalWrite(7,LOW);
  }

}

void loop() {
  
  

}
