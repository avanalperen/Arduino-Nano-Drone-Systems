//Define the battery monitoring variables
float voltage,current,batteryRemaining,batteryAtStart;
float currentConsumed = 0;
float batteryDefault = 1300;

//Measure the voltage and current of the circuit
void batteryVoltage(void)
{
  voltage = (float)analogRead(A2)/62;
  current = (float)analogRead(A1)*0.089;
}
void setup() {
  
  //Green led pin
  digitalWrite(7,HIGH);
  batteryVoltage();
  if(voltage > 8.3)
  {
    digitalWrite(7,LOW);
    batteryAtStart = batteryDefault;
  }
  else if(voltage < 7.5)
  {
    batteryAtStart = batteryDefault * 30 /100;
  }
  else 
  {
    digitalWrite(7,LOW);
    batteryAtStart = batteryDefault * (82 * voltage - 580) / 100;
  }

}

void loop() {

  //Calculate the battery capacity during flight
  batteryVoltage();
  currentConsumed = Current * 1000 * 0.004 / 3600 + currentConsumed;
  batteryRemaining = (batteryAtStart - currentConsumed) / batteryDefault * 100;
  if(batteryRemaining <= 30)
  {
    digitalWrite(7,HIGH);
  }
  else
  {
    digitalWrite(7,LOW);
  }
  


}
