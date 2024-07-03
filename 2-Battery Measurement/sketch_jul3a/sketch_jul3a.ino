
float voltage;
sensorPin = A6;

void battery_voltage(void)
  {
    voltage = (float)analogRead(sensorPin)/62;
  }

void setup() {
  
  serial.begin(9600);

}

void loop() {
  
  battery_voltage();
  Serial.print(voltage);
  Serial.println("V");
  delay(50);

}
