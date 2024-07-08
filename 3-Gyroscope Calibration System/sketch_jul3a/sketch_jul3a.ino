
//The library used for I2C communication
#include <Wire.h>

//Declare Roll,Pitch and Yaw variables
float rateRoll, ratePitch, rateYaw;

//Declare Calibration of Roll,Pitch and Yaw variables
float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
int rateCalibrationNumber;

//Start I2C communication with the gyro
void gyroSignals(void)
{

  //MPU6050 Unique Address
  Wire.beginTransmission(0x68);

  //Low pass filter register map
  Wire.write(0x1A);

  //Cut off frequency of 10 hertz of a low pass filter 
  Wire.write(0x05);
  Wire.endTransmission();

  //Set the sensitivity scale factor of the sensor
  Wire.beginTransmission(0x68);

  //The adress of the scale factor
  Wire.write(0x1B);

  //Hexidecimal value for gyro config
  Wire.write(0x8);
  Wire.endTransmission();

  //Access register storing gyro measurements
  Wire.beginTransmission(0x68);

  //Register hex number for gyroscope measurements
  Wire.write(0x43);
  Wire.endTransmission();

  //This request 6 bytes from MPU6050 (0x68) was already unique address
  Wire.requestFrom(0x68,6);

  //Read the gyro measurements around the X axis
  int16_t gyroX = Wire.read()<<8 | Wire.read();

  //Read the gyro measurements around the Y axis
  int16_t gyroY = Wire.read()<<8 | Wire.read();

  //Read the gyro measurements around the Z axis
  int16_t gyroZ = Wire.read()<<8 | Wire.read();

  //Convert the measurements unitys to in degrees per seconds ( °/s )
  //65.5 value is LSB sensitivity scale factor of MPU 6050
  rateRoll = (float)gyroX/65.5;
  ratePitch = (float)gyroY/65.5;
  rateYaw = (float)gyroZ/65.5;
}

void setup() {
  
  //57600: Good balance of speed and stability for Arduino Nano for faster data transmission
  Serial.begin(57600);

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  //Set the clock speed of I2C
  //MPU6050 supports I2C communications at up to 400kHz 
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  //Start the gyro in power mode
  Wire.beginTransmission(0x68);
  //Power management register(hex) is 6B
  Wire.write(0x6B);
  //All bits in register have to be set to 0 in order for the device start and continue in power mode
  Wire.write(0x00);

  Wire.endTransmission();

  //Perform the calibration measurements
  for(rateCalibrationNumber = 0 ; rateCalibrationNumber < 2000 ; rateCalibrationNumber++)
  {
    gyroSignals();
    rateCalibrationRoll += rateRoll;
    rateCalibrationPitch += ratePitch;
    rateCalibrationYaw += rateYaw;
    delay(1);
  }

  //Calculate the calibration values
  rateCalibrationRoll /= 2000;
  rateCalibrationPitch /= 2000;
  rateCalibrationYaw /= 2000;
}

void loop() {
  
  //Call the predefined function to read the gyro measurements
  gyroSignals();

  //Correct the measured values
  rateRoll -= rateCalibrationRoll;
  ratePitch -= rateCalibrationPitch;
  rateYaw -= rateCalibrationYaw;

  //Print them all on the serial monitor
  Serial.print("Roll rate [°/s] = ");
  Serial.print(rateRoll);

  Serial.print("Pitch rate [°/s] = ");
  Serial.print(ratePitch);

  Serial.print("Yaw rate [°/s] = ");
  Serial.println(rateYaw);

  delay(500);

}
