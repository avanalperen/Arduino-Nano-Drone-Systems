
//The library used for I2C communication
#include <Wire.h>

//Declare Roll,Pitch and Yaw variables
float rateRoll, ratePitch, rateYaw;

//Define the accelerometer variables
float accX, accY, accZ;
float angleRoll, anglePitch;

//Define the acceleration and velocity variables
float accZInertial;
float velocityVertical;

//Define the parameter containing the length of each control loop
float loopTimer;

//Start I2C communication with the gyro
//Because it was first made for Gyroscope Calibration System and then to Accelerometer configuration some hexidecimal register map values are changed 
void gyroSignals(void)
{

  //MPU6050 Unique Address
  Wire.beginTransmission(0x68);

  //Low pass filter register map
  Wire.write(0x1A);

  //Cut off frequency of 10 hertz of a low pass filter 
  Wire.write(0x05);
  Wire.endTransmission();

  //Configure the Accelerometer output
  //Set the sensitivity scale factor of the sensor
  Wire.beginTransmission(0x68);

  //The adress of the scale factor (Changed to B to C)
  Wire.write(0x1C);

  //Hexidecimal value for accelerometer config (Changed to 8 to 10)
  Wire.write(0x10);
  Wire.endTransmission();

  //Pull the accelerometer measurements from the sensor
  //Access register storing accelerometer measurements
  Wire.beginTransmission(0x68);

  //Register hex number for accelerometer measurements (Changed to 43 to 3B)
  Wire.write(0x3B);
  Wire.endTransmission();

  //This request 6 bytes from MPU6050 (0x68) was already unique address
  Wire.requestFrom(0x68,6);

  //Read the accelerometer measurements around the X axis
  int16_t accXLSB = Wire.read()<<8 | Wire.read();

  //Read the accelerometer measurements around the Y axis
  int16_t accYLSB = Wire.read()<<8 | Wire.read();

  //Read the accelerometer measurements around the Z axis
  int16_t accZLSB = Wire.read()<<8 | Wire.read();

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

  //Convert the measurements to physical values, for this LSB 4096 LSB per g is used
  //Last calibrations added
  accX = (float)accXLSB/4096 - 0.03;
  accY = (float)accYLSB/4096;
  accZ = (float)accZLSB/4096 - 0.02;

  //Calculate the absolute angles , because arduino gives the atan result in radians , it need to be converted to degrees so multiplied by (3.142 / 180) will convert it to degree.
  angleRoll = atan(accY / sqrt(accX*accX + accZ*accZ)) * (180 / 3.142);
  anglePitch = -atan(accX/sqrt(accY*accY + accZ*accZ)) * (180 / 3.142);
}

void setup() 
{
  
  //Communication with the gyroscope and calibration
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

}

void loop() 
{
  
  gyroSignals();

  //Calculate the acceleration in the inertial Z axis
  accZInertial = -sin(anglePitch * (3.142 / 180)) * accX + cos(anglePitch * (3.142 / 180)) * sin(angleRoll * (3.142 / 180)) * accY + cos(anglePitch * (3.142 / 180)) * cos(angleRoll * (3.142 / 180)) * accZ;

  //Unit of Acceleration is g so convert the acceleration to cm/s2
  accZInertial = (accZInertial - 1) * 9.81 * 100;

  //Calculate and print the vertical velocity
  velocityVertical = velocityVertical + accZInertial * 0.004;

  //Instead of using this
  /*
  Serial.print("Vertical velocity [cm/s] : ");
  Serial.println(velocityVertical);
  */

  //Use this To read serial plotter
  Serial.print("Vertical_velocity:");
  Serial.println(velocityVertical);

  while(micros() - loopTimer < 4000)
  {
    loopTimer = micros();
  }
}
