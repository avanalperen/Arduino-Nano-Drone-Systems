
//The library used for I2C communication
#include <Wire.h>

//Define the Gyroscope and Accelerometer variables
//Declare Roll,Pitch and Yaw variables
float rateRoll, ratePitch, rateYaw;

//Declare Calibration of Roll,Pitch and Yaw variables
float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
int rateCalibrationNumber;

//Define the accelerometer variables
float accX, accY, accZ;
float angleRoll, anglePitch;

//Define the parameter containing the length of each control loop
uint32_t loopTimer;

//Define the predicted angles and the uncertainties
//0 is the prediction of the initial angle and 2 * 2 is the uncertainty of the initial angle
float kalmanAngleRoll = 0;
float kalmanUncertaintyAngleRoll = 2 * 2;

float kalmanAnglePitch = 0;
float kalmanUncertaintyAnglePitch = 2 * 2;

//Initialize the output of the filter(Angle prediction, Uncertainty of the prediction)
float kalman1DOutput [] = {0,0};

//Function that calculates predicted angle and uncertainty using the Kalman equations

// 1 - Predict the current state of the system
// 2 - Calculate the uncertainty of the prediction
// 3 - Calculate the Kalman gain from the uncertainties on the predictions and measurements
// 4 - Update the predicted state of the system with the measurement of the state through the Kalman gain
// 5 - Update the uncertainty of the predicted state
void kalman1D(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMeasurement)
{
  kalmanState = kalmanState + 0.004 * kalmanInput;
  kalmanUncertainty = kalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  float kalmanGain = kalmanUncertainty * 1 / (1 * kalmanUncertainty + 3 * 3);

  kalmanState = kalmanState + kalmanGain * (kalmanMeasurement - kalmanState);

  kalmanUncertainty = (1-kalmanGain) * kalmanUncertainty;

  //Kalman Filter Output
  kalman1DOutput[0] = kalmanState;
  kalman1DOutput[1] = kalmanUncertainty;

  // kalmanInput contains the rotation rate measurement
  // kalmanMeasurement contains the accelerometer angle measurement
  // kalmanState contains the angle calculated with the Kalman filter
}

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

  //Gyro Measurement Part
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

  //Convert the measurements to physical values, for this LSB 4096 LSB per g is used
  //Last calibrations added
  accX = (float)accXLSB/4096 - 0.03;
  accY = (float)accYLSB/4096;
  accZ = (float)accZLSB/4096 - 0.02;

  //Calculate the absolute angles , because arduino gives the atan result in radians , it need to be converted to degrees so multiplied by (3.142 / 180) will convert it to degree.
  angleRoll = atan(accY / sqrt(accX*accX + accZ*accZ)) * 1 / (3.142 / 180);
  anglePitch = -atan(accX/sqrt(accY*accY + accZ*accZ)) * 1 / (3.142 / 180);
}

void setup() {
  
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
  loopTimer = micros();

}

void loop() {
  
  //Call the predefined function to read the gyro measurements
  gyroSignals();

  //Correct the measured values, Calculate the rotation rates
  rateRoll -= rateCalibrationRoll;
  ratePitch -= rateCalibrationPitch;
  rateYaw -= rateCalibrationYaw;

  //Start the iteration for the Kalman filter with the roll and pitch angles
  kalman1D(kalmanAngleRoll, kalmanUncertaintyAngleRoll, rateRoll, angleRoll);

  kalmanAngleRoll = kalman1DOutput[0];

  kalmanUncertaintyAngleRoll = kalman1DOutput[1];

  kalman1D(kalmanAnglePitch, kalmanUncertaintyAnglePitch, ratePitch, anglePitch); 

  kalmanAnglePitch = kalman1DOutput[0];

  kalmanUncertaintyAnglePitch = kalman1DOutput[1];

  //Print the predicted angle values
  Serial.print("Roll Angle [°] : ");
  Serial.print(kalmanAngleRoll);

  Serial.print("Pitch Angle [°] : ");
  Serial.println(kalmanAnglePitch);

  //For Serial Plotter test screen
  /*
  Serial.print(kalmanAngleRoll);
  Serial.print(",");
  Serial.println(kalmanAnglePitch);
  */

  while(micros() - loopTimer < 4000)
  {
    loopTimer = micros();
  }

}
