
//The library used for I2C communication
#include <Wire.h>

//Declare Roll,Pitch and Yaw variables
float rateRoll, ratePitch, rateYaw;

//Declare Calibration of Roll,Pitch and Yaw variables
float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
int rateCalibrationNumber;

//The library to read the PPM(Pulse Position Modulation) signals
#include <PulsePosition.h>
PulsePositionInput receiverInput(RISING);

//Declare the variables to store channel info
float receiverValue[] = {0,0,0,0,0,0,0,0};
int channelNumber = 0;

//Define the battery monitoring variables
float voltage,current,batteryRemaining,batteryAtStart;
float currentConsumed = 0;
float batteryDefault = 1300;

//Define the parameter containing the length of each control loop
uint32_t loopTimer;

//All variables necessary for the PID control loop including the P,I and D parameters
float desiredRateRoll, desiredRatePitch, desiredRateYaw;
float errorRateRoll, errorRatePitch, errorRateYaw;
float inputRoll, inputThrottle, inputPitch, inputYaw;
float prevErrorRateRoll, prevErrorRatePitch, prevErrorRateYaw;
float prevItermRateRoll, prevItermRatePitch, prevItermRateYaw;
float PIDReturn[] = {0,0,0};
//Define P,I and the D constants for the roll,pitch and yaw rotation rates
float PRateRoll = 0.6;
float PRatePitch = 0.6;
float PRateYaw = 2;
float IRateRoll = 3.5;
float IRatePitch = 3.5;
float IRateYaw = 12;
float DRateRoll = 0.03;
float DRatePitch = 0.03;
float DRateYaw = 0;

//Declare input variables to the motors
float motorInput1, motorInput2, motorInput3, motorInput4;

//Define the accelerometer variables
float accX, accY, accZ;
float angleRoll, anglePitch;

//Define the predicted angles and the uncertainties
//0 is the prediction of the initial angle and 2 * 2 is the uncertainty of the initial angle
float kalmanAngleRoll = 0;
float kalmanUncertaintyAngleRoll = 2 * 2;

float kalmanAnglePitch = 0;
float kalmanUncertaintyAnglePitch = 2 * 2;

//Initialize the output of the filter(Angle prediction, Uncertainty of the prediction)
float kalman1DOutput [] = {0,0};

//Define the desired roll and pitch angles and corresponding errors for the outer loop PID controller
float desiredAngleRoll, desiredAnglePitch;
float errorAngleRoll, errorAnglePitch;

//Define the values necessary for the outer loop PID controller, including the P,I and D parameters
float prevErrorAngleRoll, prevErrorAnglePitch;
float prevItermAngleRoll, prevItermAnglePitch;

float PAngleRoll = 2;
float PAnglePitch = 2;
float IAngleRoll = 0;
float IAnglePitch = 0;
float DAngleRoll = 0;
float DAnglePitch = 0;

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

//Measure the voltage and current of the circuit
void batteryVoltage(void)
{
  voltage = (float)analogRead(A2)/62;
  current = (float)analogRead(A1)*0.089;
}

//Define a function to read the receiver data
void readReceiver(void)
{
  channelNumber = ReceiverInput.available();
  
  if(channelNumber > 0)
  {
    for(int i=1;i<=channelNumber;i++)
    {
      receiverValue[i-1] = ReceiverInput.read(i);
    }
  }
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

//PID calculation function
void pidEquation(float error, float P, float I, float D, float prevError, float prevIterm)
{
  //A length of one iteration equals 250Hz which means Ts = 4ms 
  float Pterm = P*error;
  float Iterm = prevIterm + I * (error + prevError) * 0.004 / 2;

  //400 and -400 are to avoid integral windup
  if(Iterm > 400)
  {
    Iterm = 400;
  }
  else if(Iterm < -400)
  {
    Iterm = -400;
  }

  //A length of one iteration equals 250Hz which means Ts = 4ms 
  float Dterm = D * (error - prevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;

  //400 and -400 are to avoid integral windup
  if(PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if(PIDOutput < -400)
  {
    PIDOutput = -400;
  }

  //Return output from PID function
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = error;
  PIDReturn[2] = Iterm;

}

//PID reset function
void resetPID(void)
{
  prevErrorRateRoll = 0;
  prevErrorRatePitch = 0;
  prevErrorRateYaw = 0;

  prevItermRateRoll = 0;
  prevItermRatePitch = 0;
  prevItermRateYaw = 0;

  //Reset the PID error and integral values for the outer PID loop as well
  prevErrorAngleRoll = 0;
  prevErrorAnglePitch = 0;

  prevItermAngleRoll = 0;
  prevItermAnglePitch = 0;
}

void setup() {
  
  //Visualize the setup phase using the red LED
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);

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

  //Pin and hertz for motor control system
  analogWriteFrequency(5,250); //Motor 1
  analogWriteFrequency(9,250); //Motor 2 //New pin attachment to 9. pin PWM to motor
  analogWriteFrequency(10,250); //Motor 3 //New pin attachment to 10. pin PWM to motor
  analogWriteFrequency(11,250); //Motor 4 //New pin attachment to 11. pin PWM to motor
  analogWriteResolution(12);

  //External Green light to Control
  pinMode(6,OUTPUT);
  digitalWrite(6,HIGH);

  //Illimunates the Green LED to show setup process is finished
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

  //Avoid accidental lift off after the setup process
  receiver.Input.begin(3); //Connection to radio receiver is at pin 3
  while(receiverValue[2] < 1020 || receiverValue[2] > 1050)
  {
    readReceiver();
    delay(4);
  }

  //Start the timer 
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

  //Read the receiver data 
  readReceiver();

  //Calculate the desired angles from the receiver values
  desiredAngleRoll = 0.10 * (receiverValue[0] - 1500);
  desiredAnglePitch = 0.10 * (receiverValue[1] - 1500);

  inputThrottle = receiverValue[2];
  desiredRateYaw = 0.15 * (receiverValue[3]-1500);

  //Calculate the difference between the desired and the actual roll and pitch angles
  errorAngleRoll = desiredAngleRoll - kalmanAngleRoll;
  errorAnglePitch = desiredAnglePitch - kalmanAnglePitch;

  //To calculate the difference between the desired and the actual roll and pitch angles, using PID calculation function
  pidEquation(errorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, prevErrorAngleRoll, prevItermAngleRoll);

  desiredRateRoll = PIDReturn[0];
  prevErrorAngleRoll = PIDReturn[1];
  prevItermAngleRoll = PIDReturn[2];

  pidEquation(errorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, prevErrorAnglePitch, prevItermAnglePitch);

  desiredRatePitch = PIDReturn[0];
  prevErrorAnglePitch = PIDReturn[1];
  prevItermAnglePitch = PIDReturn[2];

  //Calculate the difference between the desired and the actual roll, pitch and yaw rotation rates. Use these for the PID controller of the inner loop
  errorRateRoll = desiredRateRoll - rateRoll;
  errorRatePitch = desiredRatePitch - ratePitch;
  errorRateYaw = desiredRateYaw - rateYaw;

  pidEquation(errorRateRoll, PRateRoll, IRateRoll, DRateRoll, prevErrorRateRoll, prevItermRateRoll);
  
  inputRoll = PIDReturn[0];
  prevErrorRateRoll = PIDReturn[1];
  prevItermRateRoll = PIDReturn[2];

  pidEquation(errorRatePitch, PRatePitch, IRatePitch, DRatePitch, prevErrorRatePitch, prevItermRatePitch);
  
  inputPitch = PIDReturn[0];
  prevErrorRatePitch = PIDReturn[1];
  prevItermRatePitch = PIDReturn[2];

  pidEquation(errorRateYaw, PRateYaw, IRateYaw, DRateYaw, prevErrorRateYaw, prevItermRateYaw);
  
  inputYaw = PIDReturn[0];
  prevErrorRateYaw = PIDReturn[1];
  prevItermRateYaw = PIDReturn[2];

  //Limit the throttle value to %80
  if(inputThrottle > 1800)
  {
    inputThrottle = 1800;
  }

  //Use the quadcopter dynamics equations
  motorInput1 = 1.024 * (inputThrottle - inputRoll - inputPitch - inputYaw);
  motorInput2 = 1.024 * (inputThrottle - inputRoll + inputPitch + inputYaw);
  motorInput3 = 1.024 * (inputThrottle + inputRoll + inputPitch - inputYaw);
  motorInput4 = 1.024 * (inputThrottle + inputRoll - inputPitch + inputYaw);

  //Limit the maximal power commands sent to the motors to avoid overloading them
  if(motorInput1 > 2000)
  {
    motorInput1 = 1999;
  }

  if(motorInput2 > 2000)
  {
    motorInput2 = 1999;
  }

  if(motorInput3 > 2000)
  {
    motorInput3 = 1999;
  }

  if(motorInput4 > 2000)
  {
    motorInput4 = 1999;
  }

  //Keep the quadcopter motors running at minimally %18 power during flight
  //To avoid stopping the motors in mid-flight keep them turning at %18 when the motor input decreases below 1180 microseconds
  int throttleIdle = 1180;

  if(motorInput1 < throttleIdle)
  {
    motorInput1 = throttleIdle;
  }

  if(motorInput2 < throttleIdle)
  {
    motorInput2 = throttleIdle;
  }

  if(motorInput3 < throttleIdle)
  {
    motorInput3 = throttleIdle;
  }

  if(motorInput4 < throttleIdle)
  {
    motorInput4 = throttleIdle;
  }

  //Make sure you are able to turn off the motors
  int throttleCutOff = 1000;

  if(receiverValue[2] < 1050)
  {
    motorInput1 = throttleCutOff;
    motorInput2 = throttleCutOff;
    motorInput3 = throttleCutOff;
    motorInput4 = throttleCutOff;
    resetPID();
  }

  //Sent the commands to the motors
  analogWrite(5,motorInput1);
  analogWrite(9,motorInput2);
  analogWrite(10,motorInput3);
  analogWrite(11,motorInput4);

  //Calculate the battery capacity during flight
  batteryVoltage();
  currentConsumed = Current * 1000 * 0.004 / 3600 + currentConsumed;
  batteryRemaining = (batteryAtStart - currentConsumed) / batteryDefault * 100;
  if(batteryRemaining <= 30)
  {
    //Red LED
    digitalWrite(7,HIGH);
  }
  else
  {
    digitalWrite(7,LOW);
  }

  //Finish the 250Hz control loop
  while(micros() - loopTimer < 4000);
  loopTimer = micros();

}
