
//The library used for I2C communication
#include <Wire.h>

//Declare Roll,Pitch and Yaw variables
float rateRoll, ratePitch, rateYaw;

//Define the accelerometer variables
float accX, accY, accZ;
float angleRoll, anglePitch;
float accZInertial;

//Define the parameter containing the length of each control loop
float loopTimer;

//Define the pressure sensor calibration values (Trimming Parameters)
uint16_t digT1, digP1;
int16_t digT2, digT3, digP2, digP3, digP4, digP5;
int16_t digP6, digP7, digP8, digP9;

//Define the altitude variables
float altitudeBarometer, altitudeBarometerStartUp;
int rateCalibrationNumber;

//The library that makes able to calculate with vectors and matrices
#include <BasicLinearAlgebra.h>

//Define the matrices for the two-dimensional Kalman filter
using namespace BLA;
float altitudeKalman, velocityVerticalKalman;

BLA::Matrix<2,2>F;BLA::Matrix<2,1>G;
BLA::Matrix<2,2>P;BLA::Matrix<2,2>Q;
BLA::Matrix<2,1>S;BLA::Matrix<1,2>H;
BLA::Matrix<2,2>I;BLA::Matrix<1,1>Acc;
BLA::Matrix<2,1>K;BLA::Matrix<1,1>R;
BLA::Matrix<1,1>L;BLA::Matrix<1,1>M;

//Create the function that holds the two dimensional Kalman filter
void kalman2D(void)
{
  acc = {accZInertial};
  S = F * S + G * acc;
  P = F * P * ~ F + Q;
  L = H * P * ~ H + R;
  K = P * ~ H * Invert(L);
  M = {altitudeBarometer};
  S = S + K * (M - H * S);
  altitudeKalman = S(0,0);
  velocityVerticalKalman = S(1,0);
  P = (I - K * H) * P;
}

//Make connection with the pressure sensor and read the raw uncombined pressure and temperature measurements
void barometerSignals(void)
{
  //BMP280 sensor i2c address
  Wire.beginTransmission(0x76);

  //Memory Map Register Address
  Wire.write(0xF7);
  Wire.endTransmission();

  //After comma 6 bytes to read the registers
  Wire.requestFrom(0x76,6);

  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();

  //Construct the raw temperature and pressure measurements
  //adcP = raw, uncompensated and uncalibratied pressure
  unsigned long int adcP = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);

  //adcT = temperature
  unsigned long int adcT = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

  //Construct the fine resolution temperature value (codes are given by the manufacturer of BMP280)
  signed long int var1, var2;

  var1 = ((((adcT >> 3) - ((signed long int)digT1 << 1))) * ((signed long int)digT2)) >> 11;

  var2 = (((((adcT >> 4) - ((signed long int)digT1)) * ((adcT >> 4) - ((signed long int)digT1))) >> 12) * ((signed long int)digT3)) >> 14;

  signed long int t_fine = var1 + var2;
  
  //Construct the compensated and calibrated pressure p (In Pascal) , (codes are given by the manufacturer in the datasheet)
  unsigned long int p;

  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;

  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)digP6);

  var2 = var2 + ((var1*((signed long int)digP5)) << 1);

  var2 = (var2 >> 2) + (((signed long int)digP4) << 16);

  var1 = (((digP3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3)+((((signed long int )digP2) * var1) >> 1)) >> 18;

  var1 = ((((32768 + var1)) * ((signed long int )digP1)) >> 15);

  if (var1 == 0) { p = 0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adcP)-(var2 >> 12))) * 3125;

  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}

  else { p = (p / (unsigned long int )var1) * 2;  }

  var1 = (((signed long int )digP9) * ((signed long int ) (((p >> 3) * (p >> 3)) >> 13))) >> 12;

  var2 = (((signed long int )(p >> 2)) * ((signed long int )digP8)) >> 13;

  p = (unsigned long int)((signed long int )p + ((var1 + var2 + digP7) >> 4));

  //Result of the calculation is a pressure in Pascal so need to be converted

  //Convert the pressure (calculate the altitude same time) multiply by 100 to make it cm
  double pressure =(double)p / 100;
  altitudeBarometer = 44330 * (1 - pow(pressure / 1013.25,1 / 5.255)) * 100;
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

  //Optimize the barometer for indoor navigation
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);

  //The corresponding full binary setting for measurement F4 is equal to a hexadecimal value 57
  Wire.write(0x57);
  Wire.endTransmission();

  //Setup the configuration register
  Wire.beginTransmission(0x76);
  Wire.write(0xF5);

  //The corresponding full binary setting for register F5 is equal to a hexadecimal value 57
  Wire.write(0x14);
  Wire.endTransmission();

  //Import the calibration parameters
  uint8_t data[24], i = 0;

  Wire.beginTransmission(0x76);

  //The register address of first trimming parameter
  Wire.write(0x88);
  Wire.endTransmission();

  //Request 24 byte so information can be pulled from 24 registers
  Wire.requestFrom(0x76,24);
  
  while(Wire.available())
  {
    data[i] = Wire.read();
    i++;
  }

  digT1 = (data[1] << 8) | data[0];
digT2 = (data[3] << 8) | data[2];
digT3 = (data[5] << 8) | data[4];

digP1 = (data[7] << 8) | data[6];
digP2 = (data[9] << 8) | data[8];
digP3 = (data[11] << 8) | data[10];
digP4 = (data[13] << 8) | data[12];
digP5 = (data[15] << 8) | data[14];
digP6 = (data[17] << 8) | data[16];
digP7 = (data[19] << 8) | data[18];
digP8 = (data[21] << 8) | data[20];
digP9 = (data[23] << 8) | data[22];

//Calculate the altitude reference level
for(rateCalibrationNumber = 0;rateCalibrationNumber < 2000;rateCalibrationNumber++)
{
  barometerSignals();
  altitudeBarometerStartUp += altitudeBarometer;
  delay(1);
}

altitudeBarometerStartUp /= 2000;

//Define the Kalman Matrix Values
F = {1, 0.004, 0, 1};
G = {0.5 * 0.004 * 0.004, 0.004};
H = {1,0};
I = {1,0,0,1};
Q = G * ~ G * 10 * 10;
R = {30 * 30};
P = {0,0,0,0};
S = {0,0};

loopTimer = micros();

}

void loop() {
  
  //Calculate the Kalman altitude and velocity
  gyroSignals();

  accZInertial = -sin(anglePitch * (3.142 / 180)) * accX + cos(anglePitch * (3.142 / 180)) * sin(angleRoll * (3.142 / 180)) * accY + cos(anglePitch * (3.142 / 180)) * cos(angleRoll * (3.142 / 180)) * accZ;
  accZInertial = (accZInertial-1) * 9.81 * 100;

  barometerSignals();

  altitudeBarometer -= altitudeBarometerStartUp;
  kalman2D();

  //Print the altitude and velocity
  Serial.print("Altitude [cm] : ");
  Serial.print(altitudeKalman);
  Serial.print("Vertical velocity [cm/s] : ");
  Serial.print(velocityVerticalKalman);

  while(micros() - loopTimer < 4000)
  {
    loopTimer = micros();
  }

}
