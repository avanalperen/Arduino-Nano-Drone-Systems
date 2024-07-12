
#include <Wire.h>

//Define the pressure sensor calibration values (Trimming Parameters)
uint16_t digT1, digP1;
int16_t digT2, digT3, digP2, digP3, digP4, digP5;
int16_t digP6, digP7, digP8, digP9;

//Define the altitude variables
float altitudeBarometer, altitudeBarometerStartUp;
int rateCalibrationNumber;

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

}

void loop() {
  
  //Read the barometer and print the altitudes
  barometerSignals();
  //Substract the average startup altitude to get the altitude variation in Flight
  altitudeBarometer -= altitudeBarometerStartUp;
  Serial.print("Altitude [cm] : ");
  Serial.println(altitudeBarometer);
  delay(50);

}
