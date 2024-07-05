//Use the PulsePosition library
#include <PulsePosition.h>
PulsePositionInput receiverInput(RISING);

//Declare the variables to store channel info
float receiverValue[] = {0,0,0,0,0,0,0,0};
int channelNumber = 0;

//Define the throttle variable , %0 Throttle 1000 microseconds - %100 Throttle 2000 microseconds
float inputThrottle;

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

void setup() {
  
  Serial.begin(57600);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  ReceiverInput.begin(3)

  //Send PWM signals from Arduino Nano , Pin 5(which pin goes to esc and motor) - 250Hz(PWM frequency used for most esc 250Hz)
  analogWriteFrequency(5,250);

  //Set the PWM Frequency
  //Change Resolution to 12 bit
  analogWriteResolution(12);

  delay(250);

  //Avoid uncontrolled motor start 
  while(receiverValue[2] < 1020 || receiverValue[2] > 1050)
  {
    readReceiver();
    delay(4);
  }

}

void loop() {
  
  //Send the throttle input to the motor
  readReceiver();
  inputThrottle = receiverValue[2];
  //Pin 5(which pin goes to esc and motor) , convert the throttle value in microseconds to their 12 bit equivalent by multiply 1.024
  analogWrite(5,1.024 * inputThrottle);

  //Read and display the PPM information on the serial monitor
  readReceiver();

  Serial.print("Number of channels:");
  Serial.print(channelNumber);

  Serial.print(" Roll [µs] : ");
  Serial.print(receiverValue[0]);

  Serial.print(" Pitch [µs] : ");
  Serial.print(receiverValue[1]);

  Serial.print(" Throttle [µs] : ");
  Serial.print(receiverValue[2]);

  Serial.print(" Yaw [µs] : ");
  Serial.println(receiverValue[3]);

  delay(50);

}
