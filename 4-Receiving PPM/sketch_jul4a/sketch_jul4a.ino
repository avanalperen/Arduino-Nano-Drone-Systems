//The library to read the PPM(Pulse Position Modulation) signals
#include <PulsePosition.h>
PulsePositionInput receiverInput(RISING);

//Declare the variables to store channel info
float receiverValue[] = {0,0,0,0,0,0,0,0};
int channelNumber = 0;

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

}

void loop() {
  
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
