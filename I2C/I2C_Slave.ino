#include <Wire.h>

//For the different Arduino's, change this to 
//  something not-conflicting
#define ADDR 0x53

int led = 13;


void setup()
{
  //For testing purposes, set up Serial communication
  //  If the development will not be using Serial, remove
  Serial.begin(9600);
  //Join I2C bus with address of ADDR
  Wire.begin(ADDR);
  //Wire.onRequest() is an event register for requests
  //  , only needs called once
  Wire.onRequest(requestEvent);
  //Wire.onRequest wires up the response to a READ request
  // Wire.onReceive wires up the response to a WRITE request
  //Quite embarassed it took me this long to figure out that 
  // bit of semantic difference...
  Wire.onReceive(receiveEvent);
}

void loop()
{
  //Delay, change value as needed, but make sure to change corresponding delay in 
  // the BBB I2C_Master code
  delay(100);
}

//For testing purposes, will be used to just send back a 
// character from the range of 32 - 255
int offset = 0;

//Sends a character, and then moves to the next character
//  When offset reaches 255, wrap back to 32
//  Makes sure that only visible characters are sent
void requestEvent()
{
  //Increment offset through the visible characters
  int x = offset++ % 224 + 32;
  //Snark, remove before production
  Serial.print("I wish to serve Master...");
  //Write the character to the I2C buffer
  Wire.write(x);
  //For testing purposes, write the char to the Serial Monitor
  Serial.println(x);

}

//Because of the limitations on batch write on the BBB-side of the transmission,
//  we'll only be reading a single byte at a time
void receiveEvent(int n)
{
  int x = 0;
  //Testing for what n is, it seems to be a count of the number
  //  of the data being sent over I2C? Only seen 1 so far
  Serial.print("The n is: ");
  Serial.println(n);
  
  //Check if there is info over the wire, there should be
  //  Real use will be for when we get batch writes working
  //  The line will change to:
  //  while(Wire.available(){ 
  if (Wire.available())
    //Read the data on the line
    x = Wire.read();
  
  
  //For testing purposes print it back to the Serial Monitor
  // Ideally, we'd end up actually doing something with this...
  // Once we figure out what to do, remove this and replace it with that
  Serial.print("x is: ");
  Serial.println(x);
  Serial.println("Is this correct?");
}
