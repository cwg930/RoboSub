#include <Wire.h>

//For the different Arduino's, change this to 
//  something not-conflicting
#define ADDR 0x53

//Initialize the message buffers
// Each can hold up to 20 bytes
char write[20] = { 0 };
char read[20]  = { 0 };

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
  delay(100);
}

//The count of messages to be read
int read_count  = 0;
//Count of messages to be written 
int write_count = 0;
int i = 0;
//The Arduino has detected Master asking for a READ.
//  The Arduino is the slave, so it should be unlikely for a
//  a READ as the first thing to occur. So, a WRITE should have
//  already been done, so based on the data gathered after the
//  previous WRITE, write the requested data to the Master
void requestEvent()
{
  Serial.println("Writting");
  //Check that there's a response to Master.
  if (write_count > 3 && i < write_count)
  {
     Wire.write(write[i++]);
  }
  
  if (i == write_count)
  {
    Serial.println("Write finished");
    Serial.println(i, DEC);
    Serial.println(write_count, DEC);
    i = write_count = 0;
  }
  
}

//Because of the limitations on batch write on the BBB-side of the transmission,
//  we'll only be reading a single byte at a time
void receiveEvent(int n)
{
  Serial.println("Reading...");
  //The first byte sent is the count of bytes in the message
  //  After comes the function we want performed
  //  Then come the parameters to the function
  if (Wire.available())
  {
    read[read_count++] = Wire.read();
    Serial.print("Read:");
    Serial.println(read[read_count-1], HEX);
    //If the read_count is equal to the total
    //  expected bytes sent, the message is finished
    //  send to lookup to perform the action and start filling
    //  out the write buffer
    Serial.println(read_count);
    if (read_count == read[0])
    {
      Serial.println("Lookup the function");
      lookup();
    }
    Serial.println(read_count);
  }
}

void lookup()
{
  
  switch(read[1])
  {
    case 0:
    
    break;
    case 1:
      //Testing, just copy read to write
      for (i = 0; i < read_count; i++)
        write[i] = read[i];
      
      i = 0;
      write_count = read_count;
      
      clean_read_buffer();
    break;
    //...
    default:
      //Probably not going to be used...?
	  //Error of some kind?
    break;
  }
}

//Cleans the buffer, also has side-effect of setting *_count
// to 0
void clean_read_buffer()
{
  for( ;read_count > 0; read_count--)
  {
    read[read_count] = 0;
  }
  read[read_count] = 0;
}

void clean_write_buffer()
{
  int i = 0;
  for( ; i < 20; i ++)
  {
    write[i] = 0;
  }
}
