#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

#define ADDR 	    0x40
#define DELAY 	    100000

unsigned int mergeBytes(int upper, int lower);

int main(int argc, char **argv)
{
   int i = 0, x = 0;
   int file;
   int writeBuffer[1] = {0x00};
   char readBuffer[42] = {0x00};

   //Check to see if the I2C device can be opened for Read/Write operations
   if((file = open("/dev/i2c-1", O_RDWR)) < 0) {
	    fprintf(stderr, "failed to open the bus\n");
	    return 0x210101;
   }
   //Set up the I2C communication to be between the slave at address ADDR
   if(ioctl(file, I2C_SLAVE, ADDR) < 0) {
	    fprintf(stderr, "Failed to connect to the sensor\n");
	    return 0x210101;
   }

   while(x < 21)
   {
      writeBuffer[0] = x++;
      if(write(file, writeBuffer, 1) != 1)
      {
         return 1;
      }

      if(read(file, &readBuffer[i++], 2) != 2)
      {
         return 1;
      }
      i++;
   }
   i = x = 0;
   printf("The read values...\n");
   while(i < 42)
   {
      printf("#%02x\n", x);
      x++;
      printf("0x%04x\n", mergeBytes(readBuffer[i], readBuffer[i+1]));
      i += 2;
   }

   //close I2C file, to make sure nothing goes wrong further down the line...
   close(file);

   //return could be used as communication from BBB to ROS nodes
   // All clear!
   return 0;
}

unsigned int mergeBytes(int upper, int lower)
{
    return (upper << 8) + lower;
}
