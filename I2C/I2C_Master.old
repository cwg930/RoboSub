#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

//If there are any questions, comments, concerns
// Author: Timothy Simmons
// Email : tsimmons15@knights.ucf.edu
// Last edit 16.01.2016

//Argc is the command lines argument count
// argv[0] is the name of this program
// argv[1] is the first argument given
// .
// .
// .
// argv[n] is the nth argument given
//My proposal for communication between ROS node and BBB for this project:
//	argv[1] 	for simplicity, the count of the bytes to be sent
//	argv[2] 	is the address for communication (currently not implemented)
//	argv[3] 	the function we're intending the arduino to execute
//	argv[4 - n] the arguments to be passed to the Arduino, or used by the BBB
//
//  The function which the arduino passes this list to will know what
//   to do with any of the information, so it's just copied straight over
//   to the arduino. Unless the BBB needs to preprocess any data.

//Currently assumes the single address, once we're ready to put in production
//  this will be removed, or edited.
#define ADDR 	    0x53
#define DELAY 	    10000 //Delay is currently just making sure they sync up
						  //If there's an easy way to make sure they sink,
						  // feel free to edit

//Writes to the I2C bus
int  write_data(int file, char *data);
//Reads data from the I2C bus
char *read_data(int file);

int main(int argc, char **argv)
{
   int file;
   int count = 0, err_no = 0, i = 0;
   //The 'writebuffer' is a linked list, called queue
   char *write = NULL;
   char *read = NULL;
   fprintf(stderr, "Starting...\n");
   //Fill queue with the data to be sent, using argv currently
   //There should be at least 3 values in argv
   if (argc < 3)
   {
      fprintf(stderr, "Incorrect call\n");
      fprintf(stderr, "Call should be:\n");
      fprintf(stderr, "%s <count> <address> <function> <arg1> <arg2> <arg ...>\n", argv[0]);
      //There was a problem sending command to BBB
      return 0x220104;
   }

   //The work involved is a good argument for switching away from 
   // using the argv input method.
   //Initialize the queues
   //The first string in argv is the name of the program
   // Skip that and read in argv[1], which should be the count of bytes
   count = strtol(argv[1], NULL, 0);
   write = calloc(count, sizeof(char));
   //Save the count of bytes
   write[0] = count;
   //Get the rest of the data...
   i = 1;
   while (i < write[0])
   {
      write[i] = strtol(argv[i+1], NULL, 0);
      i++;
   }

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

   //Write to the Arduino
   write_data(file, write);
   //Give the arduino time to process
   usleep(DELAY);

   //Read response
   if ((read = read_data(file)) == NULL)
   {
      fprintf(stderr, "Read returned NULL\n");
      return 0x210303;
   }
   fprintf(stderr, "The contents read back...\n");
   fprintf(stderr, "Count: %i\n", read[0]);
   i = 1;
   while (i < read[0])
   {
      fprintf(stderr, "%i) %i\n", i, read[i]);
      i++;
   }
   //Message for debugging purposes
   printf("Finishing...\n");

   //Close I2C file, to make sure nothing goes wrong further down the line...
   close(file);

   //return could be used as communication from BBB to ROS nodes
   // All clear!
   return 0;
}

int write_data(int file, char *data)
{
   int i = 0;
   
   while (i < data[0])
   {
       if (write(file, &data[i], 1) != 1)
       {
           fprintf(stderr, "Error...\n");
           exit(0x210202);
       }
       i++;
       //Could possibly use a shorter delay
       usleep(DELAY);
   }
   return 0;
}

char *read_data(int file)
{
   int i = 1;
   char size = 0;
   char *data = NULL;

   if (read(file, &size, 1) != 1)
   {
      fprintf(stderr, "Error reading...\n");
      exit(0x210303);
   }
   data = calloc(size, sizeof(char));
   data[0] = size;
   while (i < data[0])
   {
      if(read(file, &data[i], 1) != 1)
      {
         fprintf(stderr, "Error reading...\n");
         exit(0x210303);
      }
      i++;
      //Could possibly use shorter delay
      usleep(DELAY);
   }

   return data;
}
