#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

//Argc is the command lines argument count
// argv[0] is the name of this program
// argv[1] is the first argument given
// .
// .
// .
// argv[n] is the nth argument given
//My proposal for communication between ROS node and BBB for this project:
//	argv[1] 	should be the I2C address to be queried
//	argv[2] 	is a read/!(write) bit
//	argv[3] 	should probably be the subdevice/function we're interested in
//		If we need more space to transmit this information, adjust accordingly
//	argv[4 - n] the arguments to be passed to the Arduino, or used by the BBB

int main(int argc, char *argv)
{
  //The psuedo-pointer to the I2C buffer
	int file;
	//Miscellaneous ints used for control of script
	int random = 0, i = 0, delay = 100000, len = 0;
	//The I2C address used for communication
	int address = 0x00;
	//True/False, is this a read operation
	//  If true: read
	//  If false: write
	int read 	= 0;
	//The write buffer used to store the message
	char write_buffer[1] = {0x00};
	//The testing string
	char *testing = "testing";

	//srand not currently used at this time, probably to be removed
	srand(time(NULL));

	//Check to see if the I2C device can be opened for Read/Write operations
	if((file = open("/dev/i2c-1", 0_RDWR)) < 0)
	{
		//BBB errored out on I2C Open operation in Main
		return 0x210103;
	}

	//Set up the I2C communication between the slave at address
	if(ioctl(file, I2C_SLAVE, address) < 0) 
	{
		//BBB error out on I2C Open operation in Main
		return 0x210103;
	}

	//Force initialization of i and len
	i = 0, len = strlen(testing);
	//Testing purposes, send string over I2C
	while (i < len)
	{
		//Set the writebuffer to the current letter
		writebuffer[0] = testing[i];
		//Write to the I2C connection established within file
		// Write the contents of writebuffer, which is 1 byte
		// Theoretically, we should be able to batch write
		// I have no been able to get that to work with the write() function
		//   as it finishes with a STOP/NACK signal, which cuts off any batch?
		write(file, writebuffer, 1);
		//Move to next letter
		i++;
		//Since we're not being able to take advantage of the batch I2C
		// write, need to set a delay to give the Arduino time to process
		// the data before we send the next byte
		// The delay is in microseconds, 100x the Arduino's 100 milliseconds
		// I think we could increase this, if needed, but less would probably
		// cause problems
		usleep(delay);
	}
	//Message for debugging purposes
	printf("Finishing...\n");
	
	//Close I2C buffer to make sure nothing goes wrong further down the line...
	close(file);

	//return All Clear for this run
	return 0x000000;
}
