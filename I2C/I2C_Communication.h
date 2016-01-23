#include "ByteBuffer.h"

#ifndef I2C_Communication
#define I2C_Communication
#endif

class I2C_Communication {
    public:
        I2C_Communication(int bus, int address);
        I2C_Communication();
        ~I2C_Communication();
        int getBus();
        void setBus(int bus);
        int getAddress();
        void setAddress(int address);
        //ByteBuffer getBuffer();
        //void setBuffer(char* buffer);
        bool open(int address);
        int close();
        char readByte();
        char readRegister(int register);
        int writeByte(char byte);
        int writeRegister(int register, char byte);
        ByteBuffer readBytes(int count);
        ByteBuffer readRegisters(int start, int count);
        int writeBytes(ByteBuffer bytes);
        int writeRegisterRange(int start, int count, ByteBuffer buffer);
        virtual void package(ByteBuffer);
    private:
        int I2C_File;
        int I2C_Delay;
        unsigned int I2C_Address;
        char I2C_Bus[10];
}

I2C_Communication::I2C_Communication(int bus, int address, int delay=0)
{
    switch(bus)
    {
        case 1:
            //Pin 9_20/9_19, for some reason called i2c-2
            strcpy(this->I2C_Bus, "/dev/i2c-1");
            break;
        case 2:
            //Pin 9_18/8_17, for some reason called i2c-1
            strcpy(this->I2C_Bus, "/dev/i2c-2");
            break;
    }
    this->I2C_Address = address;
    this->I2C_Delay = delay;
}

I2C_Communication::I2C_Communication()
{
    this->I2C_Address = -1;
    this->I2C_Delay = 0;
}

~I2C_Communication::I2C_Communication() { }

int I2C_Communication::getBus()
{
    return this->I2C_Bus;
}

void I2C_Communication::setBus(int bus)
{
    this->I2C_Bus = bus;
}

int I2C_Communication::getAddress()
{
    return this->I2C_Address;
}

void I2C_Communication::setAddress(int address)
{
    this->I2C_Address = address;
}

ByteBuffer I2C_Communication::getBuffer() { }

bool I2C_Communication::open(int address)
{
    this->I2C_File = open(this->I2C_Bus, O_RDWR);
    if(this->I2C_File < 0)
        return false;
    int cntrl = ioctl(this->I2C_File, I2C_Slave, address);
    return this->I2C_File > 0 && cntrl > 0;
}

void I2C_Communication::close()
{
    close(this->I2C_File);
}

char I2C_Communication::readByte()
{
    char data[1];

    if (read(this->I2C_File, data, 1) != 1)
    {
        exit(0x210303);
    }
    return data[0];
}

char I2C_Communication::readRegister(int register)
{
    char data[1];

    //I think the third argument is for the register number
    if(read(file, data, register) != 1)
    {
        exit(0x210303);
    }

    return data;
}

ByteBuffer readBytes(int size)
{
    int i = 0;
    ByteBuffer buffer;
    char data[1];

    while (i < size)
    {
        if(read(this->I2C_File, data, 1 + i) != 1)
            return 0x210303;
        buffer.addByte(data[0]);
        i++;
        if(this->I2C_Delay > 0)
            usleep(this->I2C_Delay);
    }
    return buffer;
}

ByteBuffer readRegisterRange(int start, int size)
{
    int i = 0;
    ByteBuffer buffer;
    char data[1];

    while(i < size)
    {
        if(read(this->I2C_File, data, start+i) != 1)
            return 0x210303;
        buffer.addByte(data[0]);
        i++;
        if(this->I2C_Delay > 0)
            usleep(this->I2C_Delay);
    }

    return buffer;
}

int writeByte(char byte)
{
    if(write(this->I2C_File, &byte, 1) != 1))
        return 0x210202;
    return 0;
}

int writeRegister(int register, char bytes)
{
    if(write(this->I2C_File, &bytes, register) != 1))
        return 0x210202;
    return 0;
}

int writeBytes(ByteBuffer bytes)
{
    int i = 0;
    int size = bytes.getSize();

    while (i < size)
    {
        char data = bytes.getByte();
        if(write(this->I2C_Communication, &data, 1+i) != 1)
        {
            return 0x210202;
        }
        i++;
        if(this->I2C_Delay)
            usleep(this->I2C_Delay);
    }

    return 0;
}

int writeRegisterRange(int start, int size, ByteBuffer buffer)
{
    int i = 0;

    while (i < size)
    {
        char data = bytes.getByte();
        if(write(this->I2C_Communication, &data, start+i) != 1)
        {
            return 0x210202;
        }
        i++;
        if(this->I2C_Delay)
            usleep(this->I2C_Delay);
    }

    return 0;
}
