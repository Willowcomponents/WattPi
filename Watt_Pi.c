// Willow Components Watt Pi Demo code.
// Note ADC values read will be affected by the quality of the PSU supplying the boards.


#include "wiringPi.h"
#include "wiringPiI2C.h"


#include <stdlib.h>
#include <stdio.h>
#include <linux/i2c-dev.h>


#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


#define adc_address    0x68	// I2C address for ADC Default link settings
				// MCP3424 config register start converting | Channel 0 one shot mode | 18bit | x1 gain
				// 0x8C (Hex) 100011000 (bin)
				// bit 7   :  1=initiate conversion (single shot mode)  0= no effect
				// bit 6-5 : Channel:           0 - 3
				// bit 4   : conversion mode:   0 = single shot,  1= continuos
				// bit 3-2 : ADC Resolution :   0 = 12bits, 1 = 14bits, 2 = 16bits, 3 = 18bits
				// bit 1-0 : gain           :   00 = x1, 01 = x2, 10 = x4,  11 = x8

#define config_reg_ch0 0x8C	// MCP3424 config register start converting | Channel 0 | one shot mode | 18bit | x1 gain
#define config_reg_ch1 0xAC	// MCP3424 config register start converting | Channel 1 | one shot mode | 18bit | x1 gain
#define config_reg_ch2 0xCC	// MCP3424 config register start converting | Channel 2 | one shot mode | 18bit | x1 gain
#define config_reg_ch3 0xEF	// MCP3424 config register start converting | Channel 3 | one shot mode | 18bit | x8 gain


#define read_cmd_ch0  config_reg_ch0 & 0x7F
#define read_cmd_ch1  config_reg_ch1 & 0x7F
#define read_cmd_ch2  config_reg_ch2 & 0x7F
#define read_cmd_ch3  config_reg_ch3 & 0x7F


int main (void)
 {
  wiringPiSetup () ;

  printf("\033[H\033[2J Start\r\n");		// Clears terminal Screen, set cursor to top left

int fd,status;					// fd file store , status use for ADC status
long result,offset_g,offset_a;			// temp raw value returned from ADC , offset_(n) use to zero results
float volts,amps,grams;				
__u8 res[4];					// buffer for ADC returnd values
char *fileName = "/dev/i2c-1";			// Name of the port we will be using,Rasberry Pi model B (i2c-1)
int  address = adc_address;			// Address of I2C device (No links fitted

    if ((fd = open (fileName, O_RDWR)) < 0)	// Open port for reading and writing
     {
      printf("Failed to open i2c port\r\n");
      exit(1);
     }

// Set the port options and set the address of the device
    if (ioctl(fd, I2C_SLAVE, address) < 0)
     {
       printf("Unable to get bus access to talk to slave\r\n");
       exit(1);
     }

//--------------------------------------------------- VOLTS ---------------------------------------------------
 
     for(int count=25; count>=1;count--)		// loop 25 times
       {
       i2c_smbus_write_byte(fd, config_reg_ch0);    // CHANNEL 0
       do
        {
         i2c_smbus_read_i2c_block_data(fd,read_cmd_ch0,4,res);	// read 3 data bytes and 1 status byte
         status=res[3];
         } while (status & 0x80);				// keep reading data until status = ready

       result = (res[0]<<16|res[1]<<8|res[2]);			// result = read data (3 bytes)

       if (result >=  0x20000) result = (0xFFFE0000 | result);	// convert 18bit data to 32bit integer
       volts =  result * 0.0000156;				// convert 18bits ADC data to Volts(float)
       printf("Volts %2.4fV   ",volts);		// prtint Volts


//--------------------------------------------------- BRIDGE ---------------------------------------------------

       i2c_smbus_write_byte(fd, config_reg_ch3);    // CHANNEL 3 Gain x8  (Bridge input via Amp)
       do
        {
         i2c_smbus_read_i2c_block_data(fd,read_cmd_ch3,4,res);	//read 3 data bytes and 1 status byte  ,CH 2 Gain x8 
         status=res[3];
         } while (status & 0x80);

        result = (res[0]<<16|res[1]<<8|res[2]);

        if (result >=  0x20000) result = (0xFFFE0000 | result);
	if (count == 25) offset_g = result;			// apply offset on first read
	grams = result - offset_g;
        grams =  grams /  50;					// convert to grams (Depends on load cell used)
	grams =  grams; 
        printf("Grams $ %2.1fg    ",grams);
       

//--------------------------------------------------- AMPS ---------------------------------------------------

       i2c_smbus_write_byte(fd, config_reg_ch2);		// CHANNEL 2 Gain x8  (Bridge input via Amp)
       do
        {
         i2c_smbus_read_i2c_block_data(fd,read_cmd_ch2,4,res);	//read 3 data bytes and 1 status byte  ,CH 2 Gain x1 
         status=res[3];
         } while (status & 0x80);

        result = (res[0]<<16|res[1]<<8|res[2]);

        if (result >=  0x20000) result = (0xFFFE0000 | result);
        if (count == 25) offset_a = result;			// apply offset on first read
	amps = result - offset_a;				// offset
        amps =  amps /   2975;					// convert to amps
        printf("Amps %2.2fA\r\n",amps);
       }



printf("END\r\n");




}
