// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// RaspberryPi bcm2835 library port: bcm2835 library available at http://www.airspayce.com/mikem/bcm2835/index.html
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"
#include "wiringPiI2C.h"
#include <iostream>
#include <stdio.h>
#include "wiringPi.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#define RPI_I2C_REASON_OK 1

int s_fd = 0;
char sendBuf[256];
char recvBuf[256];

I2Cdev::I2Cdev() { }

int I2Cdev::initialize()
{

	s_fd = open("/dev/i2c-1", O_RDWR);
	if (s_fd < 0)
	{
		std::cout << "i2c device init fail\n";
		return wiringPiFailure (WPI_ALMOST, "Unable to open I2C device: %s\n", strerror (errno));
	}
	std::cout << s_fd << "\n";
	return true;

/*
  bcm2835_init();
  bcm2835_i2c_set_baudrate( i2c_baudrate  );
*/
}

/** Enable or disable I2C, 
 * @param isEnabled true = enable, false = disable
 */
void I2Cdev::enable(bool isEnabled) {
/*
  if ( set_I2C_pins ){
    if (isEnabled)
      bcm2835_i2c_end();
    else
      bcm2835_i2c_begin() ;
  }
*/
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t bitNum, 
	uint8_t *data
)
{
//	int fd = wiringPiI2CSetup(devAddr);
	ioctl(s_fd, I2C_SLAVE, devAddr);
	int retval = wiringPiI2CReadReg8(s_fd, regAddr);
	uint8_t response = ( retval >= 0 );

	uint8_t byte = (uint8_t) retval;
	*data = byte & ( 1 << bitNum);

	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1);
  *data = recvBuf[1] & (1 << bitNum);
  return response == BCM2835_I2C_REASON_OK ;
*/
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t bitStart, 
	uint8_t length, 
	uint8_t *data
) 
{
	ioctl(s_fd, I2C_SLAVE, devAddr);
	int retval = wiringPiI2CReadReg8(s_fd, regAddr);
	uint8_t response = ( retval >= 0 );

	uint8_t byte = (uint8_t) retval;
	if (response == RPI_I2C_REASON_OK)
	{
		uint8_t mask = ( ( 1 << length ) - 1 ) << (bitStart - length + 1);

		byte &= mask;
		byte >>= ( bitStart - length + 1 );
		*data = byte;
	}
	return response == RPI_I2C_REASON_OK;
/*
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1);
  uint8_t b = (uint8_t) recvBuf[0];
  if (response == BCM2835_I2C_REASON_OK) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }
  return response == BCM2835_I2C_REASON_OK;
*/
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t *data
)
{
	ioctl(s_fd, I2C_SLAVE, devAddr);
	int retval = wiringPiI2CReadReg8(s_fd, regAddr);
	uint8_t response = ( retval >= 0 );

	uint8_t byte = (uint8_t) retval;
	*data = byte;
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1);
  data[0] = (uint8_t) recvBuf[0];
  return response == BCM2835_I2C_REASON_OK;
*/
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t I2Cdev::readBytes
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t length, 
	uint8_t *data
)
{
	ioctl(s_fd, I2C_SLAVE, devAddr);

	int retval;
	uint8_t response = true;
	uint8_t byte = 0b00000000;

	for ( int n_register = 0; n_register < length; n_register++ )
	{
		// get retval from reading
		retval = wiringPiI2CReadReg8(s_fd, regAddr++);

		// convert retval to local_response
		uint8_t local_response = ( retval >= 0 );

		// factor local response to global response
		response &= local_response;

		// convert retval to 8-bit unsigned integer
		byte = (uint8_t) retval;

		// store read byte to data
		data[n_register] = byte;
	}
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, length);
  int i ;
  for (i = 0; i < length ; i++) {
    data[i] = (uint8_t) recvBuf[i];
  }
  return response == BCM2835_I2C_REASON_OK;
*/
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t bitNum, 
	uint8_t data
) 
{
	ioctl(s_fd, I2C_SLAVE, devAddr);

	int retval;
	uint8_t response;
	uint8_t byte;

	// get retval from reading
	retval = wiringPiI2CReadReg8(s_fd, regAddr);

	// convert retval to response
	response = ( retval >= 0 );

	if ( response == RPI_I2C_REASON_OK )
	{
		// convert retval to 8-bit unsigned integer
		byte = (uint8_t) retval;

		// write bit to read byte
		byte = (data != 0) ? ( byte | ( 1 << bitNum ) ) : ( byte & ~( 1 << bitNum ) );

		// get retval from writing
		retval = wiringPiI2CWriteReg8(s_fd, regAddr, byte) ;

		// convert retval to response
		response = retval >= 0;
	}
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  //first reading registery value
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1 );
  if ( response == BCM2835_I2C_REASON_OK ) {
    uint8_t b = recvBuf[0] ;
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    sendBuf[1] = b ;
    response = bcm2835_i2c_write(sendBuf, 2);
  }
  return response == BCM2835_I2C_REASON_OK;
*/
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t bitStart, 
	uint8_t length, 
	uint8_t data
) 
{
	ioctl(s_fd, I2C_SLAVE, devAddr);

	int retval;
	uint8_t response;
	uint8_t byte;

	// get retval from register reading
	retval = wiringPiI2CReadReg8(s_fd, regAddr);

	// convert retval to response
	response = (retval >= 0);

/*
**      010 value to write
** 76543210 bit numbers
**    xxx   args: bitStart=4, length=3
** 00011100 mask byte
** 10101111 original value (sample)
** 10100011 original & ~mask
** 10101011 masked | value
*/
	if ( response == RPI_I2C_REASON_OK )
	{
		// convert retval to 8-bit unsigned integer
		byte = (uint8_t) retval;

		// create mask
		uint8_t mask = ( (1 << length) - 1) << (bitStart - length + 1);

		// shift data into correct position
		data <<= (bitStart - length + 1); 

		// zero all non-important bits in data
		data &= mask; 

		// zero all important bits in existing byte
		byte &= ~(mask); 

		// combine data with existing byte
		byte |= data; 

		// get retval from byte write
		retval = wiringPiI2CWriteReg8(s_fd, regAddr, byte) ;

		// convert retval to response
		response = ( retval >= 0 );
	}
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  //first reading registery value
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1 );
  if ( response == BCM2835_I2C_REASON_OK ) {
    uint8_t b = recvBuf[0];
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    sendBuf[1] = b ;
    response = bcm2835_i2c_write(sendBuf, 2);
    }
  return response == BCM2835_I2C_REASON_OK;
*/
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t data
) 
{
//	int fd = wiringPiI2CSetup(devAddr);
	ioctl(s_fd, I2C_SLAVE, devAddr);

	// get retval from writing
	int retval = wiringPiI2CWriteReg8(s_fd, regAddr, data);

	// convert retval to response
	uint8_t response = retval >= 0;

	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  sendBuf[1] = data;
  uint8_t response = bcm2835_i2c_write(sendBuf, 2);
  return response == BCM2835_I2C_REASON_OK ;
*/
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord
(
	uint8_t devAddr,
	uint8_t regAddr,
	uint16_t *data
)
{
//	int fd = wiringPiI2CSetup(devAddr);
	ioctl(s_fd, I2C_SLAVE, devAddr);
	int retval = wiringPiI2CReadReg16(s_fd, regAddr);
	uint8_t response = retval >= 0;
	uint16_t word = (uint16_t) retval;
	data[0] = word;
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 2 );
  data[0] = (recvBuf[0] << 8) | recvBuf[1] ;
  return response == BCM2835_I2C_REASON_OK ;
*/
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t length, 
	uint16_t *data
)
{
//	int fd = wiringPiI2CSetup(devAddr);
	ioctl(s_fd, I2C_SLAVE, devAddr);
	int retval = 0;
	uint8_t response = true;

	uint8_t regAddr_inc = regAddr;
	for (int n_word = 0; n_word < length; n_word++)
	{
		retval = wiringPiI2CReadReg16(s_fd, regAddr_inc++);
		response &= ( retval >= 0 );
		uint16_t word = (uint16_t) retval;
		data[n_word] = word;
	}
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, length*2 );
  uint8_t i;
  for (i = 0; i < length; i++) {
    data[i] = (recvBuf[i*2] << 8) | recvBuf[i*2+1] ;
  }
  return  response == BCM2835_I2C_REASON_OK ;
*/
}

bool I2Cdev::writeWord
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint16_t data
)
{
//	int fd = wiringPiI2CSetup(devAddr);
	ioctl(s_fd, I2C_SLAVE, devAddr);
	int retval = wiringPiI2CWriteReg16(s_fd, regAddr, data);
	uint8_t response = retval >= 0;
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  sendBuf[1] = (uint8_t) (data >> 8); //MSByte
  sendBuf[2] = (uint8_t) (data >> 0); //LSByte
  uint8_t response = bcm2835_i2c_write(sendBuf, 3);
  return response == BCM2835_I2C_REASON_OK ;
*/
}

bool I2Cdev::writeBytes
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t length, 
	uint8_t *data
)
{
//	int fd = wiringPiI2CSetup(devAddr);
	ioctl(s_fd, I2C_SLAVE, devAddr);

	int retval = 0;
	uint8_t response = true;

	uint8_t regAddr_inc = regAddr;
	for (int n_byte = 0; n_byte < length; n_byte++)
	{
		retval = wiringPiI2CWriteReg8(s_fd, regAddr_inc++, data[n_byte]);
		response &= ( retval >= 0 );
	}
	return response == RPI_I2C_REASON_OK;
/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t i;
  for (i = 0; i < length; i++) {
    sendBuf[i+1] = data[i] ;
  }
  uint8_t response = bcm2835_i2c_write(sendBuf, 1+length);
  return response == BCM2835_I2C_REASON_OK ;
*/
}

bool I2Cdev::writeWords
(
	uint8_t devAddr, 
	uint8_t regAddr, 
	uint8_t length, 
	uint16_t *data
)
{
//	int fd = wiringPiI2CSetup(devAddr);
	ioctl(s_fd, I2C_SLAVE, devAddr);
	int retval = 0;
	uint8_t response = true;

	uint8_t regAddr_inc = regAddr;
	for (int n_word = 0; n_word < length; n_word++)
	{
		retval = wiringPiI2CWriteReg16(s_fd, regAddr_inc++, data[n_word]);
		response &= ( retval >= 0 );
	}
	return response == RPI_I2C_REASON_OK;

/*
  bcm2835_i2c_setSlaveAddress(devAddr);
  sendBuf[0] = regAddr;
  uint8_t i;
  for (i = 0; i < length; i++) {
    sendBuf[1+2*i] = (uint8_t) (data[i] >> 8); //MSByte
    sendBuf[2+2*i] = (uint8_t) (data[i] >> 0); //LSByte
  }
  uint8_t response = bcm2835_i2c_write(sendBuf, 1+2*length);
  return response == BCM2835_I2C_REASON_OK ;
*/
}
