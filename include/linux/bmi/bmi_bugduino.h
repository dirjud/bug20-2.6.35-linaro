/*
 * File:         include/linux/bmi/bmi_bugduino.h
 * Author:       Peter Giacomini <p.giacomini@encadis.com>
 *
 * 		This is the application header file for the BMI bus voh Hippel plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_BUGDUINO_H
#define BMI_BUGDUINO_H

#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define BUGDUINO_GPIO_RED_LED		3	// default to input
#define BUGDUINO_GPIO_GREEN_LED		2	// default to input
#define BUGDUINO_GPIO_RESET_PIN		1	// default to input
#define BUGDUINO_GPIO_0			0	// default to input

#define BUGDUINO_GPIO_LED_ON		0
#define BUGDUINO_GPIO_LED_OFF		1

#define BUGDUINO_RESET_ON		1
#define BUGDUINO_RESET_OFF		0

// I2C
// I2C Slave Addresses
#define BMI_IOX_I2C_ADDRESS	0x71	// 7-bit address

// I2C IOX register addresses
#define IOX_INPUT_REG		0x0
#define IOX_OUTPUT_REG		0x1
#define IOX_POLARITY_REG	0x2
#define IOX_CONTROL		0x3

#define	BUGDUINO_IOX_B7		7	// set to output driven high to prevent interrupts
#define	BUGDUINO_IOX_B6		6	// set to output driven high to prevent interrupts
#define	BUGDUINO_IOX_B5		5	// set to output driven high to prevent interrupts
#define	BUGDUINO_IOX_B4		4	// set to output driven high to prevent interrupts
#define	BUGDUINO_IOX_B3		3	// set to output driven high to prevent interrupts
#define	BUGDUINO_IOX_B2		2	// set to output driven high to prevent interrupts
#define	BUGDUINO_IOX_B1		1	// set to output driven high to prevent interrupts
#define	BUGDUINO_IOX_B0		0	// set to output driven high to prevent interrupts

// SPI
#define BUF_MAX_SIZE	120

// SPI transfer structure
struct spi_xfer {
	unsigned char addr;
	unsigned char data[2];
} spi_xfer;

struct i2c_xfer {
	unsigned char addr;
	unsigned char offset;
	char * data;
	int len;
};

// von hippel driver ioctl definitions
#define BMI_BUGDUINO_RLEDOFF		_IOW(BMI_BUGDUINO_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_BUGDUINO_RLEDON		_IOW(BMI_BUGDUINO_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_BUGDUINO_GLEDOFF		_IOW(BMI_BUGDUINO_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_BUGDUINO_GLEDON		_IOW(BMI_BUGDUINO_IOCTL, 0x4, unsigned int)		// Turn on green LED
#define BMI_BUGDUINO_GETSTAT		_IOR(BMI_BUGDUINO_IOCTL, 0x5, unsigned int *)		// READ IOX register
#define BMI_BUGDUINO_RESET		_IOW(BMI_BUGDUINO_IOCTL, 0x6, unsigned int)		// Assert AVR Reset pin
#define BMI_BUGDUINO_I2C_WRITE		_IOW(BMI_BUGDUINO_IOCTL, 0x7, struct i2c_xfer)		// Write a packet of data to other i2c devices
#define BMI_BUGDUINO_I2C_READ		_IOR(BMI_BUGDUINO_IOCTL, 0x8, struct i2c_xfer)		// Read a packet of data to other i2c devices
#define BMI_BUGDUINO_MKIOX_OUT	_IOW(BMI_BUGDUINO_IOCTL, 0xa, unsigned int)		// make a IOX bit an output
#define BMI_BUGDUINO_MKIOX_IN		_IOW(BMI_BUGDUINO_IOCTL, 0xb, unsigned int)		// make a IOX bit an input
#define BMI_BUGDUINO_SETIOX		_IOW(BMI_BUGDUINO_IOCTL, 0xc, unsigned int)		// set a IOX output to 1
#define BMI_BUGDUINO_CLRIOX		_IOW(BMI_BUGDUINO_IOCTL, 0xd, unsigned int)		// set a IOX output to 0
#define BMI_BUGDUINO_SUSPEND		_IOR(BMI_BUGDUINO_IOCTL, 0x16, unsigned int)
#define BMI_BUGDUINO_RESUME		_IOR(BMI_BUGDUINO_IOCTL, 0x17, unsigned int)

#endif	/* BMI_BUGDUINO_H */

