/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef MYI2C_H    /* Guard against multiple inclusion */
#define MYI2C_H

#include "debug.h"
#include "communication_public.h"

#define COLOR_SENSOR_SERVER_TESTING 1

//Information specific to the TCS34725 RGB Color Sensor
//I2C address
#define TCS_I2C_ADDRESS 0x29
//Register locations
#define TCS_ENABLE_REGISTER 0x00
#define TCS_RGBC_TIMING_REGISTER 0x01
#define TCS_WAIT_TIMING_REGISTER 0x03
#define TCS_LOW_THRESHOLD_LOW_REGISTER 0x04
#define TCS_LOW_THRESHOLD_HIGH_REGISTER 0x05
#define TCS_HIGH_THRESHOLD_LOW_REGISTER 0x06
#define TCS_HIGH_THRESHOLD_HIGH_REGISTER 0x07
#define TCS_PERSISTENCE_REGISTER 0x0c
#define TCS_CONFIG_REGISTER 0x0d
#define TCS_CONTROL_REGISTER 0x0f
#define TCS_ID_REGISTER 0x12
#define TCS_STATUS_REGISTER 0x13
#define TCS_RGBC_CLEAR_LOW_REGISTER 0x14
#define TCS_RGBC_CLEAR_HIGH_REGISTER 0x15
#define TCS_RGBC_RED_LOW_REGISTER 0x16
#define TCS_RGBC_RED_HIGH_REGISTER 0x17
#define TCS_RGBC_GREEN_LOW_REGISTER 0x18
#define TCS_RGBC_GREEN_HIGH_REGISTER 0x19
#define TCS_RGBC_BLUE_LOW_REGISTER 0x1a
#define TCS_RGBC_BLUE_HIGH_REGISTER 0x1b
//Bit masks for registers
#define TCS_ENABLE_AIEN_MASK 0x10
#define TCS_ENABLE_WEN_MASK 0x08
#define TCS_ENABLE_AEN_MASK 0x02
#define TCS_ENABLE_PON_MASK 0x01
#define TCS_PERSISTENCE_APERS_MASK 0x0f
#define TCS_CONFIG_WLONG_MASK 0x02
#define TCS_CONTROL_AGAIN_MASK 0x03
#define TCS_STATUS_AINT_MASK 0x10
#define TCS_STATUS_AVALID_MASK 0x01
//Values for the registers
#define TCS_AGAIN_1X 0x00
#define TCS_AGAIN_4X 0x01
#define TCS_AGAIN_16X 0x02
#define TCS_AGAIN_60X 0x03

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
//------------------------------------------------------------------------------
//TCS Color sensor control functions
//------------------------------------------------------------------------------
//Initializes the color sensor
void TCS_Init(I2C_MODULE_ID bus, unsigned char waitTime, unsigned char gain);

//Configures the interrupt stuff for the color sensor
void TCS_IntConfig(I2C_MODULE_ID bus, unsigned short lowThreshold, unsigned short highThreshold, unsigned char persistence);

//Send a byte to the color sensor
void TCS_SendByte(I2C_MODULE_ID bus, unsigned char address, unsigned char data);

//Receive a byte from the color sensor
unsigned char TCS_GetByte(I2C_MODULE_ID bus, unsigned char address);

//------------------------------------------------------------------------------
//I2C helper functions
//------------------------------------------------------------------------------
//Creates the 8-bit address from a 7-bit address and a read/write bit
unsigned char CreateAddressWord(unsigned char address, unsigned char readWrite);

//------------------------------------------------------------------------------
//I2C communication functions
//------------------------------------------------------------------------------
//Take control on an I2C bus and begin transmission
bool MyI2CStartBus(I2C_MODULE_ID bus);

//Restart transmission on a bus that you already control
bool MyI2CRepeatStartBus(I2C_MODULE_ID bus);

//Send the stop signal, then end the transmission
bool MyI2CStopBus(I2C_MODULE_ID bus);

//Initialize the bus
//Configures the baud rate of the bus, then enables it
bool MyI2CInit(I2C_MODULE_ID bus, unsigned int sourceFrequency, unsigned int baudRate);

//Send a byte of data over the bus
bool MyI2CSendByte(I2C_MODULE_ID bus, unsigned char data);

//Receive a byte of data from the bus
unsigned char MyI2CReadByte(I2C_MODULE_ID bus);

//------------------------------------------------------------------------------
//Server testing functions
//------------------------------------------------------------------------------
void ServerTestColorSensor(I2C_MODULE_ID bus);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* MYI2C_H */

/* *****************************************************************************
 End of File
 */
