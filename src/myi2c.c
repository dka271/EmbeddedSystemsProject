/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "myi2c.h"

//------------------------------------------------------------------------------
//TCS Color sensor control functions
//------------------------------------------------------------------------------
//Initializes the color sensor
void TCS_Init(I2C_MODULE_ID bus, unsigned char waitTime, unsigned char gain){
    //Initialize the device
    unsigned char enableData = TCS_GetByte(bus, TCS_ENABLE_REGISTER);
    TCS_SendByte(bus, TCS_ENABLE_REGISTER, (enableData | (TCS_ENABLE_PON_MASK | TCS_ENABLE_AIEN_MASK | TCS_ENABLE_AEN_MASK | TCS_ENABLE_WEN_MASK)));
    //Set the wait time
    TCS_SendByte(bus, TCS_WAIT_TIMING_REGISTER, waitTime);
    //Set the ADC gain
    TCS_SendByte(bus, TCS_CONTROL_REGISTER, (gain & TCS_CONTROL_AGAIN_MASK));
}

//Configures the interrupt stuff for the color sensor
void TCS_IntConfig(I2C_MODULE_ID bus, unsigned short lowThreshold, unsigned short highThreshold, unsigned char persistence){
    //Set the interrupt thresholds
    TCS_SendByte(bus, TCS_LOW_THRESHOLD_LOW_REGISTER, (lowThreshold & 0x00ff));
    TCS_SendByte(bus, TCS_LOW_THRESHOLD_HIGH_REGISTER, ((lowThreshold & 0xff00) >> 8));
    TCS_SendByte(bus, TCS_HIGH_THRESHOLD_LOW_REGISTER, (highThreshold & 0x00ff));
    TCS_SendByte(bus, TCS_HIGH_THRESHOLD_HIGH_REGISTER, ((highThreshold & 0xff00) >> 8));
    //Set the persistence value
    TCS_SendByte(bus, TCS_PERSISTENCE_REGISTER, (persistence & TCS_PERSISTENCE_APERS_MASK));
}

//Send a byte to the color sensor
void TCS_SendByte(I2C_MODULE_ID bus, unsigned char address, unsigned char data){
    MyI2CStartBus(bus);
    MyI2CSendByte(bus, CreateAddressWord(TCS_I2C_ADDRESS, I2C_WRITE));
    MyI2CSendByte(bus, address);
    MyI2CSendByte(bus, data);
    MyI2CStopBus(bus);
}

//Receive a byte from the color sensor
unsigned char TCS_GetByte(I2C_MODULE_ID bus, unsigned char address){
    MyI2CStartBus(bus);
    MyI2CSendByte(bus, CreateAddressWord(TCS_I2C_ADDRESS, I2C_WRITE));
    MyI2CSendByte(bus, address);
    MyI2CRepeatStartBus(bus);
    MyI2CSendByte(bus, CreateAddressWord(TCS_I2C_ADDRESS, I2C_READ));
    unsigned char data = MyI2CReadByte(bus);
    MyI2CStopBus(bus);
    return data;
}

//------------------------------------------------------------------------------
//I2C helper functions
//------------------------------------------------------------------------------
//Creates the 8-bit address from a 7-bit address and a read/write bit
unsigned char CreateAddressWord(unsigned char address, unsigned char readWrite){
    unsigned char word = (address << 1) | (readWrite & 0x01);
    return word;
}

//------------------------------------------------------------------------------
//I2C communication functions
//------------------------------------------------------------------------------
//Take control on an I2C bus and begin transmission
bool MyI2CStartBus(I2C_MODULE_ID bus){
    if (!PLIB_I2C_BusIsIdle(bus)){
        //Return if the bus is not idle
        return 0;
    }
    //Start the transmission
    PLIB_I2C_MasterStart(bus);
    
    return 1;
}

//Restart transmission on a bus that you already control
bool MyI2CRepeatStartBus(I2C_MODULE_ID bus){
    //Restart the bus
    PLIB_I2C_MasterStartRepeat(bus);
    
    return 1;
}

//Send the stop signal, then end the transmission
bool MyI2CStopBus(I2C_MODULE_ID bus){
    //Stop the bus
    PLIB_I2C_MasterStop(bus);
    
    return 1;
}

//Initialize the bus
bool MyI2CInit(I2C_MODULE_ID bus, unsigned int sourceFrequency, unsigned int baudRate){
    //Set the baud rate
//    PLIB_I2C_BaudRateSet(bus, sourceFrequency, baudRate);
    //Enable the bus
    PLIB_I2C_Enable(bus);
    
    return 1;
}

//Send a byte of data over the bus
bool MyI2CSendByte(I2C_MODULE_ID bus, unsigned char data){
    if (!PLIB_I2C_TransmitterIsReady(bus)){
        //Return false if the transmitter is not ready
        return 0;
    }
    //Send the data
    PLIB_I2C_TransmitterByteSend(bus, data);
    if (!PLIB_I2C_TransmitterByteHasCompleted(bus)){
        //Wait for the transmit to complete
    }
    if (!PLIB_I2C_TransmitterByteWasAcknowledged(bus)){
        //Wait for an acknowledgement
    }
    
    return 1;
}

//Receive a byte of data from the bus
unsigned char MyI2CReadByte(I2C_MODULE_ID bus){
    //Clock a byte from the slave device
    PLIB_I2C_MasterReceiverClock1Byte(bus);
    if (!PLIB_I2C_ReceivedByteIsAvailable(bus)){
        //Return if there is no data available
        return 0;
    }
    //Send an acknowlegement
    PLIB_I2C_ReceivedByteAcknowledge(bus, 1);
    if (!PLIB_I2C_ReceiverByteAcknowledgeHasCompleted(bus)){
        //Wait for the ack to complete
    }
    //Get the byte from the receiver
    return PLIB_I2C_ReceivedByteGet(bus);
}

//------------------------------------------------------------------------------
//Server testing functions
//------------------------------------------------------------------------------
void ServerTestColorSensor(I2C_MODULE_ID bus){
    if (!COLOR_SENSOR_SERVER_TESTING){
        //Do not preform the test if not testing
        return;
    }
    unsigned char clow = TCS_GetByte(bus, TCS_RGBC_CLEAR_LOW_REGISTER);
    unsigned char chigh = TCS_GetByte(bus, TCS_RGBC_CLEAR_HIGH_REGISTER);
    unsigned char rlow = TCS_GetByte(bus, TCS_RGBC_RED_LOW_REGISTER);
    unsigned char rhigh = TCS_GetByte(bus, TCS_RGBC_RED_HIGH_REGISTER);
    unsigned char glow = TCS_GetByte(bus, TCS_RGBC_GREEN_LOW_REGISTER);
    unsigned char ghigh = TCS_GetByte(bus, TCS_RGBC_GREEN_HIGH_REGISTER);
    unsigned char blow = TCS_GetByte(bus, TCS_RGBC_BLUE_LOW_REGISTER);
    unsigned char bhigh = TCS_GetByte(bus, TCS_RGBC_BLUE_HIGH_REGISTER);
    unsigned int c = clow | (chigh << 8);
    unsigned int r = rlow | (rhigh << 8);
    unsigned int g = glow | (ghigh << 8);
    unsigned int b = blow | (bhigh << 8);
    unsigned char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
    unsigned char id = TCS_GetByte(bus, TCS_ID_REGISTER);
    sprintf(testServerMsg, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d,%d,%d,%d],\"C\":123}~", id, c, r, g, b);
    commSendMsgToWifiQueue(testServerMsg);
}


/* *****************************************************************************
 End of File
 */
