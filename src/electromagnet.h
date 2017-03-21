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

#ifndef ELECTROMAGNET_H    /* Guard against multiple inclusion */
#define ELECTROMAGNET_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "debug.h"
#include "communication_public.h"
#include "myjson.h"


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

//The electromagnet operates using port D 8 and 11
#define ELECTROMAGNET_PINS 0x0900
    
//Used for testing
#define TEST_ELECTROMAGNET_SERVER_ID ((unsigned char) 'y')

//Sets the electromagnet on
void ElectromagnetSetOn();

//Sets the electromagnet off
void ElectromagnetSetOff();

//Used to communicate with the electromagnet testing server
//Handle receiving
void electromagnetTestCommReceive(unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE]);
//Handle sending
void electromagnetTestSend(int status);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* ELECTROMAGNET_H */

/* *****************************************************************************
 End of File
 */
