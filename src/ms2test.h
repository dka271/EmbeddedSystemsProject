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

#ifndef _MS2TEST_H    /* Guard against multiple inclusion */
#define _MS2TEST_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
#include "mapping.h"
#include "mapping_public.h"
#include "communication.h"
#include "communication_public.h"
#include "navigation_public.h"
#include "debug.h"
/* TODO:  Include other files here if needed. */
#define UNIT_TESTING true

void automatedTesting();
int mapChecksumTest();
int commChecksumTest();
int navChecksumTest();
int mapQueueTest();
void mapQueueReceiveTest(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]);
int commQueueTest();
void commQueueReceiveTest(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]);
int navQueueTest();
void navQueueReceiveTest(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]);


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif



    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
