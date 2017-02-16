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

/* This section lists the other files that are included in this file.
 */
#include "mapping.h"
#include "mapping_public.h"
#include "ms2test.h"
#include "debug.h"
/* TODO:  Include other files here if needed. */



int mapChecksumTest(){
    dbgUARTString("Map checksum test: \n\r", 21);

    unsigned char testInput1[MAP_QUEUE_BUFFER_SIZE];
    testInput1[0] = 'q';
    testInput1[1] = 'w';
    testInput1[2] = 'e';
    testInput1[3] = 'r';
    testInput1[4] = 't';
    testInput1[5] = 'y';
    testInput1[6] = 'u';
    testInput1[7] = 'i';
    testInput1[8] = 'o';
    testInput1[9] = 'p';
    testInput1[10] = 'a';
    testInput1[11] = 's';
    testInput1[12] = 'd';
    testInput1[13] = (MAP_NAVIGATION_ID & 0x7) << MAP_SOURCE_ID_OFFSET;
    testInput1[14] = mapCalculateChecksum(testInput1);
    
    unsigned char checksumVal = mapCalculateChecksum(testInput1);
    unsigned char expectedVal = (unsigned char) (1441 & 0xff);
    if ( checksumVal == expectedVal) {
        dbgUARTString("Test passed\n\r", 13);
    }
    else {
        dbgUARTString("Test failed\n\r", 13);
    }
    
}

int mapQueueTest() {
    dbgUARTString("Map queue test: \n\r", 18);

    unsigned char msg[MAP_QUEUE_BUFFER_SIZE];
    
    msg[0] = 'T';
    msg[1] = 'e';
    msg[2] = 's';
    msg[3] = 't';
    msg[4] = 'i';
    msg[5] = 'n';
    msg[6] = 'g';
    msg[7] = ' ';
    msg[8] = 'M';
    msg[9] = 'a';
    msg[10] = 'p';
    msg[11] = ' ';
    msg[12] = 'Q';
    msg[MAP_SOURCE_ID_IDX] = (MAP_ULTRASONIC_ID & 0x00000007) << MAP_SOURCE_ID_OFFSET;
    msg[MAP_CHECKSUM_IDX] = mapCalculateChecksum(msg);
    
    dbgUARTString("Test String = \"Testing Map Q\" \n\r", 32);
    
    mapSendMsg(msg);
}

void mapQueueReceiveTest(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]){
    dbgUARTString("Map Queue receive test string: \"", 32);
    dbgUARTVal(msg[0]);
    dbgUARTVal(msg[1]);
    dbgUARTVal(msg[2]);
    dbgUARTVal(msg[3]);
    dbgUARTVal(msg[4]);
    dbgUARTVal(msg[5]);
    dbgUARTVal(msg[6]);
    dbgUARTVal(msg[7]);
    dbgUARTVal(msg[8]);
    dbgUARTVal(msg[9]);
    dbgUARTVal(msg[10]);
    dbgUARTVal(msg[11]);
    dbgUARTVal(msg[12]);
    dbgUARTString("\"\n\r", 3);
}


int commChecksumTest(){
    dbgUARTString("Comm checksum test: \n\r", 22);

    unsigned char testInput2[COMM_QUEUE_BUFFER_SIZE];
    testInput2[0] = 'q';
    testInput2[1] = 'w';
    testInput2[2] = 'e';
    testInput2[3] = 'r';
    testInput2[4] = 't';
    testInput2[5] = 'y';
    testInput2[6] = (COMM_UART_ID & 0x1) << COMM_SOURCE_ID_OFFSET;
    testInput2[7] = commCalculateChecksum(testInput2);
    
    unsigned char checksumVal = commCalculateChecksum(testInput2);
    unsigned char expectedVal = (unsigned char) (684 & 0xff);
    if ( checksumVal == expectedVal) {
        dbgUARTString("Test passed\n\r", 13);
    }
    else {
        dbgUARTString("Test failed\n\r", 13);
    }
    
}

int commQueueTest() {
    dbgUARTString("Comm queue test: \n\r", 19);

    unsigned char msg[COMM_QUEUE_BUFFER_SIZE];
    
    msg[0] = 'T';
    msg[1] = 'e';
    msg[2] = 's';
    msg[3] = 't';
    msg[4] = 'C';
    msg[5] = 'Q';
    msg[COMM_SOURCE_ID_IDX] = (COMM_UART_ID & 0x00000001) << COMM_SOURCE_ID_OFFSET;
    msg[COMM_CHECKSUM_IDX] = commCalculateChecksum(msg);
    
    dbgUARTString("Test String = \"TestCQ\" \n\r", 25);
    
    commSendMsg(msg);
}

void commQueueReceiveTest(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]){
    dbgUARTString("Comm Queue receive test string: \"", 33);
    dbgUARTVal(msg[0]);
    dbgUARTVal(msg[1]);
    dbgUARTVal(msg[2]);
    dbgUARTVal(msg[3]);
    dbgUARTVal(msg[4]);
    dbgUARTVal(msg[5]);
    dbgUARTString("\"\n\r", 3);
}

int navChecksumTest(){
    dbgUARTString("Nav checksum test: \n\r", 21);

    unsigned char testInput3[NAV_QUEUE_BUFFER_SIZE];
    testInput3[0] = 'q';
    testInput3[1] = 'w';
    testInput3[2] = (NAV_TIMER_COUNTER_3_ID_SENSOR & 0x1) << NAV_SOURCE_ID_OFFSET;
    testInput3[3] = navCalculateChecksum(testInput3);
    
    unsigned char checksumVal = navCalculateChecksum(testInput3);
    unsigned char expectedVal = (unsigned char) (232 & 0xff);
    if ( checksumVal == expectedVal) {
        dbgUARTString("Test passed\n\r", 13);
    }
    else {
        dbgUARTString("Test failed\n\r", 13);
    }
    
    
}

int navQueueTest() {
    dbgUARTString("Nav queue test: \n\r", 18);

    unsigned char msg[NAV_QUEUE_BUFFER_SIZE];
    
    msg[0] = 'T';
    msg[1] = 'Q';
    msg[NAV_SOURCE_ID_IDX] = (NAV_COLOR_SENSOR_3_ID_SENSOR & 0x00000007) << NAV_SOURCE_ID_OFFSET;
    msg[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg);
    
    dbgUARTString("Test String = \"TQ\" \n\r", 21);
    
    navSendMsg(msg);
}

void navQueueReceiveTest(unsigned char msg[NAV_QUEUE_BUFFER_SIZE]){
    dbgUARTString("Nav Queue receive test string: \"", 32);
    dbgUARTVal(msg[0]);
    dbgUARTVal(msg[1]);
    dbgUARTString("\"\n\r", 3);
}
    
    void automatedTesting() {
        if (UNIT_TESTING){
            dbgUARTString("---Automated Unit Testing Starting---\n\r", 39);
            mapChecksumTest();
            commChecksumTest();
            navChecksumTest();
            mapQueueTest();
            commQueueTest();
            navQueueTest();
            //dbgUARTString("---Automated Unit Testing Ending---\n\r", 37);
        }
    }

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

// *****************************************************************************



/* *****************************************************************************
 End of File
 */
