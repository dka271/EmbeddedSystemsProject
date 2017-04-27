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

#ifndef MAPPING_PUBLIC_H    /* Guard against multiple inclusion */
#define MAPPING_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
#define MAP_NAVIGATION_ID 0
#define MAP_PIXY_CAM_ID 1
#define MAP_ULTRASONIC_ID 2
#define MAP_IR_VAL 3
#define MAP_IR_2_ID 4
#define MAP_MAPPING_TIMER_ID 5
#define MAP_US_VAL 6
#define MAP_OBJ_2_MAP_ID 7
    
#define MAP_QUEUE_BUFFER_SIZE 15
#define MAP_CHECKSUM_IDX 14
#define MAP_SOURCE_ID_IDX 13
#define MAP_SOURCE_ID_MASK 0xe0
#define MAP_SOURCE_ID_OFFSET 5
#define MAP_OCCUPANCY_ROWS 127
#define MAP_OCCUPANCY_COLUMNS 97
#define MAP_OBJECT_THRESHOLD 30
#define MAP_ROVER_THRESHOLD 200
#define MAP_OBSTACLE_BITMASK 0x7ff

    
unsigned char mapCalculateChecksum(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]);
    
void mapSendMsgFromISR(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]);
void mapSendMsg(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]);
void mapSetYPosition(float yPos);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* MAPPING_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
