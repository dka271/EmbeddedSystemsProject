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
    
    void mapSendMsgFromISR(unsigned char msg[14]);
    void mapSendMsg(unsigned char msg[14]);
    
#define MAP_NAVIGATION_ID 0
#define MAP_PIXY_CAM_ID 1
#define MAP_ULTRASONIC_ID 2
#define MAP_IR_1_ID 3
#define MAP_IR_2_ID 4
#define MAP_MAPPING_TIMER_ID 5


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* MAPPING_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
