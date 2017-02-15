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

#ifndef NAVIGATION_PUBLIC_H    /* Guard against multiple inclusion */
#define NAVIGATION_PUBLIC_H


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
    
//Sensor rover defines
#define NAV_TIMER_COUNTER_3_ID_SENSOR 0
#define NAV_TIMER_COUNTER_5_ID_SENSOR 1
#define NAV_COLOR_SENSOR_1_ID_SENSOR 2
#define NAV_COLOR_SENSOR_2_ID_SENSOR 3
#define NAV_COLOR_SENSOR_3_ID_SENSOR 4
#define NAV_MAPPING_ID_SENSOR 5

void navSendMsgFromISR(unsigned char msg[3]);
void navSendMsg(unsigned char msg[3]);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
