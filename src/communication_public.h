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

#ifndef COMMUNICATION_PUBLIC_H    /* Guard against multiple inclusion */
#define COMMUNICATION_PUBLIC_H


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
    
    void commSendMsgFromISR(unsigned char msg[6]);
    void commSendMsg(unsigned char msg[6]);
    
#define COMM_UART_ID 0
#define COMM_MAPPING_ID 1


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
