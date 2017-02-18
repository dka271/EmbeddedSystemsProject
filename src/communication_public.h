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
    
#define COMM_UART_ID 0
#define COMM_MAPPING_ID 1

#define SEND_QUEUE_BUFFER_SIZE 216 
#define COMM_QUEUE_BUFFER_SIZE 8
#define COMM_CHECKSUM_IDX 7
#define COMM_SOURCE_ID_IDX 6
#define COMM_SOURCE_ID_MASK 0x80
#define COMM_SOURCE_ID_OFFSET 7

    
unsigned char commCalculateChecksum(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]);
    
void commSendMsgFromISR(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]);
void commSendMsg(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]);
void readPublic(char* bufferToWriteTo, int size);
void writePublic(char* bufferToReadFrom);
void uartReceiveFromSendQueueInISR(unsigned char msg[SEND_QUEUE_BUFFER_SIZE]);



    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
