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

#ifndef _MYJSON_H    /* Guard against multiple inclusion */
#define _MYJSON_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "communication_public.h"
#include "debug.h"


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#define KEY_SOURCE 			((unsigned char) 'S')
#define KEY_DESTINATION 	((unsigned char) 'T')
#define KEY_MESSAGE_TYPE	((unsigned char) 'M')
#define KEY_SEQUENCE_NUMBER	((unsigned char) 'N')
#define KEY_DATA			((unsigned char) 'D')
#define KEY_FIELD_ITEM		((unsigned char) 'F')
#define KEY_CHECKSUM		((unsigned char) 'C')
#define ELEMENT_DELIM		((unsigned char) ':')

//Searches the JSON string "buffer" for the first occurrence of "key"
//The key is assumed to be a single character
//Returns the index of the colon corresponding to the found key value pair
//Will return -1 if the key is not found
int jsonFindElementFromKey (unsigned char *buffer, unsigned char key);

//Finds the source of the JSON message
//Returns 0 if success, -1 if the key is not present, 1 if there was a parse error
int jsonGetSource(unsigned char *buffer, unsigned char *source);
int jsonGetSourceString(unsigned char *buffer, unsigned char *source);

//Finds the destination of the JSON message
//Returns 0 if success, -1 if the key is not present, 1 if there was a parse error
int jsonGetDestination(unsigned char *buffer, unsigned char *destination);

//Finds the message type of the JSON message
//Returns 0 if success, -1 if the key is not present, 1 if there was a parse error
int jsonGetMessageType(unsigned char *buffer, unsigned char *messageType);

//Finds the sequence number of the JSON message
//Returns 0 if success, -1 if the key is not present, 1 if there was a parse error
int jsonGetSequenceNumber(unsigned char *buffer, unsigned char *sequenceNumber);

//Finds the checksum of the JSON message
//Returns 0 if success, -1 if the key is not present, 1 if there was a parse error
int jsonGetChecksum(unsigned char *buffer, unsigned int *checksum);
int jsonGetChecksumString(unsigned char *buffer, unsigned char *checksum);

//Finds the field item
//Returns 0 if success, -1 if the key is not present, 1 if there was a parse error
int jsonGetFieldItem(unsigned char *buffer, fieldItem *item);

//Return 0 upon success, 1 upon failure
int parseCharacter(char* bufferToReadFrom, int index, unsigned char *info);
int parseString(char* bufferToReadFrom, int index, unsigned char *info);
int parseInt(char* bufferToReadFrom, int index, unsigned int *info);
int parseUInt8(unsigned char* bufferToReadFrom, int indexFake, unsigned char *info);
int readNumInStringForm(char* bufferToReadFrom, int* index);
int parseFieldItem(char* bufferToReadFrom,int index, fieldItem* info);
int parseCheckSum(char* bufferToReadFrom, int index, unsigned char numInStringForm[6]);



    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _MYJSON_H */

/* *****************************************************************************
 End of File
 */
