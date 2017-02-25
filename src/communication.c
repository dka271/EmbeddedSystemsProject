/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    communication.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

#include "communication.h"
#include "communication_public.h"
#include "ms2test.h"


COMMUNICATION_DATA communicationData;

static QueueHandle_t commQueue;
static QueueHandle_t sendQueue;


int NumBadChecksums = 0;
int NumDroppedMessages = 0;

/*******************************************************************************
  Function:
    void COMMUNICATION_Initialize ( void )

  Remarks:
    See prototype in communication.h.
 */

void COMMUNICATION_Initialize(void) {
    /* Place the App state machine in its initial state. */
    communicationData.state = COMMUNICATION_STATE_INIT;

    //Initialize the mapping queue
    commQueue = xQueueCreate(10, sizeof (unsigned char[COMM_QUEUE_BUFFER_SIZE]));
    //    sendQueue = xQueueCreate(10, sizeof (unsigned char[SEND_QUEUE_BUFFER_SIZE]));
    sendQueue = xQueueCreate(10, sizeof (unsigned char));

    if (commQueue == 0) {
        dbgPauseAll();
    }
}

void commSendMsgFromISR(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    xQueueSendToBackFromISR(commQueue, msg, NULL);
}

void commSendMsg(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    xQueueSendToBack(commQueue, msg, portMAX_DELAY);
}

void constructFieldItem(fieldItem *object, unsigned char objectType, unsigned char versionNum, unsigned char length, unsigned char width, unsigned char centerX, unsigned char centerY, unsigned char orientation) {
    object->objectType = objectType;
    object->versionNumber = versionNum;
    object->length = length;
    object->width = width;
    object->centerX = centerX;
    object->centerY = centerY;
    object->orientation = orientation;
}

void convertFieldItemToJSON(unsigned char jsonFieldItem[208], fieldItem object, unsigned char source, unsigned char dest, unsigned char messageType, unsigned char seqNum) {
    unsigned char jsonFieldItemTemp[200];
    unsigned char jsonFieldItemEnd[8];
    int checkSum;
    sprintf(jsonFieldItemTemp, "*{\"S\":\"%c\",\"T\":\"%c\",\"M\":\"%c\",\"N\":%d,\"D\":[%d,%d],\"F\":[%d,%d,%d,%d,%d,%d,%d],\"C\":", source, dest, messageType, seqNum,NumBadChecksums, NumDroppedMessages, object.objectType, object.versionNumber, object.length, object.width, object.centerX, object.centerY, object.orientation);
    checkSum = calculateJsonStringCheckSum(jsonFieldItemTemp);
    sprintf(jsonFieldItemEnd, "%d}~", checkSum);
    strcpy(jsonFieldItem, jsonFieldItemTemp);
    strcat(jsonFieldItem, jsonFieldItemEnd);
}

void convertSensorMsgToJson(unsigned char jsonFieldItem[76], unsigned char source, unsigned char dest, unsigned char messageType, unsigned char seqNum) {
    unsigned char jsonFieldItemTemp[200];
    unsigned char jsonFieldItemEnd[8];
    int checkSum;
    sprintf(jsonFieldItemTemp, "*{\"S\":\"%c\",\"T\":\"%c\",\"M\":\"%c\",\"N\":%d,\"C\":", source, dest, messageType, seqNum);
    checkSum = calculateJsonStringCheckSum(jsonFieldItemTemp);
    sprintf(jsonFieldItemEnd, "%d}~", checkSum);
    strcpy(jsonFieldItem, jsonFieldItemTemp);
    strcat(jsonFieldItem, jsonFieldItemEnd);
}

void convertSensorMsgToJsonWithData(unsigned char jsonFieldItem[76], unsigned char source, unsigned char dest, unsigned char messageType, unsigned char seqNum) {
    unsigned char jsonFieldItemTemp[200];
    unsigned char jsonFieldItemEnd[8];
    int checkSum;
    //if true, make the bad sequence number increment twice
    sprintf(jsonFieldItemTemp, "*{\"S\":\"%c\",\"T\":\"%c\",\"M\":\"%c\",\"N\":%d,\"D\":[%d,%d],\"C\":", source, dest, messageType, seqNum, NumBadChecksums, NumDroppedMessages);

    //If true, make the bad checksum increment once
    //    if (seqNum == 60) {
    //        checkSum = DBGcalculateBADJsonStringCheckSum(jsonFieldItemTemp);
    //    } else {
    checkSum = calculateJsonStringCheckSum(jsonFieldItemTemp);
    //    }
    sprintf(jsonFieldItemEnd, "%d}~", checkSum);
    strcpy(jsonFieldItem, jsonFieldItemTemp);
    strcat(jsonFieldItem, jsonFieldItemEnd);
}

int SeqNum = 1;

void commSendMsgToSendQueue(unsigned char testString[RECEIVE_BUFFER_SIZE]) {
    //BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    int i;
    //unsigned char testString[76];
    fieldItem item;
    constructFieldItem(&item, (unsigned char)2, (unsigned char)15, (unsigned char)99, (unsigned char)66, (unsigned char)22, (unsigned char)11, (unsigned char)200);
    convertFieldItemToJSON(testString, item, 's', 'f', 'm', SeqNum);
//    convertSensorMsgToJson(testString, 'f', 's', 'm', (unsigned char)SeqNum);
//    convertSensorMsgToJsonWithData(testString, 's', 'f', 'm', (unsigned char) SeqNum);
    SeqNum++;
    if (SeqNum >= 64) {
        SeqNum = 0;
    }

    for (i = 0; i < strlen(testString); i++) {
        xQueueSendToBack(sendQueue, &testString[i], portMAX_DELAY);
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
}

int calculateJsonStringCheckSum(unsigned char* string) {
    int i;
    int checkSum = 0;
    for (i = 0; i < strlen(string); i++) {
        checkSum += string[i];
    }
    checkSum += '}';
    //checkSum += '~';
    return checkSum;
}

int DBGcalculateBADJsonStringCheckSum(unsigned char* string) {
    int i;
    int checkSum = 0;
    for (i = 0; i < strlen(string); i++) {
        checkSum += string[i];
    }
    checkSum += '}';
    checkSum++;
    //checkSum += '~';
    return checkSum;
}

void uartReceiveFromSendQueueInISR(unsigned char msg[7]) {
    xQueueReceiveFromISR(sendQueue, msg, NULL); //CHECK ME pdFalse, changes, go to taskYIED();)
}

bool checkIfSendQueueIsEmpty() {
    return xQueueIsQueueEmptyFromISR(sendQueue);
}

unsigned char commCalculateChecksum(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]) {
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < COMM_QUEUE_BUFFER_SIZE - 1; i++) {
        sum += msg[i];
    }
    return sum;
}

/******************************************************************************
  Function:
    void COMMUNICATION_Tasks ( void )

  Remarks:
    See prototype in communication.h.
 */

void COMMUNICATION_Tasks(void) {
    dbgOutputLoc(DBG_LOC_COMM_ENTER);

	unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE];
	//            unsigned char receiveFromWifiBuffer[RECEIVE_BUFFER_SIZE];
	//            unsigned int receiveFromWifiBufferIdx = 0;
	int PreviousSequenceNumber = 0;

	dbgOutputLoc(DBG_LOC_COMM_BEFORE_WHILE);
	dbgOutputLoc(DBG_LOC_COMM_BEFORE_WHILE_NEW_VAL);
	while (1) {
		//Block until a message is received
		dbgOutputLoc(DBG_LOC_COMM_BEFORE_RECEIVE);
		BaseType_t receiveCheck = xQueueReceive(commQueue, receivemsg, portMAX_DELAY);
		dbgOutputLoc(DBG_LOC_COMM_AFTER_RECEIVE);

		//Handle the message
		if (receiveCheck == pdTRUE) {
			//Convert the message into integer format
			//Get the message ID
			int msgId = (receivemsg[COMM_SOURCE_ID_IDX] & COMM_SOURCE_ID_MASK) >> COMM_SOURCE_ID_OFFSET;
			//Handle a specific message
			if (msgId == COMM_MAPPING_ID) {
				//Handle a message from the mapping thread
				dbgOutputLoc(DBG_LOC_COMM_IF_MAPPING);
			} else if (msgId == COMM_UART_ID) {
				//Handle input from the WiFly
				dbgOutputLoc(DBG_LOC_COMM_IF_UART);
				if (UNIT_TESTING) {
					commQueueReceiveTest(receivemsg);
				} else {
					//Parse that stuff
					fieldItem testFieldItem;
					unsigned char Source;
					unsigned char Dest;
					unsigned char MessageType[4];
					unsigned char SequenceNumber;
					int Checksum;
					
					if (jsonGetChecksum(receivemsg, &Checksum)) {
						//error
						dbgOutputLoc(DBG_LOC_BAD_ERROR - 5);
					}
					
					//Check Checksum
					unsigned char ChecksumStr[6];
					if (jsonGetChecksumString(receivemsg, ChecksumStr)) {
						//bad
					} else {
						int ChecksumStrChecksum = calcSimpleChecksum(ChecksumStr);// ChecksumStr[0] + ChecksumStr[1] + ChecksumStr[2] + ChecksumStr[3];
						int ChecksumDiff = calcSimpleChecksum(receivemsg) - ChecksumStrChecksum;
						if (ChecksumDiff == Checksum) {
							//Checksums check out
						} else {
							//bad
							NumBadChecksums++;
							dbgOutputVal(NumBadChecksums);
							dbgOutputLoc(DBG_LOC_BAD_ERROR - 7);
						}
					}
					
					//Check Sequence Number
					if (SequenceNumber == PreviousSequenceNumber + 1) {
						//This is good.
					} else if (PreviousSequenceNumber == 63 && SequenceNumber == 0) {
						//This is also good.
					} else {
						//This is bad
						NumDroppedMessages++;
						dbgOutputVal(NumDroppedMessages);
						dbgOutputLoc(DBG_LOC_BAD_ERROR - 6);
					}
					PreviousSequenceNumber = SequenceNumber;
					
					if (jsonGetSource(receivemsg, &Source)) {
						//error
						dbgOutputLoc(DBG_LOC_BAD_ERROR - 1);
					}else if (Source == 'v'){
						//Handle stuff from my test client
						dbgOutputVal(Source);
						int speed;
//                        Nop();
						//I store the speed in the checksum field because, at the time of writing, there is no working function to read from the data field
						if(jsonGetChecksum(receivemsg, &speed) == 0){
							//Send speed to Nav queue
							char navMsg[NAV_QUEUE_BUFFER_SIZE];
							navMsg[0] = speed & 0xff;
							navMsg[1] = NAV_MOTOR_SPEED_ID;
							navMsg[NAV_SOURCE_ID_IDX] = NAV_OTHER_ID;
							navMsg[NAV_CHECKSUM_IDX] = navCalculateChecksum(navMsg);
							dbgOutputVal(navMsg[0]);
							dbgOutputVal(navMsg[1]);
							dbgOutputVal(navMsg[2]);
							dbgOutputVal(navMsg[3]);
							navSendMsg(navMsg);
						}
					}
					
					if (jsonGetDestination(receivemsg, &Dest)) {
						//error
						dbgOutputLoc(DBG_LOC_BAD_ERROR - 2);
					}
					
					if (jsonGetMessageType(receivemsg, MessageType)) {
						//error
						dbgOutputLoc(DBG_LOC_BAD_ERROR - 3);
					}
					if (jsonGetSequenceNumber(receivemsg, &SequenceNumber)) {
						//error
						dbgOutputLoc(DBG_LOC_BAD_ERROR - 4);
					}
					if (jsonGetFieldItem(receivemsg, &testFieldItem)){
						//error
						dbgOutputLoc(DBG_LOC_BAD_ERROR-6);
					}
				}
			} else if (msgId == COMM_SEND_ID) {
				//Handle sending
				dbgOutputLoc(DBG_LOC_COMM_IF_SEND);
                unsigned char bufferToReadFrom[RECEIVE_BUFFER_SIZE];
                commSendMsgToSendQueue(bufferToReadFrom);
			} else if (msgId == COMM_OTHER_ID) {
				//Handle other stuff
				if (receivemsg[1] == NAV_MOTOR_SPEED_ID){
					//Handle motor speed server testing
					char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
					sprintf(testServerMsg, "*{\"S\":\"%c\",\"T\":\"%c\",\"M\":\"%c\",\"N\":%d,\"D\":[%d],\"C\":%d)~", 's', 'v', 's', 0, (int) receivemsg[0], (int) receivemsg[0]);
					int j;
					for (j = 0; j < strlen(testServerMsg); j++) {
						xQueueSendToBack(sendQueue, &testServerMsg[j], portMAX_DELAY);
						PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
					}
				}
			}
		}
	}
}

int calcSimpleChecksum(unsigned char* stringToCalculateFrom) {
    int checkSum = 0;
    int j = 0;
    while (stringToCalculateFrom[j] != '~' && stringToCalculateFrom[j] != '}' && stringToCalculateFrom[j] != '\0') {
        j++;
        if (j > 254) {
            break;
        }
    }
    j++;
    int i;
    for (i = 0; i < j; i++) {
        checkSum += stringToCalculateFrom[i];
    }
    return checkSum;
}

void readPublic(char* bufferToWriteTo, int MY_BUFFER_SIZE) {
    dbgOutputLoc(DBG_LOC_COMM_ENTER_READ);
    int count;
    //    DRV_HANDLE myUsartHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READ);
    for (count = 0; count < MY_BUFFER_SIZE; count++) {
        dbgOutputLoc(51);
        unsigned char receivedByte = PLIB_USART_ReceiverByteReceive(DRV_USART_INDEX_0);
        bufferToWriteTo[count] = receivedByte;

        //        dbgOutputVal(bufferToWriteTo[count]);
    }
    dbgOutputLoc(DBG_LOC_COMM_LEAVE_READ);
}

unsigned char bufferToWrite2[COMM_QUEUE_BUFFER_SIZE];
unsigned int bufferToWrite2Idx = 0;

void readPublic2(char* bufferToWriteTo, int MY_BUFFER_SIZE) {
    dbgOutputLoc(DBG_LOC_COMM_ENTER_READ);
    //    int count;
    //    DRV_HANDLE myUsartHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READ);
    //    for (count = 0; count < MY_BUFFER_SIZE; count++) {
    dbgOutputLoc(51);
    unsigned char receivedByte = PLIB_USART_ReceiverByteReceive(DRV_USART_INDEX_0);

    
    //Handle start and end bytes
    if (receivedByte == '~') {
        bufferToWrite2[COMM_SOURCE_ID_IDX] = (COMM_UART_ID & 0x00000003) << COMM_SOURCE_ID_OFFSET;
        bufferToWrite2[COMM_CHECKSUM_IDX] = commCalculateChecksum(bufferToWrite2);
        commSendMsgFromISR(bufferToWrite2);
    } else if (receivedByte == '*') {
        //        bufferToWrite2 = "";
        bufferToWrite2Idx = 0;
        bufferToWrite2[bufferToWrite2Idx] = receivedByte;
        bufferToWrite2Idx++;
    } else {
        bufferToWrite2[bufferToWrite2Idx] = receivedByte;
        bufferToWrite2Idx++;
    }

    //        dbgOutputVal(bufferToWriteTo[count]);
    //    }
    dbgOutputLoc(DBG_LOC_COMM_LEAVE_READ);
}

//This function begins reading when a start byte is found and stops reading when an end byte is found
//It writes into a buffer that is big enough to store an entire JSON message
//When an end byte is received, the buffer is parsed by a JSON parser, an object is created, and it is sent to the queue
unsigned char receiveFromWifiBuffer[RECEIVE_BUFFER_SIZE];
unsigned int receiveFromWifiBufferIdx = 0;
int readPublicIntoBuffer2(char bufferToWriteTo[COMM_QUEUE_BUFFER_SIZE]) {
    dbgOutputLoc(DBG_LOC_COMM_ENTER_READ);
    int count;
    //int max = RECEIVE_BUFFER_SIZE - receiveFromWifiBufferIdx;
    int finished = 0;
    for (count = 0; count < RECEIVE_BUFFER_SIZE; count++) {
        dbgOutputLoc(51);
        unsigned char receivedByte = PLIB_USART_ReceiverByteReceive(DRV_USART_INDEX_0);

        //Handle the received byte
        if (receiveFromWifiBufferIdx >= RECEIVE_BUFFER_SIZE) {
            //This is bad, you should not get here
            dbgOutputLoc(DBG_LOC_BAD_ERROR);
        } else if (receivedByte == '~') {
            //End byte
            receiveFromWifiBuffer[receiveFromWifiBufferIdx] = receivedByte;
            receiveFromWifiBufferIdx++;
            strcpy(bufferToWriteTo, receiveFromWifiBuffer);
            finished = 1;
        } else if (receivedByte == '*') {
            //Start byte
            //Reset the index
            receiveFromWifiBufferIdx = 0;
            receiveFromWifiBuffer[receiveFromWifiBufferIdx] = receivedByte;
            receiveFromWifiBufferIdx++;
        } else if (receivedByte == 0) {
            //null byte
        } else {
            //Put the byte in the queue
            receiveFromWifiBuffer[receiveFromWifiBufferIdx] = receivedByte;
            receiveFromWifiBufferIdx++;
        }
    }
    return finished;
    dbgOutputLoc(DBG_LOC_COMM_LEAVE_READ);
}

void writePublic(char* bufferToReadFrom) {
    dbgOutputLoc(DBG_LOC_COMM_ENTER_WRITE);
    int count = 0;
    dbgOutputLoc(DBG_LOC_COMM_WRITE_LOOP);
    PLIB_USART_TransmitterByteSend(USART_ID_1, bufferToReadFrom[count]); //CHECK ME FOR BUFFER ACTUALLY CONTAINS CORRECT INFORMATION
    count++;
    dbgOutputLoc(DBG_LOC_COMM_LEAVE_WRITE);
}

/*******************************************************************************
 End of File
 */
