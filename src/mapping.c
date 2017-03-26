/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mapping.c

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


#include "mapping.h"
#include "mapping_public.h"
#include "ms2test.h"

MAPPING_DATA mappingData;

static QueueHandle_t mapQueue;

/*******************************************************************************
  Function:
    void MAPPING_Initialize ( void )

  Remarks:
    See prototype in mapping.h.
 */

void MAPPING_Initialize(void) {
    /* Place the App state machine in its initial state. */
    mappingData.state = MAPPING_STATE_INIT;

    //Initialize the mapping queue
    mapQueue = xQueueCreate(10, sizeof (unsigned char[MAP_QUEUE_BUFFER_SIZE]));
    if (mapQueue == 0) {
        dbgPauseAll();
    }
}

void mapSendMsgFromISR(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    xQueueSendToBackFromISR(mapQueue, msg, NULL);
}

void mapSendMsg(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE; //pdFALSE;
    xQueueSendToBack(mapQueue, msg, portMAX_DELAY);
}

unsigned char mapCalculateChecksum(unsigned char msg[MAP_QUEUE_BUFFER_SIZE]) {
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < MAP_QUEUE_BUFFER_SIZE - 1; i++) {
        sum += msg[i];
    }
    return sum;
}

/******************************************************************************
  Function:
    void MAPPING_Tasks ( void )

  Remarks:
    See prototype in mapping.h.
 */

void MAPPING_Tasks(void) {
    dbgOutputLoc(DBG_LOC_MAP_ENTER);
    unsigned char receivemsg[MAP_QUEUE_BUFFER_SIZE];

    dbgOutputLoc(DBG_LOC_MAP_BEFORE_WHILE);
    DRV_ADC_Open();
    while (1) {
        //Block until a message is received
        dbgOutputLoc(DBG_LOC_MAP_BEFORE_RECEIVE);
        BaseType_t receiveCheck = xQueueReceive(mapQueue, receivemsg, portMAX_DELAY);
        dbgOutputLoc(DBG_LOC_MAP_AFTER_RECEIVE);

        //Handle the message
        if (receiveCheck == pdTRUE) {
            //Convert the message into integer format
            unsigned int receivemsgint0 = receivemsg[0] | (receivemsg[1] << 8) | (receivemsg[2] << 16) | (receivemsg[3] << 24);
            unsigned int receivemsgint1 = receivemsg[4] | (receivemsg[5] << 8) | (receivemsg[6] << 16) | (receivemsg[7] << 24);
            unsigned int receivemsgint2 = receivemsg[8] | (receivemsg[9] << 8) | (receivemsg[10] << 16) | (receivemsg[11] << 24);
            unsigned int receivemsgint3 = receivemsg[12] | (receivemsg[13] << 8) | (receivemsg[14] << 16);
            //Get the message ID
            int msgId = (receivemsg[MAP_SOURCE_ID_IDX] & MAP_SOURCE_ID_MASK) >> MAP_SOURCE_ID_OFFSET;
            //Handle a specific message
            if (msgId == MAP_NAVIGATION_ID) {
                //Handle a message from the navigation thread
                unsigned char msg2[6];
                msg2[0] = 0x1f;
                msg2[1] = 0x2e;
                msg2[2] = 0x3d;
                msg2[3] = 0x4c;
                msg2[4] = 0x5b;
                msg2[COMM_SOURCE_ID_IDX] = (COMM_MAPPING_ID & 0x00000001) << COMM_SOURCE_ID_OFFSET;
                msg2[COMM_CHECKSUM_IDX] = commCalculateChecksum(msg2);
                commSendMsg(msg2);
            } else if (msgId == MAP_PIXY_CAM_ID) {
                //Handle input from the pixy cam
            } else if (msgId == MAP_ULTRASONIC_ID) {


                if (MS3DEMO) {
                    short rec = 0;
                    rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
                    char sonVal[RECEIVE_BUFFER_SIZE];

                    float son0Dist = ((float) rec) * 10000.0;
                    son0Dist = son0Dist - 7929.0;
                    son0Dist = son0Dist / 14986.0;
                    unsigned short son0Out = ((unsigned short) son0Dist);

                    sprintf(sonVal, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", son0Out, 0);
                    commSendMsgToSendQueue(sonVal);

                }
            } else if (msgId == MAP_IR_1_ID) {
                //Handle input from the first IR sensor
                short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
                char ir0Val[RECEIVE_BUFFER_SIZE];


                unsigned short ir0Out = 0;
                float ir0Numer = 7501.53;
                float ir0Exp = 1000.0 / 1009.0;
                float ir0Denom = powf(((float) rec), ir0Exp);
                ir0Out = ((unsigned short) (ir0Numer / ir0Denom));

                sprintf(ir0Val, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", rec, 1);
                commSendMsgToSendQueue(ir0Val);


            } else if (msgId == MAP_IR_2_ID) {
                short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
                char ir1Val[RECEIVE_BUFFER_SIZE];


                unsigned short ir1Out = 0;
                float ir1Numer = 7501.53;
                float ir1Exp = 1000.0 / 1009.0;
                float ir1Denom = powf(((float) rec), ir1Exp);
                ir1Out = (unsigned short) (ir1Numer / ir1Denom);

                sprintf(ir1Val, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", ir1Out, 2);
                commSendMsgToSendQueue(ir1Val);

                //Handle input from the second IR sensor
            } else if (msgId == MAP_MAPPING_TIMER_ID) {
                //Start sampling
                //                        dbgOutputVal(receivemsg[13]);

                unsigned char msg1[NAV_QUEUE_BUFFER_SIZE];
                msg1[0] = 0x5a;
                msg1[1] = 0x6c;
                msg1[NAV_SOURCE_ID_IDX] = (NAV_MAPPING_ID_SENSOR & 0x00000007) << NAV_SOURCE_ID_OFFSET;
                msg1[NAV_CHECKSUM_IDX] = navCalculateChecksum(msg1);
                navSendMsg(msg1);
            }
        }
    }
}



/*******************************************************************************
 End of File
 */
