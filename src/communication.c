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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "communication.h"
#include "communication_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

COMMUNICATION_DATA communicationData;

static QueueHandle_t commQueue;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void COMMUNICATION_Initialize ( void )

  Remarks:
    See prototype in communication.h.
 */

void COMMUNICATION_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    communicationData.state = COMMUNICATION_STATE_INIT;
    
    //Initialize the mapping queue
    commQueue = xQueueCreate(10, sizeof(unsigned char[COMM_QUEUE_BUFFER_SIZE]));
    if(commQueue == 0){
        dbgPauseAll();
    }

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void commSendMsgFromISR(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBackFromISR(commQueue, msg, NULL);
}

void commSendMsg(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]){
    BaseType_t xHigherPriorityTaskWoken =  pdTRUE;//pdFALSE;
    xQueueSendToBack(commQueue, msg, portMAX_DELAY);
}
    
unsigned char commCalculateChecksum(unsigned char msg[COMM_QUEUE_BUFFER_SIZE]){
    unsigned char sum = 0;
    unsigned int i;
    for (i = 0; i < COMM_QUEUE_BUFFER_SIZE - 1; i++){
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

void COMMUNICATION_Tasks ( void )
{
    dbgOutputLoc(DBG_LOC_COMM_ENTER);

    /* Check the application's current state. */
    switch ( communicationData.state )
    {
        /* Application's initial state. */
        case COMMUNICATION_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                communicationData.state = COMMUNICATION_STATE_SERVICE_TASKS;
            }
            break;
        }

        case COMMUNICATION_STATE_SERVICE_TASKS:
        {
            unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE];
            
            dbgOutputLoc(DBG_LOC_COMM_BEFORE_WHILE);
            while(1){
                //Block until a message is received
                dbgOutputLoc(DBG_LOC_COMM_BEFORE_RECEIVE);
                BaseType_t receiveCheck = xQueueReceive(commQueue, receivemsg, portMAX_DELAY);
                dbgOutputLoc(DBG_LOC_COMM_AFTER_RECEIVE);
                
                //Handle the message
                if(receiveCheck == pdTRUE){
                    //Convert the message into integer format
                    unsigned int receivemsgint0 = receivemsg[0] | (receivemsg[1] << 8) | (receivemsg[2] << 16) | (receivemsg[3] << 24);
                    unsigned int receivemsgint1 = receivemsg[4] | (receivemsg[5] << 8) | (receivemsg[6] << 16) | (receivemsg[7] << 24);
                    //Get the message ID
                    int msgId = (receivemsg[COMM_SOURCE_ID_IDX] & COMM_SOURCE_ID_MASK) >> COMM_SOURCE_ID_OFFSET;
                    //Handle a specific message
                    if (msgId == COMM_MAPPING_ID){
                        //Handle a message from the mapping thread
                        dbgOutputVal(receivemsg[0]);
                        dbgOutputVal(receivemsg[1]);
                        dbgOutputVal(receivemsg[2]);
                        dbgOutputVal(receivemsg[3]);
                        dbgOutputVal(receivemsg[4]);
                        dbgOutputVal(receivemsg[5]);
                        dbgOutputVal(receivemsg[6]);
                        dbgOutputVal(receivemsg[7]);
                    }else if (msgId == COMM_UART_ID){
                        //Handle input from the WiFly
                    }
                }
            }
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
