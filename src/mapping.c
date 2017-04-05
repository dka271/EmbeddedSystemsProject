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
                
                // MERGE WITH DANIEL'S CODE TO MAKE THIS LINE LEGIT
                mappingData.rover_y_pos = 0;
                //////////////////////////////////////////////////
                
                commSendMsg(msg2);
            } else if (msgId == MAP_PIXY_CAM_ID) {
                
                
                  short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
                
                char pixyVal[RECEIVE_BUFFER_SIZE];
                Nop();
                sprintf(pixyVal, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", rec, 666);
                commSendMsgToSendQueue(pixyVal);
                
                
                
                
            } else if (msgId == MAP_ULTRASONIC_ID) {


//                if (MS3DEMO) {
                short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
                char sonVal[RECEIVE_BUFFER_SIZE];

//                float son0Dist = ((float) rec) * 10000.0;
//                son0Dist = son0Dist - 7929.0;
//                son0Dist = son0Dist / 14986.0;
                float son0Dist = ((float) rec) * 125.0;
                son0Dist = son0Dist - 2682.0;
                son0Dist = son0Dist * .0041951;
                unsigned short son0Out = ((unsigned short) son0Dist);


//                }
                
                float spreadAngle = M_PI/6.0;         // this is in radians
                // Each cell along the arc with radius son0Out must be incremented
                // To map this to the occupancy grid, it is necessary to find each of the relevant cells
                // The bounds and center line are easy because we have the angle of spread
                // The intermediate locations require some sort of value to calculate how much to increment each distance calculation
                // This incrementation value must scale with increasing radius value
                // One possible way:
                //      2 points along the arc that are 2 cm apart MUST be in adjacent cells
                //      However, they may fall in the same cell (but unlikely)
                //      One cell can be stored as history and be checked against 
                float angIncrement = 2.0/ (float)son0Out;        // this is in radians        // make sure this is cast correctly
                short numIncrements = son0Out * spreadAngle / 2.0;
                
                // set the first row and column to consider
                short prevR = (short)(mappingData.rover_y_pos - (float)son0Out * (float)sin(spreadAngle / 2.0)) / 2.0;
//                short prevC = (short)(cos(spreadAngle / 2)) / 2;
                short prevC = (short)((float)cos(spreadAngle / 2.0) * (float) son0Out) / 2.0;
                
                if (prevR < 0) prevR = 0;
                if (prevC < 0) prevC = 0;
                
                short currR = prevR;
                short currC = prevC;
                mappingData.OCCUPANCY_GRID[currR][currC] += 1;
//                mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][son0Out] += 1;

                short iFillArc;
                for (iFillArc = 1; iFillArc < numIncrements+1; ++iFillArc) {
                    currR = (short)(mappingData.rover_y_pos - (float)son0Out * sin( (spreadAngle / 2.0) - angIncrement*(float)iFillArc) ) / 2;
//                    currR = (short)(mappingData.rover_y_pos - (float)son0Out * sin(spreadAngle-angIncrement*(float)iFillArc) / 2) / 2;
                    currC = (short)(cos( (spreadAngle / (float)2)- angIncrement*(float)iFillArc) * (float)son0Out) / 2.0;
//                    currC = (short)(cos(spreadAngle-angIncrement*(float)iFillArc) / 2) / 2;

                    if (currR < 0) currR = 0;
                    if (currC < 0) currC = 0;
                    
                    //check if current cell is different from the previous cell
                    if (currR != prevR || currC != prevC) {
                        // increment existence probability
                        mappingData.OCCUPANCY_GRID[currR][currC] += 1;
                        //decrement every cell in front of the arc
                            //TODO
                    }
                    prevR = currR;
                    prevC = currC;
                }
                
                char gridOut[RECEIVE_BUFFER_SIZE];
                
                sprintf(gridOut, "*%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x~",
                        mappingData.OCCUPANCY_GRID[1][ 0 ], mappingData.OCCUPANCY_GRID[1][ 1 ], mappingData.OCCUPANCY_GRID[1][ 2 ],
                        mappingData.OCCUPANCY_GRID[1][ 3 ], mappingData.OCCUPANCY_GRID[1][ 4 ], mappingData.OCCUPANCY_GRID[1][ 5 ],
                        mappingData.OCCUPANCY_GRID[1][ 6 ], mappingData.OCCUPANCY_GRID[1][ 7 ], mappingData.OCCUPANCY_GRID[1][ 8 ],
                        mappingData.OCCUPANCY_GRID[1][ 9 ], mappingData.OCCUPANCY_GRID[1][ 10 ], mappingData.OCCUPANCY_GRID[1][ 11 ],
                        mappingData.OCCUPANCY_GRID[1][ 12 ], mappingData.OCCUPANCY_GRID[1][ 13 ], mappingData.OCCUPANCY_GRID[1][ 14 ],
                        mappingData.OCCUPANCY_GRID[1][ 15 ], mappingData.OCCUPANCY_GRID[1][ 16 ], mappingData.OCCUPANCY_GRID[1][ 17 ],
                        mappingData.OCCUPANCY_GRID[1][ 18 ], mappingData.OCCUPANCY_GRID[1][ 19 ], mappingData.OCCUPANCY_GRID[1][ 20 ],
                        mappingData.OCCUPANCY_GRID[1][ 21 ], mappingData.OCCUPANCY_GRID[1][ 22 ], mappingData.OCCUPANCY_GRID[1][ 23 ],
                        mappingData.OCCUPANCY_GRID[1][ 24 ], mappingData.OCCUPANCY_GRID[1][ 25 ], mappingData.OCCUPANCY_GRID[1][ 26 ],
                        mappingData.OCCUPANCY_GRID[1][ 27 ], mappingData.OCCUPANCY_GRID[1][ 28 ], mappingData.OCCUPANCY_GRID[1][ 29 ],
                        mappingData.OCCUPANCY_GRID[1][ 30 ], mappingData.OCCUPANCY_GRID[1][ 31 ], mappingData.OCCUPANCY_GRID[1][ 32 ],
                        mappingData.OCCUPANCY_GRID[1][ 33 ], mappingData.OCCUPANCY_GRID[1][ 34 ], mappingData.OCCUPANCY_GRID[1][ 35 ],
                        mappingData.OCCUPANCY_GRID[1][ 36 ], mappingData.OCCUPANCY_GRID[1][ 37 ], mappingData.OCCUPANCY_GRID[1][ 38 ],
                        mappingData.OCCUPANCY_GRID[1][ 39 ], mappingData.OCCUPANCY_GRID[1][ 40 ], mappingData.OCCUPANCY_GRID[1][ 41 ],
                        mappingData.OCCUPANCY_GRID[1][ 42 ], mappingData.OCCUPANCY_GRID[1][ 43 ], mappingData.OCCUPANCY_GRID[1][ 44 ],
                        mappingData.OCCUPANCY_GRID[1][ 45 ], mappingData.OCCUPANCY_GRID[1][ 46 ], mappingData.OCCUPANCY_GRID[1][ 47 ],
                        mappingData.OCCUPANCY_GRID[1][ 48 ], mappingData.OCCUPANCY_GRID[1][ 49 ], mappingData.OCCUPANCY_GRID[1][ 50 ],
                        mappingData.OCCUPANCY_GRID[1][ 51 ], mappingData.OCCUPANCY_GRID[1][ 52 ], mappingData.OCCUPANCY_GRID[1][ 53 ],
                        mappingData.OCCUPANCY_GRID[1][ 54 ], mappingData.OCCUPANCY_GRID[1][ 55 ], mappingData.OCCUPANCY_GRID[1][ 56 ],
                        mappingData.OCCUPANCY_GRID[1][ 57 ], mappingData.OCCUPANCY_GRID[1][ 58 ], mappingData.OCCUPANCY_GRID[1][ 59 ],
                        mappingData.OCCUPANCY_GRID[1][ 60 ], mappingData.OCCUPANCY_GRID[1][ 61 ], mappingData.OCCUPANCY_GRID[1][ 62 ],
                        mappingData.OCCUPANCY_GRID[1][ 63 ], mappingData.OCCUPANCY_GRID[1][ 64 ], mappingData.OCCUPANCY_GRID[1][ 65 ],
                        mappingData.OCCUPANCY_GRID[1][ 66 ], mappingData.OCCUPANCY_GRID[1][ 67 ], mappingData.OCCUPANCY_GRID[1][ 68 ],
                        mappingData.OCCUPANCY_GRID[1][ 69 ], mappingData.OCCUPANCY_GRID[1][ 70 ], mappingData.OCCUPANCY_GRID[1][ 71 ],
                        mappingData.OCCUPANCY_GRID[1][ 72 ], mappingData.OCCUPANCY_GRID[1][ 73 ], mappingData.OCCUPANCY_GRID[1][ 74 ],
                        mappingData.OCCUPANCY_GRID[1][ 75 ], mappingData.OCCUPANCY_GRID[1][ 76 ], mappingData.OCCUPANCY_GRID[1][ 77 ],
                        mappingData.OCCUPANCY_GRID[1][ 78 ], mappingData.OCCUPANCY_GRID[1][ 79 ], mappingData.OCCUPANCY_GRID[1][ 80 ],
                        mappingData.OCCUPANCY_GRID[1][ 81 ], mappingData.OCCUPANCY_GRID[1][ 82 ], mappingData.OCCUPANCY_GRID[1][ 83 ],
                        mappingData.OCCUPANCY_GRID[1][ 84 ], mappingData.OCCUPANCY_GRID[1][ 85 ], mappingData.OCCUPANCY_GRID[1][ 86 ],
                        mappingData.OCCUPANCY_GRID[1][ 87 ], mappingData.OCCUPANCY_GRID[1][ 88 ], mappingData.OCCUPANCY_GRID[1][ 89 ],
                        mappingData.OCCUPANCY_GRID[1][ 90 ], mappingData.OCCUPANCY_GRID[1][ 91 ], mappingData.OCCUPANCY_GRID[1][ 92 ],
                        mappingData.OCCUPANCY_GRID[1][ 93 ], mappingData.OCCUPANCY_GRID[1][ 94 ], mappingData.OCCUPANCY_GRID[1][ 95 ],
                        mappingData.OCCUPANCY_GRID[1][ 96]);
                
//                sprintf(gridOut, "US value=%d   %d",son0Out, rec);
//                commSendMsgToSendQueue(gridOut);
                
                                
            } else if (msgId == MAP_IR_2_ID) {
                //Handle input from the first IR sensor
                short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
                char ir0Val[RECEIVE_BUFFER_SIZE];


                unsigned short ir0Out = 0;
//                float ir0Numer = 7501.53;
//                float ir0Exp = 1000.0 / 1009.0;
//                float ir0Denom = powf(((float) rec), ir0Exp);
                float ir0Numer = 2697.74;
                float ir0Exp = 1000.0 / 1167.0;
                float ir0Denom = powf(((float) rec), ir0Exp);
                ir0Out = ((unsigned short) (ir0Numer / ir0Denom));
                
                // update occupancy grid using y position           How much to increment by?
                if (mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][ir0Out/2] != 0xffff)
                    mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][ir0Out/2] += 1;//(5<<8);
                short i;
                for (i = 0; i <ir0Out/2; ++i) {                     //ir0Out is divided by 2 because cells are 2cm wide
                    if (mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][i] != 0)
                        mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][i] -= 1;//(5<<8);
                }

                char gridOut[RECEIVE_BUFFER_SIZE];
                int iOMG;
//                for (int i = 0; i < ) {
                    sprintf(gridOut, "*%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x~",
                            mappingData.OCCUPANCY_GRID[1][ 0 ], mappingData.OCCUPANCY_GRID[0][ 1 ], mappingData.OCCUPANCY_GRID[0][ 2 ],
                            mappingData.OCCUPANCY_GRID[0][ 3 ], mappingData.OCCUPANCY_GRID[0][ 4 ], mappingData.OCCUPANCY_GRID[0][ 5 ],
                            mappingData.OCCUPANCY_GRID[0][ 6 ], mappingData.OCCUPANCY_GRID[0][ 7 ], mappingData.OCCUPANCY_GRID[0][ 8 ],
                            mappingData.OCCUPANCY_GRID[0][ 9 ], mappingData.OCCUPANCY_GRID[0][ 10 ], mappingData.OCCUPANCY_GRID[0][ 11 ],
                            mappingData.OCCUPANCY_GRID[0][ 12 ], mappingData.OCCUPANCY_GRID[0][ 13 ], mappingData.OCCUPANCY_GRID[0][ 14 ],
                            mappingData.OCCUPANCY_GRID[0][ 15 ], mappingData.OCCUPANCY_GRID[0][ 16 ], mappingData.OCCUPANCY_GRID[0][ 17 ],
                            mappingData.OCCUPANCY_GRID[0][ 18 ], mappingData.OCCUPANCY_GRID[0][ 19 ], mappingData.OCCUPANCY_GRID[0][ 20 ],
                            mappingData.OCCUPANCY_GRID[0][ 21 ], mappingData.OCCUPANCY_GRID[0][ 22 ], mappingData.OCCUPANCY_GRID[0][ 23 ],
                            mappingData.OCCUPANCY_GRID[0][ 24 ], mappingData.OCCUPANCY_GRID[0][ 25 ], mappingData.OCCUPANCY_GRID[0][ 26 ],
                            mappingData.OCCUPANCY_GRID[0][ 27 ], mappingData.OCCUPANCY_GRID[0][ 28 ], mappingData.OCCUPANCY_GRID[0][ 29 ],
                            mappingData.OCCUPANCY_GRID[0][ 30 ], mappingData.OCCUPANCY_GRID[0][ 31 ], mappingData.OCCUPANCY_GRID[0][ 32 ],
                            mappingData.OCCUPANCY_GRID[0][ 33 ], mappingData.OCCUPANCY_GRID[0][ 34 ], mappingData.OCCUPANCY_GRID[0][ 35 ],
                            mappingData.OCCUPANCY_GRID[0][ 36 ], mappingData.OCCUPANCY_GRID[0][ 37 ], mappingData.OCCUPANCY_GRID[0][ 38 ],
                            mappingData.OCCUPANCY_GRID[0][ 39 ], mappingData.OCCUPANCY_GRID[0][ 40 ], mappingData.OCCUPANCY_GRID[0][ 41 ],
                            mappingData.OCCUPANCY_GRID[0][ 42 ], mappingData.OCCUPANCY_GRID[0][ 43 ], mappingData.OCCUPANCY_GRID[0][ 44 ],
                            mappingData.OCCUPANCY_GRID[0][ 45 ], mappingData.OCCUPANCY_GRID[0][ 46 ], mappingData.OCCUPANCY_GRID[0][ 47 ],
                            mappingData.OCCUPANCY_GRID[0][ 48 ], mappingData.OCCUPANCY_GRID[0][ 49 ], mappingData.OCCUPANCY_GRID[0][ 50 ],
                            mappingData.OCCUPANCY_GRID[0][ 51 ], mappingData.OCCUPANCY_GRID[0][ 52 ], mappingData.OCCUPANCY_GRID[0][ 53 ],
                            mappingData.OCCUPANCY_GRID[0][ 54 ], mappingData.OCCUPANCY_GRID[0][ 55 ], mappingData.OCCUPANCY_GRID[0][ 56 ],
                            mappingData.OCCUPANCY_GRID[0][ 57 ], mappingData.OCCUPANCY_GRID[0][ 58 ], mappingData.OCCUPANCY_GRID[0][ 59 ],
                            mappingData.OCCUPANCY_GRID[0][ 60 ], mappingData.OCCUPANCY_GRID[0][ 61 ], mappingData.OCCUPANCY_GRID[0][ 62 ],
                            mappingData.OCCUPANCY_GRID[0][ 63 ], mappingData.OCCUPANCY_GRID[0][ 64 ], mappingData.OCCUPANCY_GRID[0][ 65 ],
                            mappingData.OCCUPANCY_GRID[0][ 66 ], mappingData.OCCUPANCY_GRID[0][ 67 ], mappingData.OCCUPANCY_GRID[0][ 68 ],
                            mappingData.OCCUPANCY_GRID[0][ 69 ], mappingData.OCCUPANCY_GRID[0][ 70 ], mappingData.OCCUPANCY_GRID[0][ 71 ],
                            mappingData.OCCUPANCY_GRID[0][ 72 ], mappingData.OCCUPANCY_GRID[0][ 73 ], mappingData.OCCUPANCY_GRID[0][ 74 ],
                            mappingData.OCCUPANCY_GRID[0][ 75 ], mappingData.OCCUPANCY_GRID[0][ 76 ], mappingData.OCCUPANCY_GRID[0][ 77 ],
                            mappingData.OCCUPANCY_GRID[0][ 78 ], mappingData.OCCUPANCY_GRID[0][ 79 ], mappingData.OCCUPANCY_GRID[0][ 80 ],
                            mappingData.OCCUPANCY_GRID[0][ 81 ], mappingData.OCCUPANCY_GRID[0][ 82 ], mappingData.OCCUPANCY_GRID[0][ 83 ],
                            mappingData.OCCUPANCY_GRID[0][ 84 ], mappingData.OCCUPANCY_GRID[0][ 85 ], mappingData.OCCUPANCY_GRID[0][ 86 ],
                            mappingData.OCCUPANCY_GRID[0][ 87 ], mappingData.OCCUPANCY_GRID[0][ 88 ], mappingData.OCCUPANCY_GRID[0][ 89 ],
                            mappingData.OCCUPANCY_GRID[0][ 90 ], mappingData.OCCUPANCY_GRID[0][ 91 ], mappingData.OCCUPANCY_GRID[0][ 92 ],
                            mappingData.OCCUPANCY_GRID[0][ 93 ], mappingData.OCCUPANCY_GRID[0][ 94 ], mappingData.OCCUPANCY_GRID[0][ 95 ],
                            mappingData.OCCUPANCY_GRID[0][ 96]);
                
//                    commSendMsgToSendQueue(gridOut);
//                }
                                            
            } 
            else if (msgId == MAP_MAPPING_TIMER_ID) {
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
