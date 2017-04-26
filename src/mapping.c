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

//Private Structures
MAPPING_DATA mappingData;
object tempObjectList[MAX_NUMBER_OF_OBJECTS];
unsigned char tempObjectListTop = 0;
object finalObjectList[MAX_NUMBER_OF_OBJECTS];
unsigned char finalObjectListTop = 0;
object deletedObjectList[MAX_NUMBER_OF_DELETED_OBJECTS];
unsigned char deletedObjectListTop = 0;
object roverList[NUMBER_OF_ROVERS];
unsigned char roverListTop = 0;

unsigned short son0Out = 0;
unsigned short ir0Out = 0;

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

void mapSetYPosition(float yPos){
    mappingData.rover_y_pos = ((short) yPos);
}

// get the probability of a rover at a given occupancy grid index
// use only the 7 least significant bits of the return char
unsigned char getRoverVal(short rowIndex, short columnIndex) {
    int tempVal = mappingData.OCCUPANCY_GRID[rowIndex][columnIndex];
    return (tempVal >> (short)8) & 0x7f;
}

//get the probability of an obstacle at a given occupancy grid index
// all of the return char is relevant
unsigned char getObstacleVal(short rowIndex, short columnIndex) {
    short tempVal = mappingData.OCCUPANCY_GRID[rowIndex][columnIndex];
    return (unsigned char)((tempVal) & 0x00ff);
}

//get whether grid space has been traversed. For use only by the floodfill and object creation functions
// use only the least significant bit of the return char
unsigned char getPaintVal(short rowIndex, short columnIndex) {
    short tempVal = mappingData.OCCUPANCY_GRID[rowIndex][columnIndex];
    return (tempVal >> (short)15);
}

void incrementObstacleVal(short rowIndex, short columnIndex, short inc) {
    if ((mappingData.OCCUPANCY_GRID[rowIndex][columnIndex] & 0xff ) < (0xff-inc)) {
        mappingData.OCCUPANCY_GRID[rowIndex][columnIndex] += inc;
    }
}

void decrementObstacleVal(short rowIndex, short columnIndex, short inc) {
    mappingData.OCCUPANCY_GRID[rowIndex][columnIndex] -= inc;
}

void incrementRoverVal(short rowIndex, short columnIndex, short inc) {
    if ((mappingData.OCCUPANCY_GRID[rowIndex][columnIndex] >> 8) < (0xff - inc)) {
        mappingData.OCCUPANCY_GRID[rowIndex][columnIndex] += (inc<<8);
        printOccupancyDebug(mappingData.OCCUPANCY_GRID[rowIndex][columnIndex]);
    }
}

void decrementRoverVal(short rowIndex, short columnIndex, short dec) {
    if ((mappingData.OCCUPANCY_GRID[rowIndex][columnIndex] >> 8) > dec) {
        mappingData.OCCUPANCY_GRID[rowIndex][columnIndex] -= (dec<<8);
    }
}
    
char gridOut[RECEIVE_BUFFER_SIZE]; 

void printOccupancyDebug(short inVal) {
    sprintf(gridOut,"yVal=%4x", mappingData.rover_y_pos);
    commSendMsgToWifiQueue(gridOut);
}

void printOccupancyGridToUART() {
    sprintf(gridOut,"yVal=%3d", mappingData.rover_y_pos);
    commSendMsgToWifiQueue(gridOut);
//    int iOMG;
//    for (iOMG = 0; iOMG < 1; ++iOMG) {
        /*sprintf(gridOut, "*%03d:%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|~", iOMG,
                mappingData.OCCUPANCY_GRID[iOMG][ 0 ], mappingData.OCCUPANCY_GRID[iOMG][ 1 ], mappingData.OCCUPANCY_GRID[iOMG][ 2 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 3 ], mappingData.OCCUPANCY_GRID[iOMG][ 4 ], mappingData.OCCUPANCY_GRID[iOMG][ 5 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 6 ], mappingData.OCCUPANCY_GRID[iOMG][ 7 ], mappingData.OCCUPANCY_GRID[iOMG][ 8 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 9 ], mappingData.OCCUPANCY_GRID[iOMG][ 10 ], mappingData.OCCUPANCY_GRID[iOMG][ 11 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 12 ], mappingData.OCCUPANCY_GRID[iOMG][ 13 ], mappingData.OCCUPANCY_GRID[iOMG][ 14 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 15 ], mappingData.OCCUPANCY_GRID[iOMG][ 16 ], mappingData.OCCUPANCY_GRID[iOMG][ 17 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 18 ], mappingData.OCCUPANCY_GRID[iOMG][ 19 ], mappingData.OCCUPANCY_GRID[iOMG][ 20 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 21 ], mappingData.OCCUPANCY_GRID[iOMG][ 22 ], mappingData.OCCUPANCY_GRID[iOMG][ 23 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 24 ], mappingData.OCCUPANCY_GRID[iOMG][ 25 ], mappingData.OCCUPANCY_GRID[iOMG][ 26 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 27 ], mappingData.OCCUPANCY_GRID[iOMG][ 28 ], mappingData.OCCUPANCY_GRID[iOMG][ 29 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 30 ], mappingData.OCCUPANCY_GRID[iOMG][ 31 ], mappingData.OCCUPANCY_GRID[iOMG][ 32 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 33 ], mappingData.OCCUPANCY_GRID[iOMG][ 34 ], mappingData.OCCUPANCY_GRID[iOMG][ 35 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 36 ], mappingData.OCCUPANCY_GRID[iOMG][ 37 ], mappingData.OCCUPANCY_GRID[iOMG][ 38 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 39 ], mappingData.OCCUPANCY_GRID[iOMG][ 40 ], mappingData.OCCUPANCY_GRID[iOMG][ 41 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 42 ], mappingData.OCCUPANCY_GRID[iOMG][ 43 ], mappingData.OCCUPANCY_GRID[iOMG][ 44 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 45 ], mappingData.OCCUPANCY_GRID[iOMG][ 46 ], mappingData.OCCUPANCY_GRID[iOMG][ 47 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 48 ], mappingData.OCCUPANCY_GRID[iOMG][ 49 ], mappingData.OCCUPANCY_GRID[iOMG][ 50 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 51 ], mappingData.OCCUPANCY_GRID[iOMG][ 52 ], mappingData.OCCUPANCY_GRID[iOMG][ 53 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 54 ], mappingData.OCCUPANCY_GRID[iOMG][ 55 ], mappingData.OCCUPANCY_GRID[iOMG][ 56 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 57 ], mappingData.OCCUPANCY_GRID[iOMG][ 58 ], mappingData.OCCUPANCY_GRID[iOMG][ 59 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 60 ], mappingData.OCCUPANCY_GRID[iOMG][ 61 ], mappingData.OCCUPANCY_GRID[iOMG][ 62 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 63 ], mappingData.OCCUPANCY_GRID[iOMG][ 64 ], mappingData.OCCUPANCY_GRID[iOMG][ 65 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 66 ], mappingData.OCCUPANCY_GRID[iOMG][ 67 ], mappingData.OCCUPANCY_GRID[iOMG][ 68 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 69 ], mappingData.OCCUPANCY_GRID[iOMG][ 70 ], mappingData.OCCUPANCY_GRID[iOMG][ 71 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 72 ], mappingData.OCCUPANCY_GRID[iOMG][ 73 ], mappingData.OCCUPANCY_GRID[iOMG][ 74 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 75 ], mappingData.OCCUPANCY_GRID[iOMG][ 76 ], mappingData.OCCUPANCY_GRID[iOMG][ 77 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 78 ], mappingData.OCCUPANCY_GRID[iOMG][ 79 ], mappingData.OCCUPANCY_GRID[iOMG][ 80 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 81 ], mappingData.OCCUPANCY_GRID[iOMG][ 82 ], mappingData.OCCUPANCY_GRID[iOMG][ 83 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 84 ], mappingData.OCCUPANCY_GRID[iOMG][ 85 ], mappingData.OCCUPANCY_GRID[iOMG][ 86 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 87 ], mappingData.OCCUPANCY_GRID[iOMG][ 88 ], mappingData.OCCUPANCY_GRID[iOMG][ 89 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 90 ], mappingData.OCCUPANCY_GRID[iOMG][ 91 ], mappingData.OCCUPANCY_GRID[iOMG][ 92 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 93 ], mappingData.OCCUPANCY_GRID[iOMG][ 94 ], mappingData.OCCUPANCY_GRID[iOMG][ 95 ],
                mappingData.OCCUPANCY_GRID[iOMG][ 96]);*/
//        }   
}

//currently checks ONLY for obstacles
/*
void floodFillCore(short r, short c, short currColor) {
    if (mappingData.OCCUPANCY_GRID[r][c] & MAP_OBSTACLE_BITMASK > MAP_OBJECT_THRESHOLD              //threshold calculation to detect an obstacle
            && mappingData.FLOOD_FILL_PAINT[r][c] == 0) {                                                     //make sure you haven't traversed this cell before
        mappingData.FLOOD_FILL_PAINT[r][c] = currColor;
        
        if (r < 126)
            floodFillCore(r+1, c, currColor);
        
        if (r > 1)
            floodFillCore(r-1, c, currColor);
        
        if (c < 96)
            floodFillCore(r, c+1, currColor);
        
        if (c > 1)
            floodFillCore(r, c-1, currColor);
    }
    // This case my not provide any optimization at all
    // else if (occupancyGrid[r][c] & obstacleBitmask < obstacleThreshold
            // && paint[r][c] == 0) {                                    // since this cell is not an obstacle, we can mark it as traversed
        // paint[r][c] = 0b11111111;
    // }
    else {
        // the current cell (occupancyGrid[r][c]) is painted some color already, either currColor or something else
        // so we do fuck all
        return;
    }
}

void floodFillParser() {
    int c, r;
    short currColor = 1;        //could be a byte, start at 1, 0=non-obstacle(background) cell
    for (c = 0; c < 97; ++c) {
        for (r = 0; r < 127; ++r) {
            floodFillCore((short)r,(short)c, currColor);
            
            //now try to template match while the current color is completed
            //templateMatch(r,c,currColor)
            
            //move on to the next color
            currColor += 1;
        }
    }
}*/

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
                
                
//                  short rec = 0;
//                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
                
//                char pixyVal[RECEIVE_BUFFER_SIZE];
//                Nop();
//                sprintf(pixyVal, "*{\"S\":\"s\",\"T\":\"v\",\"M\":\"s\",\"N\":0,\"D\":[%d,%d],\"C\":123}~\n\r", rec, 666);
//                commSendMsgToSendQueue(pixyVal);
                
                
                
                
            }else if(msgId == MAP_US_VAL){
                
                short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));

//                float son0Dist = ((float) rec) * 10000.0;
//                son0Dist = son0Dist - 7929.0;
//                son0Dist = son0Dist / 14986.0;
//                son0Out = ((unsigned short) son0Dist);
                
//                float son0Dist = ((float) rec) * 125.0;
//                son0Dist = son0Dist - 2682.0;
//                son0Dist = son0Dist * .0041951;
//                son0Out = ((unsigned short) son0Dist);//////////////////////
                
                
//                float son0Dist = ((float) rec) * 10000.0;
//                son0Dist -= 41.0;
//                son0Dist = son0Dist / 14231.0;
//                son0Out = ((unsigned short) son0Dist);
                
                float son0Dist = ((float) rec) - 41.0;
                son0Dist = son0Dist / 1.4231;
                son0Out = ((unsigned short) son0Dist);
                son0Out -= 5;
                

//                }
                
                float spreadAngle = 0.4522*2;         // this is in radians, total spread of US sensor
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
                incrementObstacleVal(currR, currC, 1);
//                mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][son0Out] += 1;

                
                
                
                
                
                short iFillArc;
                for (iFillArc = 1; iFillArc < numIncrements+1; ++iFillArc) {
                    currR = (short)(mappingData.rover_y_pos - (float)son0Out * sin( (spreadAngle / 2.0) - angIncrement*(float)iFillArc) ) / 2;
//                    currR = (short)(mappingData.rover_y_pos - (float)son0Out * sin(spreadAngle-angIncrement*(float)iFillArc) / 2) / 2;
                    currC = (short)(cos( (spreadAngle / (float)2)- angIncrement*(float)iFillArc) * (float)son0Out) / 2.0;
//                    currC = (short)(cos(spreadAngle-angIncrement*(float)iFillArc) / 2) / 2;

//                    if (currR < 0) currR = 0;
//                    if (currC < 0) currC = 0;
                    
                    //check if current cell is different from the previous cell
                    if (currR != prevR || currC != prevC) {
                        // increment existence probability
                        if ( currR >= 0 && currC >= 0) {
                            incrementObstacleVal(currR, currC, 1);
                        }
                        //decrement every cell in front of the arc
                            //TODO
                    }
                    prevR = currR;
                    prevC = currC;
                }
                
                
                printOccupancyGridToUART();

            }
            else if (msgId == MAP_IR_VAL){
                short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
//                char ir0Val[RECEIVE_BUFFER_SIZE];
                
                
//                float ir0Numer = 7501.53;
//                float ir0Exp = 1000.0 / 1009.0;
//                float ir0Denom = powf(((float) rec), ir0Exp);
//                ir0Out = ((unsigned short) (ir0Numer / ir0Denom));
//                unsigned char irVal[SEND_QUEUE_BUFFER_SIZE];
                
                
                float ir0Numer = 40538.1;
                ir0Numer *= 1.1;//
                float ir0Exp = 1000.0 / 797.0;
                float ir0Denom = powf(((float) rec), ir0Exp);
                ir0Out = ((unsigned short) (ir0Numer / ir0Denom));                
                
                
                if (mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][ir0Out/2] != 0xffff)
                    incrementRoverVal(mappingData.rover_y_pos/2, ir0Out/2, 5);
                
                short i;
                for (i = 0; i <ir0Out/2; ++i) {                     //ir0Out is divided by 2 because cells are 2cm wide
                    if (mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][i] != 0)
                        decrementRoverVal(mappingData.rover_y_pos/2, i, 1);
                }

//                float ir0Numer = 467252.0;
//                float ir0Exp = 1000.0 / 517.0;
//                float ir0Denom = powf(((float) rec), ir0Exp);
//                ir0Out = ((unsigned short) (ir0Numer / ir0Denom));
//                unsigned char irVal[SEND_QUEUE_BUFFER_SIZE];
                
                
//                float ir0Numer = 2697.74;
//                float ir0Exp = 1000.0 / 1167.0;
//                float ir0Denom = powf(((float) rec), ir0Exp);
//                ir0Out = ((unsigned short) ((2.1 * ir0Numer )/ ir0Denom));
//                unsigned char irVal[SEND_QUEUE_BUFFER_SIZE];
            
            
            }else if (msgId == MAP_ULTRASONIC_ID && 0) {


//                if (MS3DEMO) {
                short rec = 0;
                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
//                char sonVal[RECEIVE_BUFFER_SIZE];

//                float son0Dist = ((float) rec) * 10000.0;
//                son0Dist = son0Dist - 7929.0;
//                son0Dist = son0Dist / 14986.0;
                float son0Dist = ((float) rec) * 125.0;
                son0Dist = son0Dist - 2682.0;
                son0Dist = son0Dist * .0041951;
                unsigned short son0Out = ((unsigned short) son0Dist);


//                }
                
                float spreadAngle = 0.4522*2;         // this is in radians, total spread of US sensor
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
                incrementObstacleVal(currR, currC, 1);
//                mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][son0Out] += 1;

                short iFillArc;
                for (iFillArc = 1; iFillArc < numIncrements+1; ++iFillArc) {
                    currR = (short)(mappingData.rover_y_pos - (float)son0Out * sin( (spreadAngle / 2.0) - angIncrement*(float)iFillArc) ) / 2;
//                    currR = (short)(mappingData.rover_y_pos - (float)son0Out * sin(spreadAngle-angIncrement*(float)iFillArc) / 2) / 2;
                    currC = (short)(cos( (spreadAngle / (float)2)- angIncrement*(float)iFillArc) * (float)son0Out) / 2.0;
//                    currC = (short)(cos(spreadAngle-angIncrement*(float)iFillArc) / 2) / 2;

//                    if (currR < 0) currR = 0;
//                    if (currC < 0) currC = 0;
                    
                    //check if current cell is different from the previous cell
                    if (currR != prevR || currC != prevC) {
                        // increment existence probability
                        if ( currR >= 0 && currC >= 0) {
                            incrementObstacleVal(currR, currC, 1);
                        }
                        //decrement every cell in front of the arc
                            //TODO
                    }
                    prevR = currR;
                    prevC = currC;
                }
                
//                char gridOut[RECEIVE_BUFFER_SIZE];
//                int iOMG, i2;
//                for (iOMG = 0; iOMG < MAP_OCCUPANCY_ROWS; ++iOMG) {
//                    sprintf(gridOut, "*%03d:%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x~", iOMG,
//                            mappingData.OCCUPANCY_GRID[iOMG][ 0 ], mappingData.OCCUPANCY_GRID[iOMG][ 1 ], mappingData.OCCUPANCY_GRID[iOMG][ 2 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 3 ], mappingData.OCCUPANCY_GRID[iOMG][ 4 ], mappingData.OCCUPANCY_GRID[iOMG][ 5 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 6 ], mappingData.OCCUPANCY_GRID[iOMG][ 7 ], mappingData.OCCUPANCY_GRID[iOMG][ 8 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 9 ], mappingData.OCCUPANCY_GRID[iOMG][ 10 ], mappingData.OCCUPANCY_GRID[iOMG][ 11 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 12 ], mappingData.OCCUPANCY_GRID[iOMG][ 13 ], mappingData.OCCUPANCY_GRID[iOMG][ 14 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 15 ], mappingData.OCCUPANCY_GRID[iOMG][ 16 ], mappingData.OCCUPANCY_GRID[iOMG][ 17 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 18 ], mappingData.OCCUPANCY_GRID[iOMG][ 19 ], mappingData.OCCUPANCY_GRID[iOMG][ 20 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 21 ], mappingData.OCCUPANCY_GRID[iOMG][ 22 ], mappingData.OCCUPANCY_GRID[iOMG][ 23 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 24 ], mappingData.OCCUPANCY_GRID[iOMG][ 25 ], mappingData.OCCUPANCY_GRID[iOMG][ 26 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 27 ], mappingData.OCCUPANCY_GRID[iOMG][ 28 ], mappingData.OCCUPANCY_GRID[iOMG][ 29 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 30 ], mappingData.OCCUPANCY_GRID[iOMG][ 31 ], mappingData.OCCUPANCY_GRID[iOMG][ 32 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 33 ], mappingData.OCCUPANCY_GRID[iOMG][ 34 ], mappingData.OCCUPANCY_GRID[iOMG][ 35 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 36 ], mappingData.OCCUPANCY_GRID[iOMG][ 37 ], mappingData.OCCUPANCY_GRID[iOMG][ 38 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 39 ], mappingData.OCCUPANCY_GRID[iOMG][ 40 ], mappingData.OCCUPANCY_GRID[iOMG][ 41 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 42 ], mappingData.OCCUPANCY_GRID[iOMG][ 43 ], mappingData.OCCUPANCY_GRID[iOMG][ 44 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 45 ], mappingData.OCCUPANCY_GRID[iOMG][ 46 ], mappingData.OCCUPANCY_GRID[iOMG][ 47 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 48 ], mappingData.OCCUPANCY_GRID[iOMG][ 49 ], mappingData.OCCUPANCY_GRID[iOMG][ 50 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 51 ], mappingData.OCCUPANCY_GRID[iOMG][ 52 ], mappingData.OCCUPANCY_GRID[iOMG][ 53 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 54 ], mappingData.OCCUPANCY_GRID[iOMG][ 55 ], mappingData.OCCUPANCY_GRID[iOMG][ 56 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 57 ], mappingData.OCCUPANCY_GRID[iOMG][ 58 ], mappingData.OCCUPANCY_GRID[iOMG][ 59 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 60 ], mappingData.OCCUPANCY_GRID[iOMG][ 61 ], mappingData.OCCUPANCY_GRID[iOMG][ 62 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 63 ], mappingData.OCCUPANCY_GRID[iOMG][ 64 ], mappingData.OCCUPANCY_GRID[iOMG][ 65 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 66 ], mappingData.OCCUPANCY_GRID[iOMG][ 67 ], mappingData.OCCUPANCY_GRID[iOMG][ 68 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 69 ], mappingData.OCCUPANCY_GRID[iOMG][ 70 ], mappingData.OCCUPANCY_GRID[iOMG][ 71 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 72 ], mappingData.OCCUPANCY_GRID[iOMG][ 73 ], mappingData.OCCUPANCY_GRID[iOMG][ 74 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 75 ], mappingData.OCCUPANCY_GRID[iOMG][ 76 ], mappingData.OCCUPANCY_GRID[iOMG][ 77 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 78 ], mappingData.OCCUPANCY_GRID[iOMG][ 79 ], mappingData.OCCUPANCY_GRID[iOMG][ 80 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 81 ], mappingData.OCCUPANCY_GRID[iOMG][ 82 ], mappingData.OCCUPANCY_GRID[iOMG][ 83 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 84 ], mappingData.OCCUPANCY_GRID[iOMG][ 85 ], mappingData.OCCUPANCY_GRID[iOMG][ 86 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 87 ], mappingData.OCCUPANCY_GRID[iOMG][ 88 ], mappingData.OCCUPANCY_GRID[iOMG][ 89 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 90 ], mappingData.OCCUPANCY_GRID[iOMG][ 91 ], mappingData.OCCUPANCY_GRID[iOMG][ 92 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 93 ], mappingData.OCCUPANCY_GRID[iOMG][ 94 ], mappingData.OCCUPANCY_GRID[iOMG][ 95 ],
//                            mappingData.OCCUPANCY_GRID[iOMG][ 96]);

//                    sprintf(gridOut, "*%03d :%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X~", iOMG,
//                            getObstacleVal(iOMG, 0 ), getObstacleVal(iOMG, 1 ), getObstacleVal(iOMG, 2 ),
//                            getObstacleVal(iOMG, 3 ), getObstacleVal(iOMG, 4 ), getObstacleVal(iOMG, 5 ),
//                            getObstacleVal(iOMG, 6 ), getObstacleVal(iOMG, 7 ), getObstacleVal(iOMG, 8 ),
//                            getObstacleVal(iOMG, 9 ), getObstacleVal(iOMG, 10 ), getObstacleVal(iOMG, 11 ),
//                            getObstacleVal(iOMG, 12 ), getObstacleVal(iOMG, 13 ), getObstacleVal(iOMG, 14 ),
//                            getObstacleVal(iOMG, 15 ), getObstacleVal(iOMG, 16 ), getObstacleVal(iOMG, 17 ),
//                            getObstacleVal(iOMG, 18 ), getObstacleVal(iOMG, 19 ), getObstacleVal(iOMG, 20 ),
//                            getObstacleVal(iOMG, 21 ), getObstacleVal(iOMG, 22 ), getObstacleVal(iOMG, 23 ),
//                            getObstacleVal(iOMG, 24 ), getObstacleVal(iOMG, 25 ), getObstacleVal(iOMG, 26 ),
//                            getObstacleVal(iOMG, 27 ), getObstacleVal(iOMG, 28 ), getObstacleVal(iOMG, 29 ),
//                            getObstacleVal(iOMG, 30 ), getObstacleVal(iOMG, 31 ), getObstacleVal(iOMG, 32 ),
//                            getObstacleVal(iOMG, 33 ), getObstacleVal(iOMG, 34 ), getObstacleVal(iOMG, 35 ),
//                            getObstacleVal(iOMG, 36 ), getObstacleVal(iOMG, 37 ), getObstacleVal(iOMG, 38 ),
//                            getObstacleVal(iOMG, 39 ), getObstacleVal(iOMG, 40 ), getObstacleVal(iOMG, 41 ),
//                            getObstacleVal(iOMG, 42 ), getObstacleVal(iOMG, 43 ), getObstacleVal(iOMG, 44 ),
//                            getObstacleVal(iOMG, 45 ), getObstacleVal(iOMG, 46 ), getObstacleVal(iOMG, 47 ),
//                            getObstacleVal(iOMG, 48 ), getObstacleVal(iOMG, 49 ), getObstacleVal(iOMG, 50 ),
//                            getObstacleVal(iOMG, 51 ), getObstacleVal(iOMG, 52 ), getObstacleVal(iOMG, 53 ),
//                            getObstacleVal(iOMG, 54 ), getObstacleVal(iOMG, 55 ), getObstacleVal(iOMG, 56 ),
//                            getObstacleVal(iOMG, 57 ), getObstacleVal(iOMG, 58 ), getObstacleVal(iOMG, 59 ),
//                            getObstacleVal(iOMG, 60 ), getObstacleVal(iOMG, 61 ), getObstacleVal(iOMG, 62 ),
//                            getObstacleVal(iOMG, 63 ), getObstacleVal(iOMG, 64 ), getObstacleVal(iOMG, 65 ),
//                            getObstacleVal(iOMG, 66 ), getObstacleVal(iOMG, 67 ), getObstacleVal(iOMG, 68 ),
//                            getObstacleVal(iOMG, 69 ), getObstacleVal(iOMG, 70 ), getObstacleVal(iOMG, 71 ),
//                            getObstacleVal(iOMG, 72 ), getObstacleVal(iOMG, 73 ), getObstacleVal(iOMG, 74 ),
//                            getObstacleVal(iOMG, 75 ), getObstacleVal(iOMG, 76 ), getObstacleVal(iOMG, 77 ),
//                            getObstacleVal(iOMG, 78 ), getObstacleVal(iOMG, 79 ), getObstacleVal(iOMG, 80 ),
//                            getObstacleVal(iOMG, 81 ), getObstacleVal(iOMG, 82 ), getObstacleVal(iOMG, 83 ),
//                            getObstacleVal(iOMG, 84 ), getObstacleVal(iOMG, 85 ), getObstacleVal(iOMG, 86 ),
//                            getObstacleVal(iOMG, 87 ), getObstacleVal(iOMG, 88 ), getObstacleVal(iOMG, 89 ),
//                            getObstacleVal(iOMG, 90 ), getObstacleVal(iOMG, 91 ), getObstacleVal(iOMG, 92 ),
//                            getObstacleVal(iOMG, 93 ), getObstacleVal(iOMG, 94 ), getObstacleVal(iOMG, 95 ),
//                            getObstacleVal(iOMG, 96));
                    
                    
//                    sprintf(gridOut, "%02x", (unsigned int) (getObstacleVal(0,0) & 0xff));
                    
//                    commSendMsgToSendQueue(gridOut);
//                }
                
                                
            } else if (msgId == MAP_IR_2_ID && 0) {
                //Handle input from the first IR sensor
//                short rec = 0;
//                rec = ((receivemsg[0]) << 8) | ((receivemsg[1]));
//                char ir0Val[RECEIVE_BUFFER_SIZE];
//
//
//                unsigned short ir0Out = 0;
////                float ir0Numer = 7501.53;
////                float ir0Exp = 1000.0 / 1009.0;
////                float ir0Denom = powf(((float) rec), ir0Exp);
//                float ir0Numer = 2697.74;
//                float ir0Exp = 1000.0 / 1167.0;
//                float ir0Denom = powf(((float) rec), ir0Exp);
//                ir0Out = ((unsigned short) (ir0Numer / ir0Denom));
                
                // update occupancy grid using y position           How much to increment by?
                if (mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][ir0Out/2] != 0xffff)
                    incrementRoverVal(mappingData.rover_y_pos/2, ir0Out/2, 5);
                short i;
                for (i = 0; i <ir0Out/2; ++i) {                     //ir0Out is divided by 2 because cells are 2cm wide
                    if (mappingData.OCCUPANCY_GRID[mappingData.rover_y_pos/2][i] != 0)
                        decrementRoverVal(mappingData.rover_y_pos/2, i, 1);
                }

                char gridOut[RECEIVE_BUFFER_SIZE];
                int iOMG;
                for (iOMG = 0; iOMG < 1; ++iOMG) {
                    sprintf(gridOut, "*%03d:%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|%02x|~", iOMG,
                            mappingData.OCCUPANCY_GRID[iOMG][ 0 ], mappingData.OCCUPANCY_GRID[iOMG][ 1 ], mappingData.OCCUPANCY_GRID[iOMG][ 2 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 3 ], mappingData.OCCUPANCY_GRID[iOMG][ 4 ], mappingData.OCCUPANCY_GRID[iOMG][ 5 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 6 ], mappingData.OCCUPANCY_GRID[iOMG][ 7 ], mappingData.OCCUPANCY_GRID[iOMG][ 8 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 9 ], mappingData.OCCUPANCY_GRID[iOMG][ 10 ], mappingData.OCCUPANCY_GRID[iOMG][ 11 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 12 ], mappingData.OCCUPANCY_GRID[iOMG][ 13 ], mappingData.OCCUPANCY_GRID[iOMG][ 14 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 15 ], mappingData.OCCUPANCY_GRID[iOMG][ 16 ], mappingData.OCCUPANCY_GRID[iOMG][ 17 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 18 ], mappingData.OCCUPANCY_GRID[iOMG][ 19 ], mappingData.OCCUPANCY_GRID[iOMG][ 20 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 21 ], mappingData.OCCUPANCY_GRID[iOMG][ 22 ], mappingData.OCCUPANCY_GRID[iOMG][ 23 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 24 ], mappingData.OCCUPANCY_GRID[iOMG][ 25 ], mappingData.OCCUPANCY_GRID[iOMG][ 26 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 27 ], mappingData.OCCUPANCY_GRID[iOMG][ 28 ], mappingData.OCCUPANCY_GRID[iOMG][ 29 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 30 ], mappingData.OCCUPANCY_GRID[iOMG][ 31 ], mappingData.OCCUPANCY_GRID[iOMG][ 32 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 33 ], mappingData.OCCUPANCY_GRID[iOMG][ 34 ], mappingData.OCCUPANCY_GRID[iOMG][ 35 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 36 ], mappingData.OCCUPANCY_GRID[iOMG][ 37 ], mappingData.OCCUPANCY_GRID[iOMG][ 38 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 39 ], mappingData.OCCUPANCY_GRID[iOMG][ 40 ], mappingData.OCCUPANCY_GRID[iOMG][ 41 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 42 ], mappingData.OCCUPANCY_GRID[iOMG][ 43 ], mappingData.OCCUPANCY_GRID[iOMG][ 44 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 45 ], mappingData.OCCUPANCY_GRID[iOMG][ 46 ], mappingData.OCCUPANCY_GRID[iOMG][ 47 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 48 ], mappingData.OCCUPANCY_GRID[iOMG][ 49 ], mappingData.OCCUPANCY_GRID[iOMG][ 50 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 51 ], mappingData.OCCUPANCY_GRID[iOMG][ 52 ], mappingData.OCCUPANCY_GRID[iOMG][ 53 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 54 ], mappingData.OCCUPANCY_GRID[iOMG][ 55 ], mappingData.OCCUPANCY_GRID[iOMG][ 56 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 57 ], mappingData.OCCUPANCY_GRID[iOMG][ 58 ], mappingData.OCCUPANCY_GRID[iOMG][ 59 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 60 ], mappingData.OCCUPANCY_GRID[iOMG][ 61 ], mappingData.OCCUPANCY_GRID[iOMG][ 62 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 63 ], mappingData.OCCUPANCY_GRID[iOMG][ 64 ], mappingData.OCCUPANCY_GRID[iOMG][ 65 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 66 ], mappingData.OCCUPANCY_GRID[iOMG][ 67 ], mappingData.OCCUPANCY_GRID[iOMG][ 68 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 69 ], mappingData.OCCUPANCY_GRID[iOMG][ 70 ], mappingData.OCCUPANCY_GRID[iOMG][ 71 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 72 ], mappingData.OCCUPANCY_GRID[iOMG][ 73 ], mappingData.OCCUPANCY_GRID[iOMG][ 74 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 75 ], mappingData.OCCUPANCY_GRID[iOMG][ 76 ], mappingData.OCCUPANCY_GRID[iOMG][ 77 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 78 ], mappingData.OCCUPANCY_GRID[iOMG][ 79 ], mappingData.OCCUPANCY_GRID[iOMG][ 80 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 81 ], mappingData.OCCUPANCY_GRID[iOMG][ 82 ], mappingData.OCCUPANCY_GRID[iOMG][ 83 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 84 ], mappingData.OCCUPANCY_GRID[iOMG][ 85 ], mappingData.OCCUPANCY_GRID[iOMG][ 86 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 87 ], mappingData.OCCUPANCY_GRID[iOMG][ 88 ], mappingData.OCCUPANCY_GRID[iOMG][ 89 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 90 ], mappingData.OCCUPANCY_GRID[iOMG][ 91 ], mappingData.OCCUPANCY_GRID[iOMG][ 92 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 93 ], mappingData.OCCUPANCY_GRID[iOMG][ 94 ], mappingData.OCCUPANCY_GRID[iOMG][ 95 ],
                            mappingData.OCCUPANCY_GRID[iOMG][ 96]);
                
                    commSendMsgToSendQueue(gridOut);
                }
                                            
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
