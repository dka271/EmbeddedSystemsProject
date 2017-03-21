/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "motor.h"

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */
    
void Motor1SetDirection(int direction){
    /*TRISCCLR = 0x4000;
    ODCCCLR  = 0x4000;

    LATCCLR = 0x4000;
    LATCSET = (direction & 0x0001) << 14;*/
    TRISCCLR = 0x0008;
    ODCCCLR  = 0x0008;

    if (direction){
        LATCSET = 0x0008;
    }else{
        LATCCLR = 0x0008;
    }

//        LATCCLR = 0x0008;
//        LATCSET = (direction & 0x0001) << 3;
}

void Motor2SetDirection(int direction){
    TRISGCLR = 0x0002;
    ODCGCLR  = 0x0002;

    if (direction){
        LATGSET = 0x0002;
    }else{
        LATGCLR = 0x0002;
    }

//        LATGCLR = 0x0002;
//        LATGSET = (direction & 0x0001) << 1;
}

void Motor1SetPWM(int pwm){
    TRISDCLR = 0x0001;
    ODCDCLR  = 0x0001;

    LATDCLR = 0x0001;
    LATDSET = (pwm & 0x0001);

}

void Motor2SetPWM(int pwm){
    TRISDCLR = 0x0002;
    ODCDCLR  = 0x0002;

    LATDCLR = 0x0002;
    LATDSET = (pwm & 0x0001) << 1;
    /*TRISCCLR = 0x0004;
    ODCCCLR  = 0x0004;

    LATCCLR = 0x0004;
    LATCSET = (pwm & 0x0001) << 2;*/
}
    
    

int GetPWMFromValue(unsigned int val, unsigned int count){
    if (val > 25){
        val = 25;
    }
    int pwm = (count % 25) < val;
    return pwm;
}


int currentDirection = ROVER_DIRECTION_FORWARDS;
void HandleDistanceRemaining(int *desiredSpeed, int *ticksRemaining, int changeInSpeed){
    if (currentDirection = ROVER_DIRECTION_FORWARDS || currentDirection == ROVER_DIRECTION_BACKWARDS){
        //Going straight
        if (*desiredSpeed > ROVER_SPEED_STOPPED){
            *ticksRemaining -= changeInSpeed;
            if (*ticksRemaining <= ROVER_TICKS_REMAINING_NONE){
                *desiredSpeed = ROVER_SPEED_STOPPED;
            }
        }
    }else{
        //Turning
        if (*desiredSpeed > ROVER_SPEED_STOPPED){
            *ticksRemaining -= changeInSpeed;
            if (*ticksRemaining <= ROVER_TICKS_REMAINING_NONE){
                *desiredSpeed = ROVER_SPEED_STOPPED;
            }
//            else if (*ticksRemaining <= ROVER_TICKS_REMAINING_SLOW){
//                *desiredSpeed = ROVER_SPEED_SLOW;
//            }
        }
    }
}



void SetDirectionClockwise(){
    Motor1SetDirection(MOTOR_1_BACKWARDS);
    Motor2SetDirection(MOTOR_2_FORWARDS);
    currentDirection = ROVER_DIRECTION_RIGHT;
}
void SetDirectionCounterclockwise(){
    Motor1SetDirection(MOTOR_1_FORWARDS);
    Motor2SetDirection(MOTOR_2_BACKWARDS);
    currentDirection = ROVER_DIRECTION_LEFT;
}
void SetDirectionForwards(){
    Motor1SetDirection(MOTOR_1_FORWARDS);
    Motor2SetDirection(MOTOR_2_FORWARDS);
    currentDirection = ROVER_DIRECTION_FORWARDS;
}
void SetDirectionBackwards(){
    Motor1SetDirection(MOTOR_1_BACKWARDS);
    Motor2SetDirection(MOTOR_2_BACKWARDS);
    currentDirection = ROVER_DIRECTION_BACKWARDS;
}
int GetMotorDirection(){
    return currentDirection;
}

int ConvertJSONToAngle(unsigned char jsonAngle){
    int returnAngle;
    //Cut off redundant angles
    if (jsonAngle > 180){
        returnAngle = 180;
    }else{
        returnAngle = (int) jsonAngle;
    }
    
    //Convert the angle to degrees
    returnAngle *= 2;
    
    return returnAngle;
}

float TicksPerDegree = 6.9;
int deg2tick(int deg){
    return (int) (deg * TicksPerDegree);
}
int tick2deg(int ticks){
    return (int) (ticks / TicksPerDegree);
}
//float TicksPerInch = 379.425;
float TicksPerInch = 189.713;
int in2tick(int inches){
    return (int) (inches * TicksPerInch);
}
int tick2in(int ticks){
    return (int) (ticks / TicksPerInch);
}


int forwardOffset = 118;//This constant was determined through observation and testing
int CalculateTicksToTravel(unsigned char distance){
    //Get number of ticks
    int ticks = in2tick((int) distance);
    //Account for overshooting
    ticks -= forwardOffset;
    
    return ticks;
}
int angleOffsetLeft = 52;//This constant was determined through observation and testing
int angleOffsetRight = 32;//This constant was determined through observation and testing
int CalculateTicksToRotate(unsigned char angle){
    //Get number of ticks
//    int angle2 = (int) (angle / 10);
//    angle2 += angle;
//    angle2 -= 9;
    int ticks = deg2tick(ConvertJSONToAngle((int) angle));
    //Account for overshooting
    if (GetMotorDirection() == ROVER_DIRECTION_LEFT){
        ticks -= angleOffsetLeft;
    }else if (GetMotorDirection() == ROVER_DIRECTION_RIGHT){
        ticks -= angleOffsetRight;
    }
    
    //Account for the offset not being constant
//    int linearOffset = ((int) 10 - ((int) (angle / 9))) * TicksPerDegree;
//    ticks -= linearOffset;
    
    return ticks;
}

int PID1_Out = 0;
int PID1(unsigned int setpoint, unsigned int actual){
    int error = (int) ((setpoint - actual));
    
    if (setpoint <= 0){
        PID1_Out = PID1_Out >> 1;
    }else if (error >= 2){
        PID1_Out += (int) (error / 2);
    }else if (error <= -2){
        PID1_Out += (int) (error / 2);
    }else if (error > 0){
        PID1_Out++;
    }else if (error < 0){
        PID1_Out--;
    }
    if (PID1_Out > 25){
        PID1_Out = 25;
    }else if (PID1_Out < 0){
        PID1_Out = 0;
    }
    return PID1_Out;
}

int PID2_Out = 0;
int PID2(unsigned int setpoint, unsigned int actual){
    int error = (int) ((setpoint - actual));
    
    if (setpoint <= 0){
        PID2_Out = PID2_Out >> 1;
    }else if (error >= 2){
        PID2_Out += (int) (error / 2);
    }else if (error <= -2){
        PID2_Out += (int) (error / 2);
    }else if (error > 0){
        PID2_Out++;
    }else if (error < 0){
        PID2_Out--;
    }
    if (PID2_Out > 25){
        PID2_Out = 25;
    }else if (PID2_Out < 0){
        PID2_Out = 0;
    }
    return PID2_Out;
}


//------------------------------------------------------------------------------
//Testing Code
//------------------------------------------------------------------------------

//Variables to be used for testing
int Is_Testing = 0;
void motorTestCommReceive(unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE]){
    unsigned char Source;
    if (jsonGetSource(receivemsg, &Source)) {
        //error
    }else if (Source == TEST_SPEED_SERVER_ID){
        //Handle stuff from my speed test server
        int speed;
        //I store the speed in the checksum field because, at the time of writing, there is no working function to read from the data field
        if(jsonGetChecksum(receivemsg, &speed) == 0){
            //Send speed to Nav queue
            char navMsg[NAV_QUEUE_BUFFER_SIZE];
            navMsg[TEST_NAV_DATA_IDX] = speed & 0xff;
            navMsg[TEST_NAV_ID_IDX] = MOTOR_SPEED_ID;
            navMsg[NAV_SOURCE_ID_IDX] = NAV_OTHER_ID << NAV_SOURCE_ID_OFFSET;
            navMsg[NAV_CHECKSUM_IDX] = navCalculateChecksum(navMsg);
            navSendMsg(navMsg);
        }
    }else if (Source == TEST_ANGLE_DISTANCE_SERVER_ID){
        //Handle stuff from my distance and angle test server
        unsigned char Type;
        if (jsonGetSequenceNumber(receivemsg, &Type) == 0){
            //Extract the data
            int data;
            if (jsonGetChecksum(receivemsg, &data) == 0){
                //Check the message type, then send commands to Navigation based on this type
                if (Type == MOTOR_DIRECTION_ID){
                    //The data is a direction, send it to Navigation
                    char navMsg[NAV_QUEUE_BUFFER_SIZE];
                    navMsg[TEST_NAV_DATA_IDX] = data & 0xff;
                    navMsg[TEST_NAV_ID_IDX] = MOTOR_DIRECTION_ID;
                    navMsg[NAV_SOURCE_ID_IDX] = NAV_OTHER_ID << NAV_SOURCE_ID_OFFSET;
                    navMsg[NAV_CHECKSUM_IDX] = navCalculateChecksum(navMsg);
                    navSendMsg(navMsg);
                }else if (Type == MOTOR_DISTANCE_ID){
                    //The data is a distance, send it to Navigation
                    char navMsg[NAV_QUEUE_BUFFER_SIZE];
                    navMsg[TEST_NAV_DATA_IDX] = data & 0xff;
                    navMsg[TEST_NAV_ID_IDX] = MOTOR_DISTANCE_ID;
                    navMsg[NAV_SOURCE_ID_IDX] = NAV_OTHER_ID << NAV_SOURCE_ID_OFFSET;
                    navMsg[NAV_CHECKSUM_IDX] = navCalculateChecksum(navMsg);
                    navSendMsg(navMsg);
                }else if (Type == MOTOR_ANGLE_ID){
                    //The data is an angle, send it to Navigation
                    char navMsg[NAV_QUEUE_BUFFER_SIZE];
                    navMsg[TEST_NAV_DATA_IDX] = data & 0xff;
                    navMsg[TEST_NAV_ID_IDX] = MOTOR_ANGLE_ID;
                    navMsg[NAV_SOURCE_ID_IDX] = NAV_OTHER_ID << NAV_SOURCE_ID_OFFSET;
                    navMsg[NAV_CHECKSUM_IDX] = navCalculateChecksum(navMsg);
                    navSendMsg(navMsg);
                }
            }
        }
    }
}

void motorTestCommSend(unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE]){
    if (receivemsg[TEST_COMM_ID_IDX] == MOTOR_SPEED_ID){
        //Handle motor speed server testing
        char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
        int speed = (int) receivemsg[TEST_COMM_SPEED_IDX];
        speed |= (int) (receivemsg[TEST_COMM_SPEED_IDX + 1] << 8);
        sprintf(testServerMsg, "*{\"S\":\"%c\",\"T\":\"%c\",\"M\":\"%c\",\"N\":%d,\"D\":[%d],\"C\":%d}~", 's', TEST_SPEED_SERVER_ID, 's', 0, speed, speed);
        commSendMsgToWifiQueue(testServerMsg);
    }
}

void motorTestNavReceive(unsigned char receivemsg[NAV_QUEUE_BUFFER_SIZE], int *currentSpeed, int *ticksRemaining){
    unsigned char ID = receivemsg[TEST_NAV_ID_IDX];
    if (ID == MOTOR_SPEED_ID){
        //Got speed from the test server
        int desiredSpeed;
        desiredSpeed = (int) receivemsg[TEST_NAV_DATA_IDX];
//        dbgOutputVal(desiredSpeed);
        if (desiredSpeed == 0){
            Is_Testing = 0;
            *ticksRemaining = ROVER_TICKS_REMAINING_NONE;
        }else{
            Is_Testing = 1;
            *ticksRemaining = ROVER_TICKS_REMAINING_MAX;
        }
        *currentSpeed = desiredSpeed;
//        return desiredSpeed;
    }else if (ID == MOTOR_DIRECTION_ID){
        //Got a direction from the test server
        //Check the direction, and configure the rover accordingly
        unsigned char direction = receivemsg[TEST_NAV_DATA_IDX];
        if (direction == ROVER_DIRECTION_LEFT){
            SetDirectionCounterclockwise();
        }else if (direction == ROVER_DIRECTION_RIGHT){
            SetDirectionClockwise();
        }else if (direction == ROVER_DIRECTION_FORWARDS){
            SetDirectionForwards();
        }else if (direction == ROVER_DIRECTION_BACKWARDS){
            SetDirectionBackwards();
        }
    }else if (ID == MOTOR_DISTANCE_ID){
        //Got a distance from the test server
        Is_Testing = 1;
        *currentSpeed = ROVER_SPEED_STRAIGHT;
        *ticksRemaining = CalculateTicksToTravel(receivemsg[TEST_NAV_DATA_IDX]);
    }else if (ID == MOTOR_ANGLE_ID){
        //Got an angle from the test server
        Is_Testing = 1;
        *currentSpeed = ROVER_SPEED_TURNING;
//        *currentSpeed = ROVER_SPEED_STRAIGHT;
        *ticksRemaining = CalculateTicksToRotate(receivemsg[TEST_NAV_DATA_IDX]);
    }
}

void motorTestNavSendSpeed(int speed){
    if (Is_Testing){
        //Handle server testing of the speed
        char commMsg[COMM_QUEUE_BUFFER_SIZE];
        commMsg[TEST_COMM_SPEED_IDX] = speed & 0x00ff;
        commMsg[TEST_COMM_SPEED_IDX + 1] = (speed & 0xff00) >> 8;
        commMsg[TEST_COMM_ID_IDX] = MOTOR_SPEED_ID;
        commMsg[COMM_SOURCE_ID_IDX] = COMM_OTHER_ID << COMM_SOURCE_ID_OFFSET;
        commMsg[COMM_CHECKSUM_IDX] = commCalculateChecksum(commMsg);
        commSendMsg(commMsg);
    }
}
    



/* *****************************************************************************
 End of File
 */
