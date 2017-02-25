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
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

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
        
        LATCCLR = 0x0008;
        LATCSET = (direction & 0x0001) << 3;
    }
    
    void Motor2SetDirection(int direction){
        TRISGCLR = 0x0002;
        ODCGCLR  = 0x0002;
        
        LATGCLR = 0x0002;
        LATGSET = (direction & 0x0001) << 1;
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


int Kp = 0;//Test and calibrate this number//1
int Ti = 6;//Try to eliminate past errors in Ti*dt
int Td = 8;//Try to predict errors Td*dt in the future
//Set gain parameters
//int Ki = (int) (Kp / Ti);
//int Kd = Kp * Td;
int Ki = 0;//3
int Kd = 0;//80
int dt = 4;//4 ms
int PIDOffsetDiv = 13;//9
int PIDDiv = 1;//1000
int errorDiv = 1;
int PID1_PreviousError = 0;
int PID1_I = 0;
int PID1_Out = 0;
int PID1(unsigned int setpoint, unsigned int actual){
    int PIDOffset = (int) (setpoint/PIDOffsetDiv);
    int error = (int) ((setpoint - actual));
    /*
    PID1_I += error*dt;
    
    //Limit the Integral component
    if (PID1_I > 1000000){
        PID1_I = 1000000;
    }else if (PID1_I < -1000000){
        PID1_I = -1000000;
    }
    
    int PID1_D = (int) ((error - PID1_PreviousError)/dt);
    int out = (Kp*error*dt) + (Kd*PID1_D) + (Ki*PID1_I);
    PID1_PreviousError = error;
    //Limit the output to [-10,10]
    out = (int) out / PIDDiv;
    //out += PIDOffset;
    */
    if (setpoint <= 0){
        PID1_Out = PID1_Out >> 1;
    }else if (error > 0){
        PID1_Out++;
    }else{
        PID1_Out--;
    }
    if (PID1_Out > 25){
        PID1_Out = 25;
    }else if (PID1_Out < 0){
        PID1_Out = 0;
    }
    return PID1_Out;
}

int PID2_PreviousError = 0;
int PID2_I = 0;
int PID2_Out = 0;
int PID2(unsigned int setpoint, unsigned int actual){
    int PIDOffset = (int) (setpoint/PIDOffsetDiv);
    int error = (int) ((setpoint - actual));
    /*
    PID2_I += error*dt;
    
    //Limit the Integral component
    if (PID2_I > 1000000){
        PID2_I = 1000000;
    }else if (PID2_I < -1000000){
        PID2_I = -1000000;
    }
    
    int PID2_D = (int) ((error - PID2_PreviousError)/dt);
    int out = (Kp*error*dt) + (Kd*PID2_D) + (Ki*PID2_I);
    PID2_PreviousError = error;
    //Limit the output to [0,20]
    out = (int) out / PIDDiv;
    */
    //out += PIDOffset;
    if (setpoint <= 0){
        PID2_Out = PID2_Out >> 1;
    }else if (error > 0){
        PID2_Out++;
    }else{
        PID2_Out--;
    }
    if (PID2_Out > 25){
        PID2_Out = 25;
    }else if (PID2_Out < 0){
        PID2_Out = 0;
    }
    return PID2_Out;
}
    



/* *****************************************************************************
 End of File
 */
