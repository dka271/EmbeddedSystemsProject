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

#include "electromagnet.h"


//Sets the electromagnet on
void ElectromagnetSetOn(){
    TRISDCLR = ELECTROMAGNET_PINS;
    ODCDCLR  = ELECTROMAGNET_PINS;
    LATDSET  = ELECTROMAGNET_PINS;
}

//Sets the electromagnet off
void ElectromagnetSetOff(){
    TRISDCLR = ELECTROMAGNET_PINS;
    ODCDCLR  = ELECTROMAGNET_PINS;
    LATDCLR  = ELECTROMAGNET_PINS;
}


//------------------------------------------------------------------------------
//Testing Code
//------------------------------------------------------------------------------

//Handle receiving
void electromagnetTestCommReceive(unsigned char receivemsg[COMM_QUEUE_BUFFER_SIZE]){
    unsigned char Source;
    if (jsonGetSource(receivemsg, &Source)) {
        //error
    }else if (Source == TEST_ELECTROMAGNET_SERVER_ID){
        //Handle stuff from my speed test server
        int command;
        if(jsonGetChecksum(receivemsg, &command) == 0){
            //Turn on or off the electromagnet
            if (command == 0){
                //Turn off the electromagnet
                ElectromagnetSetOff();
                //Notify the server of the status
                electromagnetTestSend(command);
            }else if (command == 1){
                //Turn on the electromagnet
                ElectromagnetSetOn();
                //Notify the server of the status
                electromagnetTestSend(command);
            }
        }
    }
}

//Handle sending
void electromagnetTestSend(int status){
    //Send the status to the electromagnet test server
    char testServerMsg[SEND_QUEUE_BUFFER_SIZE];
    sprintf(testServerMsg, "*{\"S\":\"%c\",\"T\":\"%c\",\"M\":\"%c\",\"N\":%d,\"D\":[%d],\"C\":%d}~", 's', TEST_ELECTROMAGNET_SERVER_ID, 's', 0, status, status);
    commSendMsgToWifiQueue(testServerMsg);
}

/* *****************************************************************************
 End of File
 */
