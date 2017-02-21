///* ************************************************************************** */
///** Descriptive File Name
//
//  @Company
//    Company Name
//
//  @File Name
//    filename.c
//
//  @Summary
//    Brief description of the file.
//
//  @Description
//    Describe the purpose of this file.
// */
///* ************************************************************************** */
#include "myjson.h"

int jsonFindElementFromKey (unsigned char *buffer, unsigned char key){
	int length = strlen(buffer);//sizeof(buffer) / sizeof(buffer[0]);
//    dbgOutputVal(length);
	int i;
	for (i = 0; i < length; i++){
            
		if (buffer[i] == key){
//            dbgOutputVal(i);
			while (buffer[i] != ELEMENT_DELIM){
				i++;
                if (i >= length){
                    return -1;
                }
			}
//            dbgOutputVal(i);
			return i;
		}
	}
	return -1;
}

int jsonGetSource(unsigned char *buffer, unsigned char *source){
//        dbgOutputLoc(82);
	int index = jsonFindElementFromKey(buffer, KEY_SOURCE);
//        dbgOutputLoc(83);
//        dbgOutputVal(index);
	if (index == -1){
		//The key was not found
//        dbgOutputLoc(84);
		return -1;
	}
	if (parseCharacter(buffer, index, source)){
		//There was a parse error
//        dbgOutputLoc(85);
		return 1;
	}
//        dbgOutputLoc(86);
	return 0;
}

int jsonGetDestination(unsigned char *buffer, unsigned char *destination){
	int index = jsonFindElementFromKey(buffer, KEY_DESTINATION);
	if (index == -1){
		//The key was not found
		return -1;
	}
	if (parseCharacter(buffer, index, destination)){
		//There was a parse error
		return 1;
	}
	return 0;
}

int jsonGetMessageType(unsigned char *buffer, unsigned char *messageType){
	int index = jsonFindElementFromKey(buffer, KEY_MESSAGE_TYPE);
	if (index == -1){
		//The key was not found
		return -1;
	}
	//if (parseString(buffer, index, messageType)){
	if (parseCharacter(buffer, index, messageType)){
		//There was a parse error
		return 1;
	}
	return 0;
}

int jsonGetSequenceNumber(unsigned char *buffer, unsigned char *sequenceNumber){
	int index = jsonFindElementFromKey(buffer, KEY_SEQUENCE_NUMBER);
//    dbgOutputVal(index);
	if (index == -1){
//        dbgOutputLoc(80);
		//The key was not found
		return -1;
	}
        
	if (parseUInt8(buffer, index, sequenceNumber)){
//        dbgOutputLoc(81);
		//There was a parse error
		return 1;
	}
	return 0;
}

int jsonGetChecksum(unsigned char *buffer, unsigned int *checksum){
	int index = jsonFindElementFromKey(buffer, KEY_CHECKSUM);
	if (index == -1){
		//The key was not found
		return -1;
	}
	if (parseInt(buffer, index, checksum)){
		//There was a parse error
		return 1;
	}
    dbgOutputVal(*checksum);
	return 0;
}

int jsonGetChecksumString(unsigned char *buffer, unsigned char *checksum){
	int index = jsonFindElementFromKey(buffer, KEY_CHECKSUM);
	if (index == -1){
		//The key was not found
		return -1;
	}
	if (parseCheckSum(buffer, index, checksum)){
		//There was a parse error
		return 1;
	}
    dbgOutputVal(*checksum);
	return 0;
}

int jsonGetFieldItem(unsigned char *buffer, fieldItem *item){
    int index = jsonFindElementFromKey(buffer, KEY_DATA);
	if (index == -1){
		//The key was not found
		return -1;
	}
	if (parseFieldItem(buffer, index, item)){
		//There was a parse error
		return 1;
	}
	return 0;
}


//Return 0 upon success, 1 upon failure
int parseCharacter(char* bufferToReadFrom, int index, unsigned char *info) {
    if (bufferToReadFrom[index] != ':' && bufferToReadFrom[index+1] != '"') {
        return 1;
    }
    index += 2;
    int length = 1;
    
    if (bufferToReadFrom[index+1] != '"') {
        return 1;
    }
    
    info = &bufferToReadFrom[index];
    dbgOutputVal(*info);
    return 0;
}

int parseString(char* bufferToReadFrom,int index, unsigned char* info) {
    if (bufferToReadFrom[index] != ':' || bufferToReadFrom[index+1] != '"') {
        return 1;
    }
    index += 2;
    int length = 0;
    
    while(1) {
        if (bufferToReadFrom[index+length] == '"') {
            break;
        } else {
            length++;
        }
        
        if (length > 3) {
            return 1;
        }
    }
    
    int i=0;
    for (i=0; i < length; i++) {
//        strcat(info, bufferToReadFrom[index+i]);
        info[i] = bufferToReadFrom[index+i];
        dbgOutputVal(info[i]);
    }
    return 0;
}

int parseInt(char* bufferToReadFrom, int index, unsigned int* info) {
    if (bufferToReadFrom[index] != ':') {
        return 1;
    }
    
//    int inc = 1;
    int length = 0;
    
    while(1) {
        if (bufferToReadFrom[index+length+1] == ',' || bufferToReadFrom[index+length+1] == '}') {
            break;
        } else {
            length++;
        }
        
        if (length > 5) {
            return 1;
        }
    }
    unsigned char numInStringForm[length+1];
    
    int i=0;
    for (i=0; i < length; i++) {
//        strcat(info, bufferToReadFrom[index+i]);
        numInStringForm[i] = bufferToReadFrom[index+i+1];
//        dbgOutputVal(numInStringForm[i]);
    }
    numInStringForm[i] = '\0';
    
    *info = atoi(numInStringForm);
//    dbgOutputVal(220);
    dbgOutputVal(info[0] & 0x000000ff);
    dbgOutputVal((info[0] & 0x0000ff00) >> 8);
    dbgOutputVal((info[0] & 0x00ff0000) >> 16);
    dbgOutputVal((info[0] & 0xff000000) >> 24);
    return 0;
}

int parseCheckSum(char* bufferToReadFrom, int index, unsigned char numInStringForm[6]) {
    if (bufferToReadFrom[index] != ':') {
        return 1;
    }
    
//    int inc = 1;
    int length = 0;
    
    while(1) {
        if (bufferToReadFrom[index+length+1] == ',' || bufferToReadFrom[index+length+1] == '}') {
            break;
        } else {
            length++;
        }
        
        if (length > 5) {
            return 1;
        }
    }
//    unsigned char numInStringForm[length+1];
    
    int i=0;
    for (i=0; i < length; i++) {
//        strcat(info, bufferToReadFrom[index+i]);
        numInStringForm[i] = bufferToReadFrom[index+i+1];
//        dbgOutputVal(numInStringForm[i]);
    }
    numInStringForm[i] = '\0';
    
//    *info = atoi(numInStringForm);
//    dbgOutputVal(220);
//    dbgOutputVal(info[0] & 0x000000ff);
//    dbgOutputVal((info[0] & 0x0000ff00) >> 8);
//    dbgOutputVal((info[0] & 0x00ff0000) >> 16);
//    dbgOutputVal((info[0] & 0xff000000) >> 24);
    return 0;
}

int parseUInt8(unsigned char* bufferToReadFrom, int index, unsigned char* info) {
    
    if (bufferToReadFrom[index] != ':') {
        return 1;
    }
    
//    int inc = 1;
    int length = 0;
    
    while(1) {
        if (bufferToReadFrom[index+length+1] == ',') {
            break;
        } else {
            length++;
        }
        
        if (length > 5) {
            return 1;
        }
    }
    unsigned char numInStringForm[length+1];
    
    int i=0;
    for (i=0; i < length; i++) {
//        strcat(info, bufferToReadFrom[index+i]);
        numInStringForm[i] = bufferToReadFrom[index+i+1];
//        dbgOutputVal(numInStringForm[i]);
    }
    numInStringForm[i] = '\0';
    
    *info = atoi(numInStringForm);
//    dbgOutputVal(220);
//    dbgOutputVal(*info);
    return 0;
}

int readNumInStringForm(char* bufferToReadFrom, int* index) {
    unsigned char* numInStringForm;
    int length = 1;
    while(1) {
        if (bufferToReadFrom[*index+length] == ',') {
            break;
        } else {
            length++;
        }
        
        if (length > 5) {
            return -1;
        }
    }
    
    int i=0;
    for (i=0; i < length; i++) {
//        strcat(info, bufferToReadFrom[index+i]);
        numInStringForm[i] = bufferToReadFrom[*index+i];
    }
    
    *index += length;
    
     return atoi(numInStringForm);
}

int parseFieldItem(char* bufferToReadFrom,int index, fieldItem* info) {
    
    if (bufferToReadFrom[index] != ':' && bufferToReadFrom[index+1] != '[') {
        return 1;
    }
    index += 2;
    fieldItem tempFieldItem;
    
    tempFieldItem.objectType = readNumInStringForm(bufferToReadFrom, &index);
    if (bufferToReadFrom[index] != ',') {
        return 1;
    } else {
        index++;
    }
    tempFieldItem.versionNumber = readNumInStringForm(bufferToReadFrom, &index);
    if (bufferToReadFrom[index] != ',') {
        return 1;
    } else {
        index++;
    }
    tempFieldItem.length = readNumInStringForm(bufferToReadFrom, &index);
    if (bufferToReadFrom[index] != ',') {
        return 1;
    } else {
        index++;
    }
    tempFieldItem.width = readNumInStringForm(bufferToReadFrom, &index);
    if (bufferToReadFrom[index] != ',') {
        return 1;
    } else {
        index++;
    }
    tempFieldItem.centerX = readNumInStringForm(bufferToReadFrom, &index);
    if (bufferToReadFrom[index] != ',') {
        return 1;
    } else {
        index++;
    }
    tempFieldItem.centerY = readNumInStringForm(bufferToReadFrom, &index);
    if (bufferToReadFrom[index] != ',') {
        return 1;
    } else {
        index++;
    }
    tempFieldItem.orientation = readNumInStringForm(bufferToReadFrom, &index);
    
    if ((tempFieldItem.objectType < 0) || (tempFieldItem.versionNumber < 0) || (tempFieldItem.length < 0) || (tempFieldItem.width < 0) || (tempFieldItem.centerX < 0) || (tempFieldItem.centerY < 0) || (tempFieldItem.orientation < 0)) {
        return 1;
    }
}







/* *****************************************************************************
 End of File
 */
