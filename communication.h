#include <string>
#define NONE 250

//============================
//==    Pin Assignments     ==
//============================
//Knobs
#define POT1 p17  //Knob1
#define POT2 p18  //Knob2
#define POT3 p16  //Knob3
#define POT4 p15  //Knob4
//JoyStick
#define POTV p19 //Vertial
#define POTH p20 //Horizontal
//MRF24J
#define SDI p11
#define SDO p12
#define SCK p13
#define CS p7
#define RESET p8
//Button
#define BUTTON1 p21

//MRF24J40
PinName mosi(SDI);
PinName miso(SDO);
PinName sck(SCK);
PinName cs(CS);
PinName reset(RESET);
// RF tranceiver
MRF24J40 mrf(mosi, miso, sck, cs, reset);

// Send / receive buffers.
// IMPORTANT: The MRF24J40 is intended as zigbee tranceiver; it tends
// to reject data that doesn't have the right header. So the first
// 8 bytes in txBuffer look like a valid header. The remaining 120
// bytes can be used for anything you like.
char txBuffer[128];
char rxBuffer[128];
int rxLen;


/**
* Receive data from the MRF24J40.
*
* @param data A pointer to a char array to hold the data
* @param maxLength The max amount of data to read.
*/
int rf_receive(char *data, uint8_t maxLength)
{
    uint8_t len = mrf.Receive((uint8_t *)data, maxLength);
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};

    if(len > 10) {
        //Remove the header and footer of the message
        for(uint8_t i = 0; i < len-2; i++) {
            if(i<8) {
                //Make sure our header is valid first
                if(data[i] != header[i]) {
                    //mrf.Reset();
                    return 0;
                }
            } else {
                data[i-8] = data[i];
            }
        }
        //pc.printf("Received: %s length:%d\r\n", data, ((int)len)-10);
    }
    return ((int)len)-10;
}

/**
* Send data to another MRF24J40.
*
* @param data The string to send
* @param maxLength The length of the data to send.
*                  If you are sending a null-terminated string you can pass strlen(data)+1
*/
void rf_send(char *data, uint8_t len)
{
    //We need to prepend the message with a valid ZigBee header
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};
    uint8_t *send_buf = (uint8_t *) malloc( sizeof(uint8_t) * (len+8) );

    for(uint8_t i = 0; i < len+8; i++) {
        //prepend the 8-byte header
        send_buf[i] = (i<8) ? header[i] : data[i-8];
    }
    //pc.printf("Sent: %s\r\n", send_buf+8);

    mrf.Send(send_buf, len+8);
    free(send_buf);
}

//Returns true if c is a letter, false otherwise
bool isLetter(char c) {
    if(('a'<=c & c<='z') | ('A'<=c & c<='Z'))
        return true;
    return false;
}

//Returns true if c is a number, false otherwise
bool isNumber(char c) {
    if('0'<=c & c<='9')
        return true;
    return false;
}

//Pulls data out fo rxBuffer and updates global variables accordingly 
void communication_protocal(int len)
{
    bool found_name = false;
    bool found_num = false;
    bool complete_name = false;
    bool complete_num = false;
    uint8_t name_start = NONE; uint8_t name_end = NONE;
    uint8_t num_start = NONE; uint8_t num_end = NONE;
    
    //Loop through all charaters in rxBuffer
    for(uint8_t i = 0; i <= rxLen; i++) {
        char c = rxBuffer[i];
        //pc.printf("Indexed char '%c'\r\n", c);

        //Find the start of a name (Check if its a letter)
        if(isLetter(c) & name_start==NONE) { //if a num
            //If We havent found a name yet, this is start of a name
            if(found_name == false) {
                //pc.printf("found name start at: '%c'\r\n", c);
                name_start = i;
                found_name = true;
            }
        }
        //Find 'end of name' charater: ' ', ':', '-'
        else if(((c == ' ') | (c == ':') | (c == '-')) & found_name & !complete_name) {// found end name character
            if(found_name) {
                complete_name = true;
                name_end = i;
                //pc.printf("found end of name at: '%c'\r\n", txBuffer[name_end]);
            }
        }

        //Find 'start of a number' charater, Check if its a number, or '-', or a '.'
        else if( (isNumber(c) | (c=='-') | (c=='.')) & complete_name & num_start==NONE) {
            if(found_num == false) {
                //pc.printf("found num start at: '%c'\r\n",c);
                num_start = i;
                found_num = true;
            }
        }
        //Found end of number character: ' ', ':', '-', or a letter
        else if( (((c==' ')|(c==':')|(c=='-')) | isLetter(c)) & found_num & complete_name) {
            if(found_num) {
                complete_num = true;
                num_end = i;
                //pc.printf("found end of num at: '%c' \r\n", txBuffer[num_end]);
            }
        }
        
        //If we have a complete name AND number value (ie. start and end of each != NONE)
        if(found_name & found_num & complete_name & complete_num) {
            //pc.printf("Found MATCH\r\n");
            //Reset flags
            found_name = false;
            found_num = false;
            complete_name = false;
            complete_num = false;
            
            //Set name
            uint8_t nameLen = uint8_t((name_end-name_start) + 1);
            char * name[nameLen];
            *name = &rxBuffer[name_start];
            rxBuffer[name_end] = '\0';
            
            //Set num
            uint8_t numLen = uint8_t((num_end-num_start) + 1);
            char * num[numLen];
            *num = &rxBuffer[num_start];
            rxBuffer[num_end] = '\0';
            
            //Set variables
            if(strcmp(*name, "jstick_h\0")==0)
                jstick_h = atof(*num);
            else if(strcmp(*name, "jstick_v\0")==0) 
                jstick_v = atof(*num);
            else if(strcmp(*name, "knob1\0")==0) 
                knob1 = atof(*num);
            else if(strcmp(*name, "knob2\0")==0) 
                knob2 = atof(*num);
            else if(strcmp(*name, "knob3\0")==0)
                knob3 = atof(*num);
            else if(strcmp(*name, "knob4\0")==0) 
                knob4 = atof(*num);
            else if(strcmp(*name, "button\0")==0) 
                button = (bool)atoi(*num);
            
            //Reset flags
            name_start = NONE;
            name_end = NONE;
            num_start = NONE;
            num_end = NONE;
        }
    }
}