#include "stdint.h"

typedef enum
{
    LSB_FIRST=0,
    MSB_FIRST=1,
    RESERVED
}direction_t;
typedef struct 
{
    uint16_t uHeaderHighTime;   //duration of header "1" pulse in u_sec
    uint16_t uHeaderLowTime;    //duration of header "0" pulse in u_sec
    uint16_t uDelayTime;        //duration of delay pulse in u_sec. high level pulse
    uint16_t uZeroByteTime;     //duration of data "0" pulse in u_sec. Low level pulse
    uint16_t uOneByteTime;      //duration of data "1" pulse in u_sec. Low level pulse
    uint32_t cur_command;       //Current command. Max size - 32 bits
    uint8_t command_len;        //Number of bits in command
    direction_t bit_direction;  
    uint8_t next_bit;             //Possion of next bit to send

}IRimitator_t;

