#pragma once
#include "stdint.h"
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

typedef enum
{
    LSB_FIRST=0,
    MSB_FIRST=1,
    DIRECTION_MAX
}direction_t;

typedef enum
{
    PULSE_ENC=0,
    DELAY_ENC=1,
    IR_ENCODING_MAX
}ir_encoding_t;

/**
 * @brief IRimitator_t object should be initialized before sending commands by calling IRimitatorInit()
 * 
 */
typedef struct 
{
    uint16_t uHeaderHighTime;   //< duration of header "1" pulse in u_sec
    uint16_t uHeaderLowTime;    //duration of header "0" pulse in u_sec
    uint16_t uDelayTime;        //duration of delay pulse in u_sec. high level pulse
    uint16_t uZeroBitTime;      //duration of data "0" pulse in u_sec. Low level pulse
    uint16_t uOneBitTime;       //duration of data "1" pulse in u_sec. Low level pulse
    uint32_t cur_command;       //Current command. Max size - 32 bits
    uint8_t command_len;        //Number of bits in command
    direction_t bit_direction;  //
    ir_encoding_t encoding;     //Type of IR signal encoding


}IRimitator_t;

typedef struct
{
    uint8_t size;
    uint16_t durations[256];
    uint8_t *pulse_sequence;
    uint8_t next_pulse;           //Position of next pulse in durations array to send
}IRcommand_t;

/**
 * @brief Initialize IRimitator with custom parameters.
 * 
 * 
 *
 * @param IRimit [out] Pointer to IRimitator_t object that should be initialized
 * @param uHHtime [in] duration of header "1" pulse in u_sec. Low level pulse.
 * @param uHLtime [in] duration of header "0" pulse in u_sec
 * @param uDtime [in] duration of delay pulse in u_sec. Low level pulse
 * @param uZBtime [in] duration of data "0" pulse in u_sec. High level pulse
 * @param uOBtime [in] duration of data "1" pulse in u_sec. High level pulse
 * @param bit_dir [in] bit direction
 * @param enc [in] signal encoding
 * @retval 0 - initialization error
 * @retval 1 - success 
 */
uint8_t IRimitatorInit(IRimitator_t *IRimit,uint16_t uHHtime,uint16_t uHLtime, uint16_t uDtime, uint16_t uZBtime,uint16_t uOBtime, direction_t bit_dir, ir_encoding_t enc);

/**
 * @brief Initialize IRimitator with dyson ir protocol default parameters 
 * 
 *
 * @param IRimit [out] Pointer to IRimitator_t object that should be initialized
 * @retval 0 - initialization error
 * @retval 1 - success 
 */
uint8_t IRimitatorInitDysonDefault(IRimitator_t *IRimit);

/**
 * @brief Executing command send
 * 
 *
 * @param IRimit [in] Pointer to IRimitator_t object. Must be initialized before.
 * @param ir_command [in] Command to send. 
 * @param comLen [in] Command length. Number of significant bits in ir_command
 * @retval 0 - Invalid IRimit pointer or comLen=0
 * @retval 1 - Sending sequence started 
 */
uint8_t IRimitatorSend(IRimitator_t *IRimit,uint32_t ir_command, uint8_t comLen);

/**
 * @brief Generates timing sequence
 * 
 *
 * @param source [in] Pointer to IRimitator_t object. Must be initialized before.
 * @return Pointer to IRcommand type object. Warning! To prevent memory leakage object should be 
 * destructed using free() as well as \c duration field
 */
IRcommand_t* GenerateIRsequence(IRimitator_t *source);

/**
 * @brief The function is called when command is ready to be sent. Function should be implemented by user 
 * to provide sending procedures using hardware routines
 * 
 *
 * @param command [in] Pointer to IRcommand_t object with all information to arrange sending
 */
void StartCommandSequence(IRcommand_t* command);
