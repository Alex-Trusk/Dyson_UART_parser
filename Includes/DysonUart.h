#ifndef DYSON_UART_H
#define DYSON_UART_H

//#define DYSON_TEST 1 /* uncomment to print test info*/
#include "stdint.h"
#include "stddef.h"
#ifdef DYSON_TEST
    #include "TestDysonPackets.h"
#endif

/**
 * @brief Byte Stuffing settings
 */

#define START_BYTE 0x12
#define STOP_BYTE 0x12
#define STUFF_BYTE_1 0xDB
#define STUFF_BYTE_2 0xDE
#define STUFF_BYTE_3 0xDD

/**
  * @brief Structure to store received UART packet data
*/
typedef struct uart_packet_t
{
    uint8_t ptr[256];       // @brief Variable to store uart packet data
    size_t packet_size;     // @brief Size of ptr in bytes
    float time;             // @brief Time of recieved packets, seconds from the device start (optional)
        
}uart_packet_t;


/**
  * @brief Structure to store Dyson parameters data
*/
typedef struct Dyson_regs
{
    struct Flow_reg_t
    {
        uint16_t Flow1;
        uint16_t Flow2;
        uint16_t Flow3;
        uint16_t Flow4;
    } Flow_reg;

    uint32_t UpTime_reg;

    struct reg_2_0_4_0
    {
        double Temp1;
        double Temp2;
        double Temp3;
        double Humidity;
        double Vent_level;
    }Temp_reg;

    struct reg_2_0_3_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
        double Reg5;
    }reg_2_0_3_0;
    struct reg_2_0_2_0
    {
        uint8_t Reg1;
        uint8_t Reg2;
        uint8_t Reg3;
        uint8_t Reg4;
    }reg_2_0_2_0;

    struct reg_2_0_5_0
    {
        uint32_t Reg1;
        uint32_t Reg2;
        uint32_t Reg3;
    }reg_2_0_5_0;

        struct reg_2_0_6_0
    {
        uint16_t Reg1;
        uint16_t Reg2;

    }reg_2_0_6_0;

    struct reg_2_0_7_0
    {
        double Part_2_5;
        double Part_10;
        double Reg3;
        double Reg4;
        double Reg5;
        double Reg6;
        double Reg7;
        double Reg8;
        double Reg9;
        double Reg10;
        double Reg11;
        double Reg12;
        double Reg13;
    }reg_2_0_7_0;

    struct reg_2_0_8_0
    {
        uint8_t Part_2_5_int;
        uint8_t Reg2;
        uint8_t Reg3;
        uint8_t Reg4;
        uint8_t Reg5;
        uint8_t Reg6;
        uint8_t Reg7;
        uint8_t Reg8;
        uint8_t Reg9;
        uint8_t Reg10;
        uint8_t Part_10_int;
        uint8_t Reg12;
    }reg_2_0_8_0;

    struct reg_2_0_9_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
        double Reg5;
    }reg_2_0_9_0;

    struct reg_2_0_10_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
        double Reg5;
        double Reg6;
        double Reg7;
    }reg_2_0_10_0;

    struct reg_2_0_12_0
    {
        uint8_t Reg1;
        uint8_t Reg2;
        uint8_t Reg3;
        uint8_t Reg4;

    }reg_2_0_12_0;

    struct reg_3_0_1_0
    {
        uint32_t Reg1;
        uint32_t Reg2;
    }reg_3_0_1_0;

    struct reg_3_0_5_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
    }reg_3_0_5_0;

    struct reg_3_0_6_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
        double Reg5;
    }reg_3_0_6_0;

    struct reg_3_0_7_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
        double Reg5;
    }reg_3_0_7_0;

    struct reg_3_0_8_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
    }reg_3_0_8_0;

    struct reg_3_0_9_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
        double Reg5;
        double Reg6;
    }reg_3_0_9_0;

    struct reg_3_0_10_0
    {
        double Reg1;
        double Reg2;
        double Reg3;
        double Reg4;
        double Up_time;
    }reg_3_0_10_0;
}Dyson_regs_t;

void ParseUartStream(uint8_t *rx_buf,size_t buf_size);

void PacketReadyCallback(uart_packet_t * newPacket);

uart_packet_t* UnstuffPacket(uart_packet_t* pack);

uint8_t ParseDysonPacket(uart_packet_t *pack,Dyson_regs_t *regs);

uint8_t ParseData(uint8_t *ptr, uint32_t reg);
#endif