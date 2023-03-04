#ifndef DYSON_UART_H
#define DYSON_UART_H
#include "stdint.h"
#include "stddef.h"
#define START_BYTE 0x12
#define STOP_BYTE 0x12
#define STUFF_BYTE_1
typedef struct uart_packet_t
{
    uint8_t ptr[256];
     size_t packet_size;
    float time;
        
}uart_packet_t;

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
        double Temp4;
        double Temp5;
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
        double Reg1;
        double Reg2;
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
        uint8_t Reg1;
        uint8_t Reg2;
        uint8_t Reg3;
        uint8_t Reg4;
        uint8_t Reg5;
        uint8_t Reg6;
        uint8_t Reg7;
        uint8_t Reg8;
        uint8_t Reg9;
        uint8_t Reg10;
        uint8_t Reg11;
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
        double Reg5;
    }reg_3_0_10_0;
}Dyson_regs;
/*
#define BUFFER_SIZE 10
#define NUM_BUFFERS 2

typedef struct packets_buffer {
    uart_packet_t buffer[BUFFER_SIZE];
    size_t front;
    size_t rear;
}packets_buffer_t;



void push(packets_buffer_t *buf, int value) {
    if ((buf->front == 0 && buf->rear == BUFFER_SIZE - 1) || buf->rear == buf->front - 1) {
        printf("Buffer overflow!\n");
        return;
    }

    if (buf->front == -1 && buf->rear == -1) {
        buf->front = buf->rear = 0;
        buf->buffer[buf->rear] = value;
    } else if (buf->rear == BUFFER_SIZE - 1 && buf->front != 0) {
        buf->rear = 0;
        buf->buffer[buf->rear] = value;
    } else {
        buf->rear++;
        buf->buffer[buf->rear] = value;
    }
}

int remove(struct fifo_buffer *buf) {
    if (buf->front == -1) {
        printf("Buffer underflow!\n");
        return -1;
    }

    int value = buf->buffer[buf->front];
    if (buf->front == buf->rear) {
        buf->front = buf->rear = -1;
    } else if (buf->front == BUFFER_SIZE - 1) {
        buf->front = 0;
    } else {
        buf->front++;
    }
    return value;
}
*/
void ParseUartStream(uint8_t *rx_buf,size_t buf_size);

void PacketReadyCallback(uart_packet_t * newPacket);

uart_packet_t* UnstuffPacket(uart_packet_t* pack);

uint8_t ParseDysonPacket(uart_packet_t *pack,Dyson_regs *regs);

void ParseData(uint8_t *ptr, uint32_t reg);
#endif