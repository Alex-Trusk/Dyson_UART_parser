#include "DysonUart.h"
#include "stdlib.h"
void ParseUartStream(uint8_t *RX_buffer,size_t buf_size)
{
    static uint8_t isPacketStarted=0;
    static uart_packet_t *prevPacket=NULL;
    uart_packet_t *newPacket=NULL;
    if(isPacketStarted&&prevPacket!=NULL)
        newPacket=prevPacket;

    for (size_t i = 0; i < buf_size; i++)
    {
        if (isPacketStarted)
        {
            newPacket->ptr[newPacket->packet_size++]=RX_buffer[i];
            if(RX_buffer[i]==START_BYTE)
            {
                PacketReadyCallback(newPacket);
                isPacketStarted=0;
            }
        }
        else
        {
            if(RX_buffer[i]==START_BYTE)
            {
                if(RX_buffer[i+1]==18 && i!=buf_size-1)
                    i++;
                newPacket= (uart_packet_t*)malloc(sizeof(uart_packet_t));
                newPacket->packet_size=0;
                isPacketStarted=1;
                newPacket->ptr[newPacket->packet_size++]=RX_buffer[i];
            }
        }
        
    }
    
}

__attribute__((weak)) void PacketReadyCallback(uart_packet_t * newPacket)
 {
    free(newPacket);
 }

uart_packet_t* UnstuffPacket(uart_packet_t* pack)
{
    if(pack->packet_size<2)
        return NULL;
    if(*pack->ptr!=18 ||*(pack->ptr+pack->packet_size-1)!=18)
        return NULL;
    uart_packet_t *newPack=(uart_packet_t*)malloc(sizeof(uart_packet_t));
    for(int i=1,j=0; i<pack->packet_size-1;i++)
    {
        if(pack->ptr[i]==219)
        {
            if(pack->ptr[i+1]==222)
                newPack->ptr[j]=18;
            else if(pack->ptr[i+1]==221)
                    newPack->ptr[j]=219;
            ++i;
        }
        else
            newPack->ptr[j]=pack->ptr[i];
        ++newPack->packet_size;
        ++j;
    }
    newPack->time=pack->time;
    return newPack;
}

void ParseData(uint8_t *ptr, uint32_t reg)
{
    switch (*ptr)
    {
    case 98:  //uint8_t data
        {
        uint8_t *d_ptr;
        d_ptr=(uint8_t*)reg;
        for(uint8_t i=0;i<*(ptr+1);++i)
        {
            *(d_ptr+i)=*(ptr+2+i);
        }
        }
        break;
    case 100: //uint16_t data
        {
        uint16_t *d_ptr;
        d_ptr=(uint16_t*)reg;
        for(uint8_t i=0;i<*(ptr+1);++i)
        {
            *(d_ptr+i)=0;
            *(d_ptr+i)|=*(ptr+2+i*2+1)<<8;
            *(d_ptr+i)|=*(ptr+2+i*2);
        }
        }
        break;
    case 102:   //uint32_t data
        {
        uint32_t *d_ptr;
        d_ptr=(uint32_t*)reg;
        for(uint8_t i=0;i<*(ptr+1);++i)
        {
            *(d_ptr+i)=0;
            *(d_ptr+i)|=*(ptr+2+i*4+3)<<24;
            *(d_ptr+i)|=*(ptr+2+i*4+2)<<16;
            *(d_ptr+i)|=*(ptr+2+i*4+1)<<8;
            *(d_ptr+i)|=*(ptr+2+i*4);
        }
        }
        break;
    case 107: //double-type data
        {
        double *d1_ptr;
        uint8_t *u8_ptr;
        d1_ptr=(double*)reg;
        for(uint8_t i=0;i<*(ptr+1);++i)
        {
            u8_ptr=(uint8_t*)(d1_ptr+i);
            *(u8_ptr+7)=*(ptr+2+i*8+7);
            *(u8_ptr+6)=*(ptr+2+i*8+6);
            *(u8_ptr+5)=*(ptr+2+i*8+5);
            *(u8_ptr+4)=*(ptr+2+i*8+4);
            *(u8_ptr+3)=*(ptr+2+i*8+3);
            *(u8_ptr+2)=*(ptr+2+i*8+2);
            *(u8_ptr+1)=*(ptr+2+i*8+1);
            *(u8_ptr)=*(ptr+2+i*8);

        }
        }
        break;

    default:
        break;
    }
}

uint8_t ParseDysonPacket(uart_packet_t *pack,Dyson_regs *regs)
{
    
    uint16_t packSize=0;
    uint32_t packCRC32=0;
    uint8_t *data_pos_ptr;
    uint8_t regpos=10;
    packSize=*pack->ptr;
    packSize|=((uint16_t)pack->ptr[1])<<8;
    if(packSize<18)
        return 1;
    //if(!CRC8_check(pacSize,pack->prt[2])) TODO: compose CRC8 func
        // return 2;
    packCRC32|=((uint32_t)pack->ptr[packSize+3-1])<<24;
    packCRC32|=((uint32_t)pack->ptr[packSize+3-2])<<16;
    packCRC32|=((uint32_t)pack->ptr[packSize+3-3])<<8;
    packCRC32|=((uint32_t)pack->ptr[packSize+3-4]);

    // if(!CRC32_check(packCRC32,&prt[3],packSize)) TODO: compose CRC32 func
        //return 2;
    if(pack->ptr[9]!=49)
        return 1;
    data_pos_ptr=&pack->ptr[regpos+4];
    switch(pack->ptr[regpos])
    {
    case 18:
        if (pack->ptr[regpos+1]==0&&pack->ptr[regpos+2]==1&&pack->ptr[regpos+3]==0)
        {
            ParseData(data_pos_ptr,(uint32_t)&(regs->Flow_reg));
        }
        break;

    case 2:
        switch (pack->ptr[regpos+2])
        {
        case 2:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_2_0));
            break;
        case 3:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_3_0));
            break;
        case 4:
            ParseData(data_pos_ptr,(uint32_t)&(regs->Temp_reg));
            break;
        case 5:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_5_0));
            break;
        case 6:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_6_0));
            break;
        case 7:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_7_0));
            break;
        case 8:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_8_0));
            break;
        case 9:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_9_0));
            break;
        case 10:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_10_0));
            break;
        case 11:
            ParseData(data_pos_ptr,(uint32_t)&(regs->UpTime_reg));
            break;
         case 12:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_12_0));
            break;
        default:
            break;
        }
        break;
    case 3:
        switch (pack->ptr[regpos+2])
        {
        case 1:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_1_0));
            break;
        case 5:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_5_0));
            break;
        case 6:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_6_0));
            break;
        case 7:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_7_0));
            break;
        case 8:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_8_0));
            break;
        case 9:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_9_0));
            break;
         case 10:
            ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_10_0));
            break;
        default:
            break;
        }
    default:
        break;
    }

return 0;
}
