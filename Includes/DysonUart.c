#include "DysonUart.h"
#include "stdlib.h"
#ifdef DYSON_TEST
    #include "stdio.h"
#endif

/*
  * @brief            Call this function to parse uart stream buffer  
  * @param  RX_buffer Pointer to buffer containing UART data
  * @param  buf_size  Size of RX_buffer
  * @retval           Nothing.

  */
void ParseUartStream(uint8_t *RX_buffer,size_t buf_size)
{
    uint8_t isPacketStarted=0;
    uart_packet_t *newPacket=NULL;
    
    for (size_t i = 0; i < buf_size; i++)
    {
        if (isPacketStarted)
        {
            newPacket->ptr[newPacket->packet_size++]=RX_buffer[i];
            if(RX_buffer[i]==START_BYTE)
            {
            #ifdef DYSON_TEST
                printf("UART packet recieve finished\n");
            #endif
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
            #ifdef DYSON_TEST
                printf("UART packet recieve started\n");
            #endif
                newPacket= (uart_packet_t*)malloc(sizeof(uart_packet_t));
                newPacket->packet_size=0;
                isPacketStarted=1;
                newPacket->ptr[newPacket->packet_size++]=RX_buffer[i];
            }
        }
    }
}
/*
  * @brief           Callback function. Called in ParseUartStream func when new uart packet has been created. 
                         It is a weak function, must be reimplemented in user code to get access to uart packet object
  * @param  ptr      Pointer to uart_packet_t object containing received data
  * @return          Nothing.
  */
__attribute__((weak)) void PacketReadyCallback(uart_packet_t * newPacket)
 {
    free(newPacket);
    #ifdef DYSON_TEST
        printf("WARNING: weak func is called, create strong func of PacketReadyCallback\n");
    #endif
 }

/*
  * @brief           Call this function to unstuffed received packet. This function allocates memory of uart_packet_t size
                         and returns a pointer to it
  * @param  ptr      Pointer to uart_packet_t object
  * @return          Pointer to unstuffed packet
  */
uart_packet_t* UnstuffPacket(uart_packet_t* pack)
{
    if(pack->packet_size<2)
        return NULL;
    if(*pack->ptr!=START_BYTE ||*(pack->ptr+pack->packet_size-1)!=START_BYTE)
        return NULL;
#ifdef DYSON_TEST
        printf("Unstuffing %u bytes, first byte i %u\n",pack->packet_size,*pack->ptr);
#endif
    uart_packet_t *newPack=(uart_packet_t*)malloc(sizeof(uart_packet_t));
    newPack->packet_size=0;
    for(int i=1,j=0; i<pack->packet_size-1;i++)
    {
        if(pack->ptr[i]==STUFF_BYTE_1)
        {
            if(pack->ptr[i+1]==STUFF_BYTE_2)
                newPack->ptr[j]=START_BYTE;
            else if(pack->ptr[i+1]==STUFF_BYTE_3)
                    newPack->ptr[j]=STUFF_BYTE_1;
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

/*
  * @brief           Call this function to parse registers data from dyson packet 
  * @param  ptr      Pointer to buffer containing data. Format: |var SIZE (1 byte) | N bytes in packet (1 byte) | 1..N vars of SIZE type
  * @param  reg      Pointer to Dyson_regs_t struct object
  * @retval          0 - unknown format
  * @retval          1 - succesfully parsed
  */
uint8_t ParseData(uint8_t *ptr, uint32_t reg)
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
#ifdef DYSON_TEST
            printf("ParseData: unknown format\n");
#endif
        return 0;
        break;
    }
    return 1;
}

/*
  * @brief           Call this function to parse uart packet. Packet must be unstuffed
  * @param  ptr      Pointer to uart_packet_t object
  * @param  reg      Pointer to Dyson_regs_t object 
  * @retval          0 - unknown format
  * @retval          1 - succesfully parsed
  * @retval          2 - CRC fail
  * @retval          3 - input variable pointer is NULL 
  */
uint8_t ParseDysonPacket(uart_packet_t *pack,Dyson_regs_t *regs)
{
    
    uint16_t packSize=0;
    uint32_t packCRC32=0;
    uint8_t *data_pos_ptr;
    uint8_t regpos=10;
    uint8_t parseRes=0;

    if(!pack||!regs)
        return 3;
    packSize=*pack->ptr;
    packSize|=((uint16_t)pack->ptr[1])<<8;
#ifdef DYSON_TEST
    printf("packSize= %u, packet->size = %u\n",packSize,pack->packet_size);
#endif
    if(packSize<18||packSize!=((uint16_t)pack->packet_size-3))
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
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->Flow_reg));
        }
        break;

    case 2:
        switch (pack->ptr[regpos+2])
        {
        case 2:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_2_0));
            break;
        case 3:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_3_0));
            break;
        case 4:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->Temp_reg));
            break;
        case 5:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_5_0));
            break;
        case 6:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_6_0));
            break;
        case 7:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_7_0));
            break;
        case 8:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_8_0));
            break;
        case 9:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_9_0));
            break;
        case 10:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_10_0));
            break;
        case 11:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->UpTime_reg));
            break;
         case 12:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_2_0_12_0));
            break;
        default:
#ifdef DYSON_TEST
            printf("unknown second reg: %d\n",pack->ptr[regpos+2]);
#endif
            break;
        }
        break;
    case 3:
        switch (pack->ptr[regpos+2])
        {
        case 1:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_1_0));
            break;
        case 5:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_5_0));
            break;
        case 6:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_6_0));
            break;
        case 7:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_7_0));
            break;
        case 8:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_8_0));
            break;
        case 9:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_9_0));
            break;
         case 10:
            parseRes=ParseData(data_pos_ptr,(uint32_t)&(regs->reg_3_0_10_0));
            break;
        default:
#ifdef DYSON_TEST
            printf("unknown second reg: %d\n",pack->ptr[regpos+2]);
#endif
            break;
        }
        break;
    default:
#ifdef DYSON_TEST
        printf("unknown first reg: %d\n",pack->ptr[regpos]);
#endif
        break;
    }

return parseRes;
}
