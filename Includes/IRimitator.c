#include "IRimitator.h"
#include "stdlib.h"
#include "stdio.h"


IRcommand_t* GenerateIRsequence(IRimitator_t* source)
{
    IRcommand_t* out_command=malloc(sizeof(IRcommand_t));
    out_command->pulse_sequence=NULL;
    uint32_t command_tmp=source->cur_command;
    out_command->size=source->command_len*2;

    uint8_t  dur_pos=0;
    if(source->uHeaderHighTime)
    {
        out_command->durations[dur_pos++]=source->uHeaderHighTime;
        out_command->size++;
    }
    if(source->uHeaderLowTime)
    {
        out_command->durations[dur_pos++]=source->uHeaderLowTime;
        out_command->size++;
    }
    if(source->encoding==DELAY_ENC)
    {
        out_command->durations[dur_pos++]=source->uDelayTime;
        out_command->size++;
    }

    for(uint8_t i=0;i<source->command_len;++i)
    {
        if(CHECK_BIT(command_tmp,i))
        {
            out_command->durations[dur_pos++]=source->uOneBitTime; //if bit ='1'
            out_command->durations[dur_pos++]=source->uDelayTime;
        }
        else
        {
            out_command->durations[dur_pos++]=source->uZeroBitTime; //if bit ='0'
            out_command->durations[dur_pos++]=source->uDelayTime;

        }
    }
    return out_command;
    

}

uint8_t IRimitatorSend(IRimitator_t *IRimit,uint32_t ir_command, uint8_t comLen)
{
    if(!IRimit||!IRimit->uOneBitTime||!IRimit->uZeroBitTime)
        return 0;
    IRimit->cur_command=ir_command;
    if(comLen)
        IRimit->command_len=comLen;
    else
        return 0;
    
    IRcommand_t *new_command=GenerateIRsequence(IRimit);
    new_command->next_pulse=0;
    
    StartCommandSequence(new_command);
    
    return 1;


}

__attribute__((weak)) void StartCommandSequence(IRcommand_t* command)
{
    printf("WARNING: weak func is called. Function should be user reimplemented\n");
    if(command)
        free(command);
}

uint8_t IRimitatorInitDysonDefault(IRimitator_t *IRimit)
{
    if(!IRimit)
        return 0;
    return IRimitatorInit(IRimit, 2220,720,750,720,1450,LSB_FIRST,DELAY_ENC);
}

uint8_t IRimitatorInit(IRimitator_t *IRimit,uint16_t uHHtime,uint16_t uHLtime, uint16_t uDtime, 
                        uint16_t uZBtime,uint16_t uOBtime, direction_t bit_dir, ir_encoding_t enc)
{
    if(!IRimit)
        return 0;
    IRimit->uHeaderHighTime=uHHtime;
    IRimit->uHeaderLowTime=uHLtime;
    IRimit->uDelayTime=uDtime;
    IRimit->uOneBitTime=uOBtime;
    IRimit->uZeroBitTime=uZBtime;
    IRimit->bit_direction=bit_dir;
    IRimit->encoding=enc;
    return 1;

}