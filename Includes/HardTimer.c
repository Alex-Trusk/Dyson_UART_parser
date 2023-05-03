#include "HardTimer.h"
#include "stdlib.h"

hard_timer_handle_t hard_timer_init(uint8_t timer_group, uint8_t timer_idx, bool autoreload, uint8_t count_direction)
{
    timer_config_t config = 
    {
        .divider = TIMER_DIVIDER,
        .counter_dir = count_direction,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = autoreload,
        .clk_src = TIMER_SRC_CLK_APB
    };
    hard_timer_info_t *timer_info= calloc(1,sizeof(hard_timer_info_t));
    if(!timer_info)
        return NULL;
    timer_info->alarm_interval=0;
    timer_info->arg=NULL;
    timer_info->callback_func=NULL;
    timer_info->auto_reload=autoreload;
    timer_info->timer_group=timer_group;
    timer_info->timer_idx=timer_idx;

    hard_timer_handle_t ret_handle = timer_info;
    ESP_ERROR_CHECK(timer_init(timer_group, timer_idx, &config));
    return ret_handle;
}

void hard_timer_set(hard_timer_handle_t timer_handle, uint32_t mc_sec, void (*callback_func)(void*), void* callback_arg)
{
    hard_timer_info_t *timer_info=timer_handle;
    uint64_t alarmValue = (80000000/(TIMER_DIVIDER*1000000))*mc_sec;
    ESP_ERROR_CHECK(timer_set_counter_value(timer_info->timer_group, timer_info->timer_idx, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(timer_info->timer_group, timer_info->timer_idx, alarmValue));
    if(callback_func)
    {
        timer_info->callback_func=callback_func;
        timer_info->arg=callback_arg;
        ESP_ERROR_CHECK(timer_enable_intr(timer_info->timer_group, timer_info->timer_idx));
        ESP_ERROR_CHECK(timer_isr_callback_add(timer_info->timer_group, timer_info->timer_idx, callback_func, timer_info, ESP_INTR_FLAG_LEVEL2));
    }
}

void hard_timer_set_alarm(hard_timer_handle_t timer_handle, uint32_t mc_sec)
{
    uint64_t alarmValue = (80000000/(TIMER_DIVIDER*1000000))*mc_sec;
    hard_timer_info_t *timer_info=timer_handle;
    timer_info->alarm_interval=alarmValue;
    //ESP_ERROR_CHECK(timer_set_counter_value(IR_TIMER_GROUP, IR_TIMER, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(timer_info->timer_group , timer_info->timer_idx, alarmValue));
}

void hard_timer_set_alarm_in_isr(hard_timer_handle_t timer_handle, uint32_t mc_sec)
{
    //uint64_t alarmValue = (80000000/(TIMER_DIVIDER*1000000))*mc_sec;
    timer_group_set_alarm_value_in_isr(((hard_timer_info_t*)  timer_handle)->timer_group, ((hard_timer_info_t*)  timer_handle)->timer_idx, 
                                        (80000000/(TIMER_DIVIDER*1000000))*mc_sec);
}

void hard_timer_start(hard_timer_handle_t timer_handle)
{
    ESP_ERROR_CHECK(timer_start(((hard_timer_info_t*)  timer_handle)->timer_group, ((hard_timer_info_t*)  timer_handle)->timer_idx));
}

void hard_timer_pause(hard_timer_handle_t timer_handle)
{
    ESP_ERROR_CHECK(timer_pause(((hard_timer_info_t*)  timer_handle)->timer_group, ((hard_timer_info_t*)  timer_handle)->timer_idx));
}

void hard_timer_remove_callback_from_isr(hard_timer_handle_t timer_handle)
{
    timer_enable_intr(((hard_timer_info_t*)  timer_handle)->timer_group, ((hard_timer_info_t*)  timer_handle)->timer_idx);
    timer_isr_callback_remove(((hard_timer_info_t*)  timer_handle)->timer_group, ((hard_timer_info_t*)  timer_handle)->timer_idx);
}

void hard_timer_deinit(hard_timer_handle_t timer_handle)
{
    hard_timer_info_t *timer_info=timer_handle;
    if(timer_info->callback_func)
        timer_isr_callback_remove(timer_info->timer_group, timer_info->timer_idx);
    timer_deinit(timer_info->timer_group, timer_info->timer_idx);
    free(timer_info);
}

hard_timer_info_t* hard_timer_get_info(hard_timer_handle_t timer_handle)
{
    return (hard_timer_info_t*)timer_handle;
}