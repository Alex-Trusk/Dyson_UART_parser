#pragma once
#include "stdint.h"
#include "esp_err.h"
#include "driver/timer.h"

#define TIMER_DIVIDER   (80)        // Timer pre-scaler. Assuming that APB clock set to 80 MHz

/**
 * @brief Stores the settings of timer.  hard_timer_init() allocates memory for the hard_timer_info_t object.
 *
 * Call hard_timer_deinit() after timer usage to free memory in heap. 
 * 
 */
typedef struct {
    uint8_t timer_group;
    uint8_t timer_idx;
    uint64_t alarm_interval;
    bool auto_reload;
    void* arg;
    void (*callback_func)(void*);
} hard_timer_info_t;

/**
 * @brief Handle for each hardware timer. 
 * 
 */
typedef struct hard_timer_info_t *hard_timer_handle_t;

/**
 * @brief Initializes new hardware timer object with given parameters
 * 
 *
 * @param timer_group       [in] Timer group, 0 for TIMERG0 or 1 for TIMERG1
 * @param timer_idx         [in] Timer index in timer group
 * @param autoreload        [in] Timer auto-reload. 0 - disabled, 1 - enabled
 * @param count_direction   [in] Timer counter direction. 0 - downward, 1 - upwards
 * @return                  Hard Timer Handle
 */
hard_timer_handle_t hard_timer_init(uint8_t timer_group, uint8_t timer_idx, bool autoreload, uint8_t count_direction);

/**
 * @brief Sets parameters for a specific timer
 * 
 * @param timer_handle  [in] Handle of a specific timer
 * @param mc_sec        [in] Timer alarm value in microseconds
 * @param callback_func [in] Pointer to callback function
 * @param callback_arg  [in] Arguments for a callback function
 */
void hard_timer_set(hard_timer_handle_t timer_handle, uint32_t mc_sec, void (*callback_func)(void*), void* callback_arg);

/**
 * @brief Starts the timer counter
 * 
 *
 * @param timer_handle [in] Handle of a specific timer
 */
void hard_timer_start(hard_timer_handle_t timer_handle);

/**
 * @brief Sets alarm value of a timer
 * 
 *
 * @param timer_handle  [in] Handle of a specific timer
 * @param mc_sec        [in] Alarm value in microseconds
 */
void hard_timer_set_alarm(hard_timer_handle_t timer_handle, uint32_t mc_sec);

/**
 * @brief Stops the timer counter
 * 
 *
 * @param timer_handle [in] Handle of a specific timer
 */
void hard_timer_pause(hard_timer_handle_t timer_handle);

/**
 * @brief Sets alarm value of a timer.
 * 
 * @note                Call this function from interrupt context
 * @param timer_handle  [in] Handle of a specific timer
 * @param mc_sec        [in] Alarm value in microseconds
 */
void hard_timer_set_alarm_in_isr(hard_timer_handle_t timer_handle, uint32_t mc_sec);

/**
 * @brief Removes callback function from timer. Disables timer interrupts
 * 
 * @note               Call this function from interrupt context
 * @param timer_handle [in] Handle of a specific timer
 */
void hard_timer_remove_callback_from_isr(hard_timer_handle_t timer_handle);

/**
 * @brief De-initialize timer. Free heap memory.
 * 
 * @param timer_handle [in] Handle of a specific timer
 */
void hard_timer_deinit(hard_timer_handle_t timer_handle);

/**
 * @brief Gets current info about specific timer
 * 
 *
 * @param timer_handle  [in] Handle of a specific timer
 * @return              Pointer to hard_timer_info_t object
 */
hard_timer_info_t* hard_timer_get_info(hard_timer_handle_t timer_handle);
