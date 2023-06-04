
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "../Includes/DysonUart.h"
#include <esp_wifi.h>
#include <esp_netif.h>
#include "../components/esp32-wifi-manager/src/wifi_manager.h"
#include "cJSON.h"
#include "../components/esp32-wifi-manager/src/http_app.h"
#include"../Includes/IRimitator.h"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION>ESP_IDF_VERSION_VAL(4,4,4)
    #define IDF_VER_5
#endif

//*** Settings section *///
//#define DYSON_DEBUG               /* uncomment to print debug info*/
//#define TERMINAL_CLEAR_SCREEN     /* uncomment to enable debug terminal clear screen option */
#define USE_HARDWARE_TIMER          /* comment to use software ESP timer*/
                                    /* Warning! Software timer can be inaccurate because of low priority*/
#define IR_PIN (GPIO_NUM_4)             // This Pin connected to IR cable from Dyson main board
#define TXD_PIN (GPIO_NUM_17)           //UART TX pin
#define RXD_PIN (GPIO_NUM_16)           //UART RX pin
#define CONFIG_BLINK_GPIO GPIO_NUM_2    //Onboard led pin
//*** End of Settings section *///

#ifdef TERMINAL_CLEAR_SCREEN
    #include "linenoise/linenoise.h"
#endif
#ifdef USE_HARDWARE_TIMER
    #ifndef IDF_VER_5
        #define IR_TIMER_GROUP TIMER_GROUP_0
        #define IR_TIMER TIMER_0
        #include"../Includes/HardTimer.h"
        static bool  timer_group_isr_callback(void * args);
        hard_timer_handle_t irTimerHandle;
    #else
        #include "driver/gptimer.h"
        gptimer_handle_t ir_gptimer = NULL;
    #endif
#else
    #include "esp_timer.h"
    esp_timer_handle_t ir_timer_handle;
    void ir_timer_callback(void* arg);   
#endif

#ifdef DYSON_DEBUG
    #include "esp_heap_trace.h"   //For memory leakage testing purposes only
    #include "esp_heap_caps.h"    //For memory leakage testing purposes only
#endif

/*** Global variables***/
uint16_t callbackCount=0;               //for testing purposes
static const int RX_BUF_SIZE = 1024;    //size of UART RX buffer
QueueHandle_t UartQueueHandle;          //Queue for Dyson UART packets
QueueHandle_t IRQueueHandle;            //Queue for IR imitator commands
QueueHandle_t LedQueueHandle;           //Queue for led modes
cJSON *DysonIRcommands;                 //Dyson IR commands list
Dyson_regs_t dyson_reg;                 //All dyson reg data will be stored here
#ifdef DYSON_DEBUG
    #define NUM_RECORDS 256
    static heap_trace_record_t trace_record[NUM_RECORDS];
#endif

/***End of GV section***/

/*** Prototypes section***/
uint8_t ParseHTTPcommand(const char *command_str);
void dysonIR_init();
void Dyson_SetFanLevel(uint8_t new_level);
void SetLedState(uint8_t newLedState);
/*** End of prototypes section***/

#ifdef DYSON_DEBUG
    void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
    {
        printf("%s was called but failed to allocate %d bytes with 0x%lX capabilities. \n",function_name, requested_size, caps);
    }
#endif

/**
 * @brief Setup function
 *
 */
void init(void) 
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
     
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);    //Installing uart without TX buffer
    uart_param_config(UART_NUM_1, &uart_config);                        //Configuring uart
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); 
    memset(&dyson_reg,0,sizeof(Dyson_regs_t));                          //Initializing Dyson Regs object
#ifdef DYSON_DEBUG
    ESP_ERROR_CHECK(heap_trace_init_standalone(trace_record, NUM_RECORDS));
#endif
    dysonIR_init();                                                     
#ifdef USE_HARDWARE_TIMER
    #ifndef IDF_VER_5
        irTimerHandle=hard_timer_init(IR_TIMER_GROUP,IR_TIMER,TIMER_AUTORELOAD_EN,TIMER_COUNT_UP);
    #else
        gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &ir_gptimer)); //Creating new timer for IR imitator
    #endif
#endif
}

/**
  * @brief FreeRTOS task. Constantly reading data from UART pin in blocking mode
  */
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    uint32_t rxBytes=0;
    while (1) {
        rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            
#ifdef DYSON_DEBUG
            ESP_LOGI(RX_TASK_TAG, "Read %lu bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
#endif
            if(!ParseUartStream(data,rxBytes))
            {
                esp_log_level_set(RX_TASK_TAG, ESP_LOG_ERROR);
                ESP_LOGE(RX_TASK_TAG, "Allocation failed!!!");
                esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    free(data);
}

/**
  * @brief FreeRTOS task. Processing data from UART buffer to retrieve dyson regs info
  */
static void Dyson_data_process(void *arg)
{
    static const char *DYSON_TASK_TAG = "DYSON_TASK";
    uart_packet_t *unstuffed_pack;
    uint32_t packet_buf;
    esp_log_level_set(DYSON_TASK_TAG, ESP_LOG_DEBUG);
    ESP_LOGI(DYSON_TASK_TAG, "Dyson data task started!");
    while (1) 
    {
        if(xQueueReceive(UartQueueHandle,&packet_buf, portMAX_DELAY))       //Waiting for message from parser
        {
#ifdef DYSON_DEBUG
            ESP_LOGI(DYSON_TASK_TAG, "Packet received! %u bytes, %u - first byte",((uart_packet_t*)packet_buf)->packet_size,
                    ((uart_packet_t*)packet_buf)->ptr[0]);
#endif
            if(!packet_buf)
                ESP_LOGI(DYSON_TASK_TAG, "Packet is null-pointer");
            else
            {
                unstuffed_pack=UnstuffPacket((uart_packet_t*)packet_buf);   
                if(unstuffed_pack)
                {
#ifdef DYSON_DEBUG
                    ESP_LOGI(DYSON_TASK_TAG, "Packet received! %u bytes, %u - first byte",unstuffed_pack->packet_size,unstuffed_pack->ptr[0]);
                    ESP_LOG_BUFFER_HEXDUMP(DYSON_TASK_TAG, unstuffed_pack->ptr, unstuffed_pack->packet_size, ESP_LOG_INFO);
#endif
                    free((uart_packet_t*)packet_buf);
                    if(!ParseDysonPacket(unstuffed_pack,&dyson_reg))
                        ESP_LOGI(DYSON_TASK_TAG, "Parser: Packet format unknown");
                    free(unstuffed_pack);
                        
                }
                else
                {
                    ESP_LOGI(DYSON_TASK_TAG, "Unstuffer: Packet format unknown");
                    free((uart_packet_t*)packet_buf);
                }
            }
        }
    }
}

/**
  * @brief  FreeRTOS task. Controls onboard led. 
  * Led can be set to off, on or blinking mode.
  * 
  */
static void Blynk_led(void *arg)
{
    static const char *BLYNK_LED_TAG = "BLYNK_LED";
    esp_log_level_set(BLYNK_LED_TAG, ESP_LOG_INFO);
    uint8_t newLedState, blink_mode_on=1;
    uint8_t s_led_state =0;
    ESP_LOGI(BLYNK_LED_TAG, "Configure GPIO LED!");
    gpio_reset_pin(CONFIG_BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CONFIG_BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1)
    {
        if(xQueueReceive(LedQueueHandle,&newLedState, 0))       //Waiting for message from parser
        {
            printf("State led blink= %d\n",newLedState);
            switch (newLedState)
            {
            case 0:
                gpio_set_level(CONFIG_BLINK_GPIO, 0);
                blink_mode_on=0;
                break;
            case 1:
                gpio_set_level(CONFIG_BLINK_GPIO, 1);
                blink_mode_on=0;
                break;
            case 2:
                blink_mode_on=1;
                break;
            default:
                break;
            }
        }
            
        if(blink_mode_on)
        {
            gpio_set_level(CONFIG_BLINK_GPIO, s_led_state);
            s_led_state=!s_led_state;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
  * @brief FreeRTOS task. Repeatedly prints values of dyson_reg using ESP logging
*/
static void DysonInfo_task(void *arg)
{
    static const char *DYSON_INFO_TAG = "DYSON_INFO";
    esp_log_level_set(DYSON_INFO_TAG, ESP_LOG_INFO);                        // change to ESP_LOG_DEBUG to get dyson regs states
    ESP_LOGI(DYSON_INFO_TAG, "Dysoninfo data task started!");
    while(1)
    {
#ifdef TERMINAL_CLEAR_SCREEN
        linenoiseClearScreen(); 
#endif
        ESP_LOGI(DYSON_INFO_TAG,"Temp1= %0.2f, Temp2= %0.2f, Temp3= %0.2f, Humidity= %0.2f, Vent_level= %0.2f",
                dyson_reg.Temp_reg.Temp1,dyson_reg.Temp_reg.Temp2,dyson_reg.Temp_reg.Temp3,dyson_reg.Temp_reg.Humidity,
                dyson_reg.Temp_reg.Vent_level);
#ifdef DYSON_DEBUG
        ESP_LOGI(DYSON_INFO_TAG,"Free heap= %lu",xPortGetFreeHeapSize());
        ESP_LOGI(DYSON_INFO_TAG,"Queue size= %u",uxQueueMessagesWaiting(UartQueueHandle));
        heap_trace_stop();
        heap_trace_dump();
        esp_log_level_set(DYSON_INFO_TAG, ESP_LOG_DEBUG);

#endif        
        ESP_LOGD(DYSON_INFO_TAG,"Flow1= %u, Flow2= %u, Flow3= %u, Flow4= %u",dyson_reg.Flow_reg.Flow1, dyson_reg.Flow_reg.Flow2,
                dyson_reg.Flow_reg.Flow3, dyson_reg.Flow_reg.Flow4);

        ESP_LOGD(DYSON_INFO_TAG,"2_0_3_reg1= %0.2f, 2_0_3_reg2= %0.2f, 2_0_3_reg3= %0.2f, 2_0_3_reg4= %0.2f, 2_0_3_reg5= %0.2f",
                dyson_reg.reg_2_0_3_0.Reg1,dyson_reg.reg_2_0_3_0.Reg2,dyson_reg.reg_2_0_3_0.Reg3,dyson_reg.reg_2_0_3_0.Reg4,
                dyson_reg.reg_2_0_3_0.Reg5);

        ESP_LOGD(DYSON_INFO_TAG,"2_0_5_reg1= %lu, 2_0_5_reg2= %lu, 2_0_5_reg3= %lu",
                dyson_reg.reg_2_0_5_0.Reg1,dyson_reg.reg_2_0_5_0.Reg2,dyson_reg.reg_2_0_5_0.Reg3);


        ESP_LOGD(DYSON_INFO_TAG,"Part_2.5= %0.2f, Part_10= %0.2f, 2_0_7_reg3= %0.2f, 2_0_7_reg4= %0.2f, 2_0_7_reg5= %0.2f",
                dyson_reg.reg_2_0_7_0.Part_2_5, dyson_reg.reg_2_0_7_0.Part_10, dyson_reg.reg_2_0_7_0.Reg3, dyson_reg.reg_2_0_7_0.Reg4,
                dyson_reg.reg_2_0_7_0.Reg5);
        ESP_LOGD(DYSON_INFO_TAG,"2_0_7_reg6= %0.2f, 2_0_7_reg7= %0.2f, 2_0_7_reg8= %0.2f, 2_0_7_reg9= %0.2f, 2_0_7_reg10= %0.2f",
                dyson_reg.reg_2_0_7_0.Reg6, dyson_reg.reg_2_0_7_0.Reg7, dyson_reg.reg_2_0_7_0.Reg8, dyson_reg.reg_2_0_7_0.Reg9,
                dyson_reg.reg_2_0_7_0.Reg10);
        ESP_LOGD(DYSON_INFO_TAG,"2_0_7_reg11= %0.2f, 2_0_7_reg12= %0.2f, 2_0_7_reg13= %0.2f",dyson_reg.reg_2_0_7_0.Reg11, 
                dyson_reg.reg_2_0_7_0.Reg12, dyson_reg.reg_2_0_7_0.Reg13);

        ESP_LOGD(DYSON_INFO_TAG,"Part_2_5int= %u, 2_0_8_reg2= %u, 2_0_8_reg3= %u, 2_0_8_reg4= %u, 2_0_8_reg5= %u",
                dyson_reg.reg_2_0_8_0.Part_2_5_int, dyson_reg.reg_2_0_8_0.Reg2, dyson_reg.reg_2_0_8_0.Reg3, dyson_reg.reg_2_0_8_0.Reg4,
                dyson_reg.reg_2_0_8_0.Reg5);
        ESP_LOGD(DYSON_INFO_TAG,"2_0_8_reg6= %u, 2_0_8_reg7= %u, 2_0_8_reg8= %u, 2_0_8_reg9= %u, 2_0_8_reg10= %u",
                dyson_reg.reg_2_0_8_0.Reg6, dyson_reg.reg_2_0_8_0.Reg7, dyson_reg.reg_2_0_8_0.Reg8, dyson_reg.reg_2_0_8_0.Reg9,
                dyson_reg.reg_2_0_8_0.Reg10);
        ESP_LOGD(DYSON_INFO_TAG,"2_0_8_reg11= %u, 2_0_8_reg12= %u",dyson_reg.reg_2_0_8_0.Part_10_int, 
                dyson_reg.reg_2_0_8_0.Reg12);

        ESP_LOGD(DYSON_INFO_TAG,"3_0_5_reg1= %0.2f, 3_0_5_reg2= %0.2f, 3_0_5_reg3= %0.2f, 3_0_5_reg4= %0.2f",
                dyson_reg.reg_3_0_5_0.Reg1, dyson_reg.reg_3_0_5_0.Reg2, dyson_reg.reg_3_0_5_0.Reg3, dyson_reg.reg_3_0_5_0.Reg4);

        ESP_LOGD(DYSON_INFO_TAG,"3_0_6_reg1= %0.2f, 3_0_6_reg2= %0.2f, 3_0_6_reg3= %0.2f, 3_0_6_reg4= %0.2f, 3_0_6_reg5= %0.2f",
                dyson_reg.reg_3_0_6_0.Reg1, dyson_reg.reg_3_0_6_0.Reg2, dyson_reg.reg_3_0_6_0.Reg3, dyson_reg.reg_3_0_6_0.Reg4,
                dyson_reg.reg_3_0_6_0.Reg5);

        ESP_LOGD(DYSON_INFO_TAG,"3_0_7_reg1= %0.2f, 3_0_7_reg2= %0.2f, 3_0_7_reg3= %0.2f, 3_0_7_reg4= %0.2f, 3_0_7_reg5= %0.2f",
                dyson_reg.reg_3_0_7_0.Reg1, dyson_reg.reg_3_0_7_0.Reg2, dyson_reg.reg_3_0_7_0.Reg3, dyson_reg.reg_3_0_7_0.Reg4,
                dyson_reg.reg_3_0_7_0.Reg5);

        ESP_LOGD(DYSON_INFO_TAG,"3_0_8_reg1= %0.2f, 3_0_8_reg2= %0.2f, 3_0_8_reg3= %0.2f, 3_0_8_reg4= %0.2f",
                dyson_reg.reg_3_0_8_0.Reg1, dyson_reg.reg_3_0_8_0.Reg2, dyson_reg.reg_3_0_8_0.Reg3, dyson_reg.reg_3_0_8_0.Reg4);
        
        ESP_LOGD(DYSON_INFO_TAG,"3_0_9_reg1= %0.2f, 3_0_9_reg2= %0.2f, 3_0_9_reg3= %0.2f, 3_0_9_reg4= %0.2f, 3_0_9_reg5= %0.2f, 3_0_9_reg6= %0.2f",
                dyson_reg.reg_3_0_9_0.Reg1, dyson_reg.reg_3_0_9_0.Reg2, dyson_reg.reg_3_0_9_0.Reg3, dyson_reg.reg_3_0_9_0.Reg4,
                dyson_reg.reg_3_0_9_0.Reg5, dyson_reg.reg_3_0_9_0.Reg6);

        ESP_LOGD(DYSON_INFO_TAG,"3_0_10_reg1= %0.2f, 3_0_10_reg2= %0.2f, 3_0_10_reg3= %0.2f, 3_0_10_reg4= %0.2f, 3_0_10_Up_time= %0.2f",
                dyson_reg.reg_3_0_10_0.Reg1, dyson_reg.reg_3_0_10_0.Reg2, dyson_reg.reg_3_0_10_0.Reg3, dyson_reg.reg_3_0_10_0.Reg4,
                dyson_reg.reg_3_0_10_0.Up_time);
        
        ESP_LOGD(DYSON_INFO_TAG,"2_0_2_Reg1= %u, 2_0_2_Reg2= %u, 2_0_2_Reg3= %u, 2_0_2_Reg4= %u",dyson_reg.reg_2_0_2_0.Reg1, dyson_reg.reg_2_0_2_0.Reg2,
                dyson_reg.reg_2_0_2_0.Reg3, dyson_reg.reg_2_0_2_0.Reg4);

        ESP_LOGD(DYSON_INFO_TAG,"2_0_6_Reg1= %u, 2_0_6_Reg2= %u",dyson_reg.reg_2_0_6_0.Reg1, dyson_reg.reg_2_0_6_0.Reg2);

        ESP_LOGD(DYSON_INFO_TAG,"2_0_10_reg1= %0.2f, 2_0_10_reg2= %0.2f, 2_0_10_reg3= %0.2f, 2_0_10_reg4= %0.2f, 2_0_10_reg5= %0.2f",
                dyson_reg.reg_2_0_10_0.Reg1, dyson_reg.reg_2_0_10_0.Reg2, dyson_reg.reg_2_0_10_0.Reg3, dyson_reg.reg_2_0_10_0.Reg4,
                dyson_reg.reg_2_0_10_0.Reg5);
        ESP_LOGD(DYSON_INFO_TAG,"2_0_10_reg6= %0.2f, 2_0_10_reg7= %0.2f",dyson_reg.reg_2_0_10_0.Reg6, dyson_reg.reg_2_0_10_0.Reg7);

        ESP_LOGD(DYSON_INFO_TAG,"3_0_1_reg1= %lu, 3_0_1_reg2= %lu",
                dyson_reg.reg_3_0_1_0.Reg1,dyson_reg.reg_3_0_1_0.Reg2);

        ESP_LOGD(DYSON_INFO_TAG,"2_0_12_Reg1= %u, 2_0_12_Reg2= %u, 2_0_12_Reg3= %u, 2_0_12_Reg4= %u",dyson_reg.reg_2_0_12_0.Reg1, dyson_reg.reg_2_0_12_0.Reg2,
                dyson_reg.reg_2_0_12_0.Reg3, dyson_reg.reg_2_0_12_0.Reg4);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * @brief Function is called after receiving GET request by HTTP server that was started by WifiManager
 * 
 * @param req [in] Pointer to httpd_req_t object
 * @return esp_err_t type
 */
static esp_err_t sensors_get_handler(httpd_req_t *req)
{
	static const char *TAG = "REQUEST HANDLER";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    if(strcmp(req->uri, "/api/states/sensors") == 0)
    {

		ESP_LOGI(TAG, "New GET request. Processing page /api/states/sensors");
        cJSON* sensor = cJSON_CreateObject();
        cJSON_AddStringToObject(sensor, "dev_name", "Dyson ESP Purifier");
        cJSON_AddNumberToObject(sensor, "state", GetWorkStateFromReg(&dyson_reg));
        cJSON_AddNumberToObject(sensor, "temperature", GetTempFromReg(&dyson_reg));
        cJSON_AddNumberToObject(sensor, "humidity", GetHumFromReg(&dyson_reg));
        cJSON_AddNumberToObject(sensor, "vent_level", GetVentLevelFromReg(&dyson_reg));
        cJSON_AddNumberToObject(sensor, "pm25_level", GetPart25FromReg(&dyson_reg));
        cJSON_AddNumberToObject(sensor, "pm10_level", GetPart10FromReg(&dyson_reg));
     
        // Serialize JSON payload and send to Home Assistant
        char* output = cJSON_Print(sensor);
        ESP_LOGD(TAG, "JSON = %s",output);
		httpd_resp_set_status(req, "200 OK");
		httpd_resp_set_type(req, "application/json");
		httpd_resp_send(req, output, strlen(output));
        cJSON_Delete(sensor);
        free(output);
	}
	else
    {
		/* send a 404 otherwise */
		httpd_resp_send_404(req);
	}

	return ESP_OK;
}


/**
 * @brief Function is called after receiving POST request by HTTP server that was started by WifiManager
 * 
 * @param req [in] Pointer to httpd_req_t object
 * @return esp_err_t type
 */
static esp_err_t sensors_post_handler(httpd_req_t *req)
{
	static const char *POST_TAG = "POST REQUEST HANDLER";
    esp_log_level_set(POST_TAG, ESP_LOG_INFO);
    char req_payload_buf[100];                                              // buffer for POST request payload

    static const char *output_OK="200 OK";                                  // response messages
    static const char *output_FAIL="400 Bad request";

    size_t req_buf_size = (sizeof(req_payload_buf)>req->content_len)?       // Setting payload buffer size
                            (req->content_len):sizeof(req_payload_buf)-1;
    size_t bytes_read;
    if(strcmp(req->uri, "/api/controls") == 0)
    {

		ESP_LOGI(POST_TAG, "New POST request. Processing page /api/controls");
        ESP_LOGI(POST_TAG, "msg_size = %u",req_buf_size);
    
        if ((bytes_read=httpd_req_recv(req, req_payload_buf,req_buf_size))<0)   // reading payload data from POST request
            return ESP_FAIL;                                                    // error check

        ESP_LOGI(POST_TAG, "bytes_read = %u",bytes_read);
        
        cJSON *json_payload =cJSON_ParseWithLength(req_payload_buf,bytes_read); // payload array to json parsing
        cJSON *command_item;
        cJSON* return_json = cJSON_CreateObject();
        if(!json_payload)                                                       // check for JSON parsing error
        {
            const char *error_ptr = cJSON_GetErrorPtr();                        // retrieving error
            if (error_ptr != NULL)
            {
                ESP_LOGE(POST_TAG, "cJSON error: %s\n", error_ptr);
            }
        
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, output_FAIL, strlen(output_FAIL));
            return ESP_OK;                                          
        }
        command_item=cJSON_GetObjectItem(json_payload,"attributes");                // looking for an object in json payload

        if (cJSON_IsString(command_item) && (command_item->valuestring != NULL))    
        {
            
            if(!ParseHTTPcommand(command_item->valuestring))                        // Calling command parser
            {                                                                       // Command not found.
                httpd_resp_set_status(req, "400 Bad Request");                      // Forming a response
                httpd_resp_set_type(req, "text/html");
                httpd_resp_send(req, output_FAIL, strlen(output_FAIL));             // Sending "Bad request" response. 
                cJSON_Delete(json_payload);
                return ESP_OK;
            }
            uint8_t newState=GetWorkStateFromReg(&dyson_reg);                       //preparing post response
            if(strcmp(command_item->valuestring,"on_off")==0)
                newState=!newState;
            cJSON_AddNumberToObject(return_json, "state", newState);
            SetWorkStateToReg(&dyson_reg,newState);
        }
        else if((command_item=cJSON_GetObjectItem(json_payload,"set_fan")))         //checking for fan speed command
        {
            if (cJSON_IsNumber(command_item))
                Dyson_SetFanLevel(command_item->valueint);
        }
        else if((command_item=cJSON_GetObjectItem(json_payload,"set_led_mode")))    //checking for led control command
        {
            if (cJSON_IsNumber(command_item))
                SetLedState(command_item->valueint);
        }
#ifdef DYSON_TEST 
        ESP_LOGI(POST_TAG,"req_payload_buf = %s",req_payload_buf);
        char *json_str=cJSON_Print(json_payload);                                   
        ESP_LOGI(POST_TAG, "bytes_read = %u",bytes_read);
        ESP_LOGI(POST_TAG, "json_str   = %ld",(uint32_t)json_str);
        if(json_str)
            ESP_LOGI(POST_TAG, "json_str   = %s",json_str);
        free(json_str);
#endif

        char* output = cJSON_Print(return_json);
        ESP_LOGD(POST_TAG, "JSON = %s",output);
        httpd_resp_set_status(req, "200 OK");   
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, output, strlen(output));                                   //sending response on POST request
        cJSON_Delete(return_json);
        cJSON_Delete(json_payload);
        free(output);
	}
	else
    {
		/* send a 404 otherwise */
		httpd_resp_send_404(req);
	}
	return ESP_OK;
}

/**
  * @brief FreeRTOS task. Controls IR imitator submodule.
*/
static void IR_port_control_task(void *arg)
{
    uint32_t IR_com_buf;                                            //buffer for  command messages 
    IRimitator_t IRsettings;                                        //object for ir settings
    
    static const char *IR_CONTROL_TASK_TAG = "IR_CONTROL_TASK";
    esp_log_level_set(IR_CONTROL_TASK_TAG, ESP_LOG_DEBUG);
    ESP_LOGI(IR_CONTROL_TASK_TAG, "IRimitator started!");
    IRimitatorInitDysonDefault(&IRsettings);                        //Initialization with default settings
    
    gpio_reset_pin(IR_PIN);                                         //Init IR pin, pull-up enabled
    gpio_set_direction(IR_PIN, GPIO_MODE_OUTPUT);                   //Set pin - output
    gpio_set_level(IR_PIN, 1);                                      //High is default level

    while (1) 
    {
        if(xQueueReceive(IRQueueHandle,&IR_com_buf, portMAX_DELAY)) //waiting for command
        {
            IRsettings.cur_command=IR_com_buf;
            IRimitatorSend(&IRsettings,IR_com_buf,21);              //sending command, 21 - number of significant bits in command
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }
}

void cb_connection_ok(void *pvParameter)
{
    uint8_t newLedState=0;
    xQueueSend(LedQueueHandle,(uint8_t*)&(newLedState),0);
}

void cb_disconnected(void *pvParameter)
{
    uint8_t newLedState=2;
    xQueueSend(LedQueueHandle,(uint8_t*)&(newLedState),0);
}

void app_main(void)
{
    init();                                                         //Calling main setup function
    ESP_LOGI("APP_MAIN", "Device Started!");
/*** FreeRTOS queue section***/
    UartQueueHandle=xQueueCreate(30,sizeof(uart_packet_t*));
    IRQueueHandle=xQueueCreate(10,sizeof(uint32_t));
    LedQueueHandle=xQueueCreate(3,sizeof(uint8_t));
/*** End of FreeRTOS queue section***/
    ESP_LOGI("QUEUE_TAG", "Queue created!");
/*** FreeRTOS tasks section***/
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(Dyson_data_process, "Dyson_data_task", 1024*5, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(Blynk_led, "Blynk_Led", 1024*2, NULL, configMAX_PRIORITIES-4, NULL);
    xTaskCreate(DysonInfo_task, "DysonInfo_task", 1024*3, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(IR_port_control_task, "IR_control_task", 1024*3, NULL, configMAX_PRIORITIES-3, NULL);
 /*** End of FreeRTOS tasks section***/

    wifi_manager_start();                                           //Wifi Manager captive portal start
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED , &cb_disconnected);
    http_app_set_handler_hook(HTTP_GET, &sensors_get_handler);      //Setting control func for all incoming GET requests
    http_app_set_handler_hook(HTTP_POST, &sensors_post_handler);    //Setting control func for all incoming POST requests

#ifdef DYSON_TEST
    esp_err_t error = heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook); 
#endif
}

/**
 * @brief This is an re-implementation of a weak PacketReadyCallback function. Called after new UART packet is received
 *          
 * @param newPacket Pointer to uart_packet_t object, containing received data
 */
void PacketReadyCallback(uart_packet_t * newPacket)
{
    uart_packet_t *flush_buf=NULL;
    static size_t totalPackNum=0;
    uint8_t q_free_space= uxQueueSpacesAvailable(UartQueueHandle);
#ifdef DYSON_DEBUG
    ESP_LOGI("P_READY_CALLBACK", "Callback fired!\nPacket received= %u, Queue free space= %u",(++totalPackNum),q_free_space);
#endif
    
    if(!q_free_space)
    {
        xQueueReceive(UartQueueHandle,flush_buf,0);
        free(flush_buf);
    }
    xQueueSend(UartQueueHandle,&newPacket,0);
}

#ifndef USE_HARDWARE_TIMER
/**
 * @brief Callback function called when IR timer interrupt occurs
 * 
 *
 * @param arg [out] Pointer to IRcommand_t object
 */
void ir_timer_callback(void* arg)
{
    IRcommand_t *command=(IRcommand_t*)arg;                             //converting from void to IRcommand_
    static uint8_t gpio_level = 0;                                      //GPIO level after the first launch will be 0
    if(command->next_pulse<command->size)                               //checking for end of command
    {
        gpio_level= (gpio_level==0)?1:0;                                //switching gpio level var
        gpio_set_level(IR_PIN, gpio_level);                             //IR pin toggle
        esp_timer_start_once(ir_timer_handle, command->durations[command->next_pulse]);  //starting timer for next pulse

        command->next_pulse++;
    }
    else                                                                //if end of command is reached
    {
        command->next_pulse=0;
        gpio_set_level(IR_PIN,1);                                       //setting gpio to default state
        free(command);                                                  //freeing memory
    }
}
#else
#ifndef IDF_VER_5
/**
 * @brief Callback function called when IR hardware timer interrupt occurs
 * 
 *
 * @param arg [out] Pointer to hard_timer_info_t object. IRcommand_t data stored in arg field of hard_timer_info_t object
 */
static bool  timer_group_isr_callback(void * args)
{
    IRcommand_t *command=(IRcommand_t*)((hard_timer_info_t*)args)->arg; //converting from void to IRcommand_t
    static uint8_t gpio_level = 0;                                      //GPIO level after the first launch will be 0
    callbackCount++;                                                    //used for debug 
    if(command->next_pulse<command->size)                               //checking for end of command
    {
        gpio_level= (gpio_level==0)?1:0;                                //switching gpio level var
        gpio_set_level(IR_PIN, gpio_level);                             //IR pin toggle
        hard_timer_set_alarm_in_isr(irTimerHandle, command->durations[command->next_pulse]); //setting pulse duration
        command->next_pulse++;
    }
    else                                                                //if end of command is reached
    {
        hard_timer_pause(irTimerHandle);
        gpio_set_level(IR_PIN,1);                                       //setting gpio to default state
        command->next_pulse=0;                                          
        hard_timer_remove_callback_from_isr(irTimerHandle);             //Otherwise exception will occur during the next command 
        free(command);                                                  //freeing memory
    }
    return 0;
}
#else
/**
 * @brief Callback function called when IR hardware timer interrupt occurs (ESP IDF ver 5 and above)
 * 
 * @param timer_h   Handler of specific timer
 * @param edata     Timer status obj
 * @param user_data User defined data in gptimer_register_event_callbacks() func
 * 
 */
static bool IRAM_ATTR gp_timer_callback(gptimer_handle_t timer_h, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    IRcommand_t *command=(IRcommand_t*)user_data; //converting from void to IRcommand_t
    static uint8_t gpio_level = 0;                                      //GPIO level after the first launch will be 0
    callbackCount++;                                                    //used for debug
    if(command->next_pulse<command->size)                               //checking for end of command
    {
        gpio_level= (gpio_level==0)?1:0;                                //switching gpio level var
        gpio_set_level(IR_PIN, gpio_level);                             //IR pin toggle
        gptimer_alarm_config_t alarm_config={
            .alarm_count=edata->alarm_value + command->durations[command->next_pulse],        //new pulse length
        };
        gptimer_set_alarm_action(timer_h, &alarm_config);               //setting pulse duration
        command->next_pulse++;
    }
    else                                                                //if end of command is reached
    {
        gptimer_stop(timer_h);
        gptimer_disable(timer_h);
        gpio_set_level(IR_PIN,1);                                       //setting gpio to default state
        command->next_pulse=0;                                          
        //hard_timer_remove_callback_from_isr(irTimerHandle);             //Otherwise exception will occur during the next command 
        free(command);                                                  //freeing memory
    }
    return 0;
}
#endif //IDF_VER_5
#endif //USE_HARDWARE_TIMER

void StartCommandSequence(IRcommand_t* command)                         //this is re-implementation of weak function defined in IRimitator.h
{
#ifndef USE_HARDWARE_TIMER

    esp_timer_handle_t  timer_handle;
    esp_timer_create_args_t ir_timer_args =                             //setting timer arguments
    {
        .callback = &ir_timer_callback,
        .arg = command,
        .name = "IRtimer"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&ir_timer_args, &ir_timer_handle));   //creating new timer
    gpio_set_level(IR_PIN, 0);                                          //setting GPIO to low level
    
    ESP_ERROR_CHECK(esp_timer_start_once(ir_timer_handle, command->durations[0]));     
    command->next_pulse=1;                                              //next pulse in sequence

#else
#ifdef DYSON_TEST 
    printf("Callback fired %d times\n",callbackCount);
#endif
    callbackCount=0;
    #ifndef IDF_VER_5
        hard_timer_set(irTimerHandle, command->durations[0],timer_group_isr_callback,command);  //Setting alarm value, callback func and callback payload
        hard_timer_start(irTimerHandle);
    #else
        gptimer_event_callbacks_t cbs={
            .on_alarm=gp_timer_callback,
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(ir_gptimer, &cbs, command));           //Setting callback and payload
        ESP_ERROR_CHECK(gptimer_enable(ir_gptimer));                            //Change timer state init -> enabled

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = command->durations[0], 
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(ir_gptimer, &alarm_config));   //Setting counter alarm value
        ESP_ERROR_CHECK(gptimer_set_raw_count(ir_gptimer,0));                   //Setting counter start value
        ESP_ERROR_CHECK(gptimer_start(ir_gptimer));                             //Change timer state enabled -> run

    #endif //IDF_VER_5
    gpio_set_level(IR_PIN, 0);              //Toggle IR imitator line from idle to active
    command->next_pulse=1;                  
#endif //USE_HARDWARE_TIMER
}
/**
 * @brief Function looks up for string command in DysonIRcommands JSON list, extracts associated uint32_t command
 * and sends it via IR queue 
 * 
 *
 * @param command_str [in] Pointer to command string
 * @retval 0 - command string not found
 * @retval 1 - command string found
 * 
 */
uint8_t ParseHTTPcommand(const char *command_str)
{
    if(!command_str)
        return 0;
    cJSON *json_item=cJSON_GetObjectItem(DysonIRcommands,command_str);                      // Stores single or multiple command values 
    
    if(json_item)
    {                       
        printf("JSON payload = %s\n",command_str);                                          //Printing the payload
        if(cJSON_IsArray(json_item))                                                        //check for multiple commands
        {
            uint8_t commands_count= cJSON_GetArraySize(json_item);
            cJSON *tmpItem;
            printf("com value = %d\n",json_item->valueint);
            for(uint8_t i = 0; i<commands_count;i++)                                        //sending sequence of commands
            {
                tmpItem=cJSON_GetArrayItem(json_item,i);
                printf("com value = %d\n",tmpItem->valueint);
                xQueueSend(IRQueueHandle,(void*)&(tmpItem->valueint),0);
            }
        }
        else if(cJSON_IsNumber(json_item))                                                  //check for single command
        {
            xQueueSend(IRQueueHandle,(void*)&(json_item->valueint),0);                      //sending command
            printf("com value = %d\n",json_item->valueint);
        }
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
 * @brief IR command list init function. Creates a JSON structure and stores possible variants
 * Command name stored in string format is associated with uint32_t value.
 */
void dysonIR_init()
{
    DysonIRcommands=cJSON_CreateObject();
    cJSON *tmp_num, *tmp_arr;
    uint32_t num_array[6];
    tmp_num=cJSON_CreateNumber(0x00070120);
    cJSON_AddItemToObject(DysonIRcommands,"on_off",tmp_num);

    tmp_num=cJSON_CreateNumber(0x00064320);
    cJSON_AddItemToObject(DysonIRcommands,"direction",tmp_num);

    tmp_num=cJSON_CreateNumber(0x0000D520);
    cJSON_AddItemToObject(DysonIRcommands,"vent_up",tmp_num);

    tmp_num=cJSON_CreateNumber(0x00043520);
    cJSON_AddItemToObject(DysonIRcommands,"vent_down",tmp_num);

    tmp_num=cJSON_CreateNumber(0x0001DB20);
    cJSON_AddItemToObject(DysonIRcommands,"info",tmp_num);
    
    tmp_num=cJSON_CreateNumber(0x00041320);
    cJSON_AddItemToObject(DysonIRcommands,"night_mode",tmp_num);

    tmp_num=cJSON_CreateNumber(0x00002B20);
    cJSON_AddItemToObject(DysonIRcommands,"auto_mode",tmp_num);

    tmp_num=cJSON_CreateNumber(0x00009920);
    cJSON_AddItemToObject(DysonIRcommands,"sw_rotation",tmp_num);

    num_array[0]=(cJSON_GetObjectItem(DysonIRcommands,"vent_up"))->valueint;        //////////////////////////////
    num_array[1]=(cJSON_GetObjectItem(DysonIRcommands,"vent_down"))->valueint;      //
    num_array[2]=(cJSON_GetObjectItem(DysonIRcommands,"auto_mode"))->valueint;      // AUTO Mode on command set
    tmp_arr=cJSON_CreateIntArray(num_array,3);                                      //
    cJSON_AddItemToObject(DysonIRcommands,"auto_mode_on",tmp_arr);                  //////////////////////////////
    
    num_array[0]=(cJSON_GetObjectItem(DysonIRcommands,"vent_up"))->valueint;        //////////////////////////////
    num_array[1]=(cJSON_GetObjectItem(DysonIRcommands,"vent_down"))->valueint;      //
    tmp_arr=cJSON_CreateIntArray(num_array,2);                                      //AUTO mode off command set
    cJSON_AddItemToObject(DysonIRcommands,"auto_mode_off",tmp_arr);                 //////////////////////////////
/*** For testing purposes***/

    char *commands_list = cJSON_Print(DysonIRcommands);
    printf("%s\n",commands_list);
    free(commands_list);
}

/**
 * @brief Sets the fan level of dyson purifier. Function checks current fan level
 * and sends as many vent_up or vent_down command to achieve desired fan level 
 *
 * @param new_level New level to set. A number from MIN_FAN_LEVEL to MAX_FAN_LEVEL
 */
void Dyson_SetFanLevel(uint8_t new_level)
{
    printf("fan level %u\n",new_level);
    if(new_level>MAX_FAN_LEVEL)
        new_level=MAX_FAN_LEVEL;
    if(new_level<MIN_FAN_LEVEL)
        new_level=MIN_FAN_LEVEL;
    uint32_t fan_up_command =(cJSON_GetObjectItem(DysonIRcommands,"vent_up"))->valueint;
    uint32_t fan_down_command =(cJSON_GetObjectItem(DysonIRcommands,"vent_down"))->valueint;
    uint8_t  curFanLevel=GetVentLevelFromReg(&dyson_reg);
    if(curFanLevel>new_level)
    {
        for(int i= curFanLevel-new_level;i>0;i--)
            xQueueSend(IRQueueHandle,(void*)&(fan_down_command),0);
    }
    else
    {
        for(int i= new_level-curFanLevel;i>0;i--)
            xQueueSend(IRQueueHandle,(void*)&(fan_up_command),0);
    }
}


void SetLedState(uint8_t newLedState)
{
    printf("State led = %d\n",newLedState);
    xQueueSend(LedQueueHandle,(uint8_t*)&(newLedState),0);
}