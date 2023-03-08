/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "../Includes/DysonUart.h"
#include "linenoise/linenoise.h"
//#define DYSON_DEBUG


static const int RX_BUF_SIZE = 1024;
QueueHandle_t UartQueueHandle;
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define CONFIG_BLINK_GPIO GPIO_NUM_2
/*
Dyson_regs_t dyson_reg={
    .Flow_reg.Flow1=0, .Flow_reg.Flow2=0,.Flow_reg.Flow3=0,.Flow_reg.Flow4=0,
    .reg_2_0_10_0.Reg1=0.0f, .reg_2_0_10_0.Reg2=0.0f,.reg_2_0_10_0.Reg3=0.0f,.reg_2_0_10_0.Reg4=0.0f,.reg_2_0_10_0.Reg5=0.0f,
    .reg_2_0_10_0.Reg6=0.0f,.reg_2_0_10_0.Reg7=0.0f,
    .reg_2_0_12_0.Reg1=0,.reg_2_0_12_0.Reg2=0,.reg_2_0_12_0.Reg3=0,.reg_2_0_12_0.Reg4=0,
    .reg_2_0_2_0.Reg1=0,.reg_2_0_2_0.Reg2=0,.reg_2_0_2_0.Reg3=0,.reg_2_0_2_0.Reg4=0,
    .reg_2_0_3_0.Reg1=0.0f,.reg_2_0_3_0.Reg2=0.0f,.reg_2_0_3_0.Reg3=0.0f,.reg_2_0_3_0.Reg4=0.0f,.reg_2_0_3_0.Reg5=0.0f,
    .reg_2_0_5_0.Reg1=0,.reg_2_0_5_0.Reg2=0,.reg_2_0_5_0.Reg3=0,
    .reg_2_0_6_0.Reg1=0,.reg_2_0_6_0.Reg2=0,
    .reg_2_0_7_0.Part_2_5=0.0f,.reg_2_0_7_0.Part_10=0.0f,.reg_2_0_7_0.Reg3=0.0f,.reg_2_0_7_0.Reg4=0.0f,.reg_2_0_7_0.Reg5=0.0f,
    .reg_2_0_7_0.Reg6=0.0f,.reg_2_0_7_0.Reg7=0.0f,.reg_2_0_7_0.Reg8=0.0f,.reg_2_0_7_0.Reg9=0.0f,.reg_2_0_7_0.Reg10=0.0f,
    .reg_2_0_7_0.Reg11=0.0f,.reg_2_0_7_0.Reg12=0.0f,
    .reg_2_0_8_0.Part_2_5_int=0.0f,.reg_2_0_8_0.Reg2=0,.reg_2_0_8_0.Reg3=0,.reg_2_0_8_0.Reg4=0,.reg_2_0_8_0.Reg5=0,.reg_2_0_8_0.Reg6=0,
    .reg_2_0_8_0.Reg7=0.0f,.reg_2_0_8_0.Reg8=0.0f,.reg_2_0_8_0.Reg9=0.0f,.reg_2_0_8_0.Reg10=0.0f,.reg_2_0_8_0.Part_10_int=0.0f,.reg_2_0_8_0.Reg12=0.0f,
    .reg_2_0_9_0.Reg1=0.0f,.reg_2_0_9_0.Reg2=0.0f,.reg_2_0_9_0.Reg3=0.0f,.reg_2_0_9_0.Reg4=0.0f,.reg_2_0_9_0.Reg5=0.0f,
    .reg_3_0_10_0.Reg1=0.0f,.reg_3_0_10_0.Reg2=0.0f,.reg_3_0_10_0.Reg3=0.0f,.reg_3_0_10_0.Reg4=0.0f,.reg_3_0_10_0.Up_time=0.0f,
    .reg_3_0_1_0.Reg1=0,.reg_3_0_1_0.Reg2=0,
    .reg_3_0_5_0.Reg1=0.0f,.reg_3_0_5_0.Reg2=0.0f,.reg_3_0_5_0.Reg3=0.0f,.reg_3_0_5_0.Reg4=0.0f,
    .reg_3_0_6_0.Reg1=0.0f,.reg_3_0_6_0.Reg2=0.0f,.reg_3_0_6_0.Reg3=0.0f,.reg_3_0_6_0.Reg4=0.0f,.reg_3_0_6_0.Reg5=0.0f,
    .reg_3_0_7_0.Reg1=0.0f,.reg_3_0_7_0.Reg2=0.0f,.reg_3_0_7_0.Reg3=0.0f,.reg_3_0_7_0.Reg4=0.0f,.reg_3_0_7_0.Reg5=0.0f,
    .reg_3_0_8_0.Reg1=0.0f,.reg_3_0_8_0.Reg2=0.0f,.reg_3_0_8_0.Reg3=0.0f,.reg_3_0_8_0.Reg4=0.0f,
    .reg_3_0_9_0.Reg1=0.0f,.reg_3_0_9_0.Reg2=0.0f,.reg_3_0_9_0.Reg3=0.0f,.reg_3_0_9_0.Reg4=0.0f,.reg_3_0_9_0.Reg5=0.0f,
    .reg_3_0_9_0.Reg6=0.0f,
    .Temp_reg.Temp1=0.0f,.Temp_reg.Temp2=0.0f,.Temp_reg.Temp3=0.0f,.Temp_reg.Humidity=0.0f,.Temp_reg.Vent_level=0.0f,
    .UpTime_reg=0
};*/
Dyson_regs_t dyson_reg;

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    memset(&dyson_reg,0,sizeof(Dyson_regs_t));
}

int sendData(const char* logName, const char* data)
{
#ifdef DYSON_TEST    
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, TEST_UART_TEMP_FLOW, sizeof(TEST_UART_TEMP_FLOW));
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;

#else
    return 0;
#endif

}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    ESP_LOGI(TX_TASK_TAG, "UART TX Task started!");
    while (1) 
    {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    uint32_t rxBytes=0;
    while (1) {
        rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            //data[rxBytes] = 0;
#ifdef DYSON_DEBUG
            ESP_LOGI(RX_TASK_TAG, "Read %lu bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
#endif
            ParseUartStream(data,rxBytes);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    free(data);
}
static void Dyson_data_process(void *arg)
{
    static const char *DYSON_TASK_TAG = "DYSON_TASK";
    uart_packet_t *unstuffed_pack;
    uint32_t packet_buf;
    esp_log_level_set(DYSON_TASK_TAG, ESP_LOG_DEBUG);
    ESP_LOGI(DYSON_TASK_TAG, "Dyson data task started!");
    while (1) 
    {

        if(xQueueReceive(UartQueueHandle,&packet_buf, portMAX_DELAY))
        {
#ifdef DYSON_DEBUG
            ESP_LOGI(DYSON_TASK_TAG, "Packet recieved! %u bytes, %u - firstbyte",((uart_packet_t*)packet_buf)->packet_size,
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
                    ESP_LOGI(DYSON_TASK_TAG, "Packet recieved! %u bytes, %u - firstbyte",unstuffed_pack->packet_size,unstuffed_pack->ptr[0]);
                    ESP_LOG_BUFFER_HEXDUMP(DYSON_TASK_TAG, unstuffed_pack->ptr, unstuffed_pack->packet_size, ESP_LOG_INFO);
#endif
                    free((uart_packet_t*)packet_buf);
                    if(ParseDysonPacket(unstuffed_pack,&dyson_reg))
                        ESP_LOGI(DYSON_TASK_TAG, "Packet format unknown");
                    free(unstuffed_pack);
                        
                }
                else
                {
                    ESP_LOGI(DYSON_TASK_TAG, "Packet format unknown");
                    free((uart_packet_t*)packet_buf);
                }
            }
        
            
        }

    }
}

static void Blynk_led(void *arg)
{
    static const char *BLYNK_LED_TAG = "BLYNK_LED";
    esp_log_level_set(BLYNK_LED_TAG, ESP_LOG_INFO);

    uint8_t s_led_state =0;
    ESP_LOGI(BLYNK_LED_TAG, "Configure GPIO LED!");
    gpio_reset_pin(CONFIG_BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CONFIG_BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1)
    {
        gpio_set_level(CONFIG_BLINK_GPIO, s_led_state);
        s_led_state=!s_led_state;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
static void DysonInfo_task(void *arg)
{
    static const char *DYSON_INFO_TAG = "DYSON_INFO";
    esp_log_level_set(DYSON_INFO_TAG, ESP_LOG_INFO);
    ESP_LOGI(DYSON_INFO_TAG, "Dysoninfo data task started!");
    while(1)
    {

        linenoiseClearScreen();
        ESP_LOGI(DYSON_INFO_TAG,"Temp1= %0.2f, Temp2= %0.2f, Temp3= %0.2f, Humidity= %0.2f, Vent_level= %0.2f",
                dyson_reg.Temp_reg.Temp1,dyson_reg.Temp_reg.Temp2,dyson_reg.Temp_reg.Temp3,dyson_reg.Temp_reg.Humidity,
                dyson_reg.Temp_reg.Vent_level);

        ESP_LOGI(DYSON_INFO_TAG,"Flow1= %u, Flow2= %u, Flow3= %u, Flow4= %u",dyson_reg.Flow_reg.Flow1, dyson_reg.Flow_reg.Flow2,
                dyson_reg.Flow_reg.Flow3, dyson_reg.Flow_reg.Flow4);

        ESP_LOGI(DYSON_INFO_TAG,"2_0_3_reg1= %0.2f, 2_0_3_reg2= %0.2f, 2_0_3_reg3= %0.2f, 2_0_3_reg4= %0.2f, 2_0_3_reg5= %0.2f",
                dyson_reg.reg_2_0_3_0.Reg1,dyson_reg.reg_2_0_3_0.Reg2,dyson_reg.reg_2_0_3_0.Reg3,dyson_reg.reg_2_0_3_0.Reg4,
                dyson_reg.reg_2_0_3_0.Reg5);

        ESP_LOGI(DYSON_INFO_TAG,"2_0_5_reg1= %lu, 2_0_5_reg2= %lu, 2_0_5_reg3= %lu",
                dyson_reg.reg_2_0_5_0.Reg1,dyson_reg.reg_2_0_5_0.Reg2,dyson_reg.reg_2_0_5_0.Reg3);


        ESP_LOGI(DYSON_INFO_TAG,"Part_2.5= %0.2f, Part_10= %0.2f, 2_0_7_reg3= %0.2f, 2_0_7_reg4= %0.2f, 2_0_7_reg5= %0.2f",
                dyson_reg.reg_2_0_7_0.Part_2_5, dyson_reg.reg_2_0_7_0.Part_10, dyson_reg.reg_2_0_7_0.Reg3, dyson_reg.reg_2_0_7_0.Reg4,
                dyson_reg.reg_2_0_7_0.Reg5);
        ESP_LOGI(DYSON_INFO_TAG,"2_0_7_reg6= %0.2f, 2_0_7_reg7= %0.2f, 2_0_7_reg8= %0.2f, 2_0_7_reg9= %0.2f, 2_0_7_reg10= %0.2f",
                dyson_reg.reg_2_0_7_0.Reg6, dyson_reg.reg_2_0_7_0.Reg7, dyson_reg.reg_2_0_7_0.Reg8, dyson_reg.reg_2_0_7_0.Reg9,
                dyson_reg.reg_2_0_7_0.Reg10);
        ESP_LOGI(DYSON_INFO_TAG,"2_0_7_reg11= %0.2f, 2_0_7_reg12= %0.2f, 2_0_7_reg13= %0.2f",dyson_reg.reg_2_0_7_0.Reg11, 
                dyson_reg.reg_2_0_7_0.Reg12, dyson_reg.reg_2_0_7_0.Reg13);

        ESP_LOGI(DYSON_INFO_TAG,"Part_2_5int= %u, 2_0_8_reg2= %u, 2_0_8_reg3= %u, 2_0_8_reg4= %u, 2_0_8_reg5= %u",
                dyson_reg.reg_2_0_8_0.Part_2_5_int, dyson_reg.reg_2_0_8_0.Reg2, dyson_reg.reg_2_0_8_0.Reg3, dyson_reg.reg_2_0_8_0.Reg4,
                dyson_reg.reg_2_0_8_0.Reg5);
        ESP_LOGI(DYSON_INFO_TAG,"2_0_8_reg6= %u, 2_0_8_reg7= %u, 2_0_8_reg8= %u, 2_0_8_reg9= %u, 2_0_8_reg10= %u",
                dyson_reg.reg_2_0_8_0.Reg6, dyson_reg.reg_2_0_8_0.Reg7, dyson_reg.reg_2_0_8_0.Reg8, dyson_reg.reg_2_0_8_0.Reg9,
                dyson_reg.reg_2_0_8_0.Reg10);
        ESP_LOGI(DYSON_INFO_TAG,"2_0_8_reg11= %u, 2_0_8_reg12= %u",dyson_reg.reg_2_0_8_0.Part_10_int, 
                dyson_reg.reg_2_0_8_0.Reg12);

        ESP_LOGI(DYSON_INFO_TAG,"3_0_5_reg1= %0.2f, 3_0_5_reg2= %0.2f, 3_0_5_reg3= %0.2f, 3_0_5_reg4= %0.2f",
                dyson_reg.reg_3_0_5_0.Reg1, dyson_reg.reg_3_0_5_0.Reg2, dyson_reg.reg_3_0_5_0.Reg3, dyson_reg.reg_3_0_5_0.Reg4);

        ESP_LOGI(DYSON_INFO_TAG,"3_0_6_reg1= %0.2f, 3_0_6_reg2= %0.2f, 3_0_6_reg3= %0.2f, 3_0_6_reg4= %0.2f, 3_0_6_reg5= %0.2f",
                dyson_reg.reg_3_0_6_0.Reg1, dyson_reg.reg_3_0_6_0.Reg2, dyson_reg.reg_3_0_6_0.Reg3, dyson_reg.reg_3_0_6_0.Reg4,
                dyson_reg.reg_3_0_6_0.Reg5);

        ESP_LOGI(DYSON_INFO_TAG,"3_0_7_reg1= %0.2f, 3_0_7_reg2= %0.2f, 3_0_7_reg3= %0.2f, 3_0_7_reg4= %0.2f, 3_0_7_reg5= %0.2f",
                dyson_reg.reg_3_0_7_0.Reg1, dyson_reg.reg_3_0_7_0.Reg2, dyson_reg.reg_3_0_7_0.Reg3, dyson_reg.reg_3_0_7_0.Reg4,
                dyson_reg.reg_3_0_7_0.Reg5);

        ESP_LOGI(DYSON_INFO_TAG,"3_0_8_reg1= %0.2f, 3_0_8_reg2= %0.2f, 3_0_8_reg3= %0.2f, 3_0_8_reg4= %0.2f",
                dyson_reg.reg_3_0_8_0.Reg1, dyson_reg.reg_3_0_8_0.Reg2, dyson_reg.reg_3_0_8_0.Reg3, dyson_reg.reg_3_0_8_0.Reg4);
        
        ESP_LOGI(DYSON_INFO_TAG,"3_0_9_reg1= %0.2f, 3_0_9_reg2= %0.2f, 3_0_9_reg3= %0.2f, 3_0_9_reg4= %0.2f, 3_0_9_reg5= %0.2f, 3_0_9_reg6= %0.2f",
                dyson_reg.reg_3_0_9_0.Reg1, dyson_reg.reg_3_0_9_0.Reg2, dyson_reg.reg_3_0_9_0.Reg3, dyson_reg.reg_3_0_9_0.Reg4,
                dyson_reg.reg_3_0_9_0.Reg5, dyson_reg.reg_3_0_9_0.Reg6);

        ESP_LOGI(DYSON_INFO_TAG,"3_0_10_reg1= %0.2f, 3_0_10_reg2= %0.2f, 3_0_10_reg3= %0.2f, 3_0_10_reg4= %0.2f, 3_0_10_Up_time= %0.2f",
                dyson_reg.reg_3_0_10_0.Reg1, dyson_reg.reg_3_0_10_0.Reg2, dyson_reg.reg_3_0_10_0.Reg3, dyson_reg.reg_3_0_10_0.Reg4,
                dyson_reg.reg_3_0_10_0.Up_time);
        
        ESP_LOGI(DYSON_INFO_TAG,"2_0_2_Reg1= %u, 2_0_2_Reg2= %u, 2_0_2_Reg3= %u, 2_0_2_Reg4= %u",dyson_reg.reg_2_0_2_0.Reg1, dyson_reg.reg_2_0_2_0.Reg2,
                dyson_reg.reg_2_0_2_0.Reg3, dyson_reg.reg_2_0_2_0.Reg4);

        ESP_LOGI(DYSON_INFO_TAG,"2_0_6_Reg1= %u, 2_0_6_Reg2= %u",dyson_reg.reg_2_0_6_0.Reg1, dyson_reg.reg_2_0_6_0.Reg2);

        ESP_LOGI(DYSON_INFO_TAG,"2_0_10_reg1= %0.2f, 2_0_10_reg2= %0.2f, 2_0_10_reg3= %0.2f, 2_0_10_reg4= %0.2f, 2_0_10_reg5= %0.2f",
                dyson_reg.reg_2_0_10_0.Reg1, dyson_reg.reg_2_0_10_0.Reg2, dyson_reg.reg_2_0_10_0.Reg3, dyson_reg.reg_2_0_10_0.Reg4,
                dyson_reg.reg_2_0_10_0.Reg5);
        ESP_LOGI(DYSON_INFO_TAG,"2_0_10_reg6= %0.2f, 2_0_10_reg7= %0.2f",dyson_reg.reg_2_0_10_0.Reg6, dyson_reg.reg_2_0_10_0.Reg7);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void app_main(void)
{
    init();
    
    ESP_LOGI("APP_MAIN", "Device Started!");
    UartQueueHandle=xQueueCreate(30,sizeof(uart_packet_t*));
    ESP_LOGI("QUEUE_TAG", "Queue created!");
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(Dyson_data_process, "Dyson_data_task", 1024*5, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(Blynk_led, "Blynk_Led", 1024*2, NULL, configMAX_PRIORITIES-4, NULL);
    xTaskCreate(DysonInfo_task, "DysonInfo_task", 1024*3, NULL, configMAX_PRIORITIES-3, NULL);
    
    
    
}

void PacketReadyCallback(uart_packet_t * newPacket)
{
    uart_packet_t *flush_buf=NULL;
    static size_t totalPackNum=0;
    uint8_t q_free_space= uxQueueSpacesAvailable(UartQueueHandle);
#ifdef DYSON_DEBUG
    ESP_LOGI("P_READY_CALLBACK", "Callback fired!\nPacket recieved= %u, Queue free space= %u",(++totalPackNum),q_free_space);
#endif
    
    if(!q_free_space)
    {
        xQueueReceive(UartQueueHandle,flush_buf,0);
        free(flush_buf);

    }
    xQueueSend(UartQueueHandle,&newPacket,0);
}