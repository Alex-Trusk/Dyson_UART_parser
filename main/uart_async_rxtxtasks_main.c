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

static const int RX_BUF_SIZE = 1024;
QueueHandle_t UartQueueHandle;
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
#define CONFIG_BLINK_GPIO GPIO_NUM_2

Dyson_regs dyson_reg={
    .Flow_reg.Flow1=0, .Flow_reg.Flow2=0,.Flow_reg.Flow3=0,.Flow_reg.Flow4=0,
    .reg_2_0_10_0.Reg1=0.0f, .reg_2_0_10_0.Reg2=0.0f,.reg_2_0_10_0.Reg3=0.0f,.reg_2_0_10_0.Reg4=0.0f,.reg_2_0_10_0.Reg5=0.0f,
    .reg_2_0_10_0.Reg6=0.0f,.reg_2_0_10_0.Reg7=0.0f,
    .reg_2_0_12_0.Reg1=0,.reg_2_0_12_0.Reg2=0,.reg_2_0_12_0.Reg3=0,.reg_2_0_12_0.Reg4=0,
    .reg_2_0_2_0.Reg1=0,.reg_2_0_2_0.Reg2=0,.reg_2_0_2_0.Reg3=0,.reg_2_0_2_0.Reg4=0,
    .reg_2_0_3_0.Reg1=0.0f,.reg_2_0_3_0.Reg2=0.0f,.reg_2_0_3_0.Reg3=0.0f,.reg_2_0_3_0.Reg4=0.0f,.reg_2_0_3_0.Reg5=0.0f,
    .reg_2_0_5_0.Reg1=0,.reg_2_0_5_0.Reg2=0,.reg_2_0_5_0.Reg3=0,
    .reg_2_0_6_0.Reg1=0,.reg_2_0_6_0.Reg2=0,
    .reg_2_0_7_0.Reg1=0.0f,.reg_2_0_7_0.Reg2=0.0f,.reg_2_0_7_0.Reg3=0.0f,.reg_2_0_7_0.Reg4=0.0f,.reg_2_0_7_0.Reg5=0.0f,
    .reg_2_0_7_0.Reg6=0.0f,.reg_2_0_7_0.Reg7=0.0f,.reg_2_0_7_0.Reg8=0.0f,.reg_2_0_7_0.Reg9=0.0f,.reg_2_0_7_0.Reg10=0.0f,
    .reg_2_0_7_0.Reg11=0.0f,.reg_2_0_7_0.Reg12=0.0f,
    .reg_2_0_8_0.Reg1=0,.reg_2_0_8_0.Reg2=0,.reg_2_0_8_0.Reg3=0,.reg_2_0_8_0.Reg4=0,.reg_2_0_8_0.Reg5=0,.reg_2_0_8_0.Reg6=0,
    .reg_2_0_8_0.Reg7=0,.reg_2_0_8_0.Reg8=0,.reg_2_0_8_0.Reg9=0,.reg_2_0_8_0.Reg10=0,.reg_2_0_8_0.Reg11=0,.reg_2_0_8_0.Reg12=0,
    .reg_2_0_9_0.Reg1=0.0f,.reg_2_0_9_0.Reg2=0.0f,.reg_2_0_9_0.Reg3=0.0f,.reg_2_0_9_0.Reg4=0.0f,.reg_2_0_9_0.Reg5=0.0f,
    .reg_3_0_10_0.Reg1=0.0f,.reg_3_0_10_0.Reg2=0.0f,.reg_3_0_10_0.Reg3=0.0f,.reg_3_0_10_0.Reg4=0.0f,.reg_3_0_10_0.Reg5=0.0f,
    .reg_3_0_1_0.Reg1=0,.reg_3_0_1_0.Reg2=0,
    .reg_3_0_5_0.Reg1=0.0f,.reg_3_0_5_0.Reg2=0.0f,.reg_3_0_5_0.Reg3=0.0f,.reg_3_0_5_0.Reg4=0.0f,
    .reg_3_0_6_0.Reg1=0.0f,.reg_3_0_6_0.Reg2=0.0f,.reg_3_0_6_0.Reg3=0.0f,.reg_3_0_6_0.Reg4=0.0f,.reg_3_0_6_0.Reg5=0.0f,
    .reg_3_0_7_0.Reg1=0.0f,.reg_3_0_7_0.Reg2=0.0f,.reg_3_0_7_0.Reg3=0.0f,.reg_3_0_7_0.Reg4=0.0f,.reg_3_0_7_0.Reg5=0.0f,
    .reg_3_0_8_0.Reg1=0.0f,.reg_3_0_8_0.Reg2=0.0f,.reg_3_0_8_0.Reg3=0.0f,.reg_3_0_8_0.Reg4=0.0f,
    .reg_3_0_9_0.Reg1=0.0f,.reg_3_0_9_0.Reg2=0.0f,.reg_3_0_9_0.Reg3=0.0f,.reg_3_0_9_0.Reg4=0.0f,.reg_3_0_9_0.Reg5=0.0f,
    .reg_3_0_9_0.Reg6=0.0f,
    .Temp_reg.Temp1=0.0f,.Temp_reg.Temp2=0.0f,.Temp_reg.Temp3=0.0f,.Temp_reg.Temp4=0.0f,.Temp_reg.Temp5=0.0f,
    .UpTime_reg=0
};


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
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        // SendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            ParseUartStream(data,rxBytes);
        }
    }
    free(data);
}
static void Dyson_data_process(void *arg)
{
    static const char *DYSON_TASK_TAG = "DYSON_TASK";
    uart_packet_t *packet_buf=NULL, *unstuffed_pack;

    esp_log_level_set(DYSON_TASK_TAG, ESP_LOG_INFO);
    while (1) 
    {

        if(xQueueReceive(UartQueueHandle,packet_buf,0xFFFFFFFF))
        {
            unstuffed_pack=UnstuffPacket(packet_buf);
            free(packet_buf);
            ParseDysonPacket(unstuffed_pack,&dyson_reg);
        
            
        }
       // vTaskDelay(2000 / portTICK_PERIOD_MS);
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
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
}
void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(Dyson_data_process, "Dyson_data_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(Blynk_led, "Blynk_Led", 256, NULL, configMAX_PRIORITIES-3, NULL);
    UartQueueHandle=xQueueCreate(30,sizeof(uart_packet_t*));
    
}

void PacketReadyCallback(uart_packet_t * newPacket)
{
    uart_packet_t *flush_buf=NULL;
    if(!uxQueueSpacesAvailable(UartQueueHandle))
    {
        xQueueReceive(UartQueueHandle,flush_buf,0);
        free(flush_buf);

    }
    xQueueSend(UartQueueHandle,newPacket,0);
}