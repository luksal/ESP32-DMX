#include <dmx.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#define DMX_SERIAL_INPUT_PIN    16          // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN   17          // pin for dmx tx
#define DMX_UART_NUM            UART_NUM_2  // dmx uart

#define HEALTHY_TIME            500         // timeout in ms 

#define BUF_SIZE                1024        //  buffer size for rx events

#define DMX_IDLE                    0
#define DMX_BREAK                   1
#define DMX_DATA                    2

QueueHandle_t DMX::dmx_rx_queue;

SemaphoreHandle_t DMX::sync_dmx;

uint8_t DMX::dmx_state = DMX_IDLE;

uint16_t DMX::current_rx_addr = 0;

long DMX::last_dmx_packet = 0;

uint8_t DMX::dmx_data[513];

DMX::DMX()
{

}

void DMX::Initialize()
{
    // configure UART for DMX
    uart_config_t uart_config =
    {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(DMX_UART_NUM, &uart_config);

    // Set pins for UART
    uart_set_pin(DMX_UART_NUM, DMX_SERIAL_OUTPUT_PIN, DMX_SERIAL_INPUT_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // install queue
    uart_driver_install(DMX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &dmx_rx_queue, 0);

    // create receive task
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    // create mutex for syncronisation
    sync_dmx = xSemaphoreCreateMutex();
}

uint8_t DMX::Read(uint16_t channel)
{
    // restrict acces to dmx array to valid values
    if(channel < 1)
    {
        channel = 1;
    }
    else if(channel > 512)
    {
        channel = 512;
    }
    // take data threadsafe from array and return
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
    uint8_t tmp_dmx = dmx_data[channel];
    xSemaphoreGive(sync_dmx);
    return tmp_dmx;
}

uint8_t DMX::IsHealthy()
{
    // get timestamp of last received packet
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
    long dmx_timeout = last_dmx_packet;
    xSemaphoreGive(sync_dmx);
    // check if elapsed time < defined timeout
    if(xTaskGetTickCount() - dmx_timeout < HEALTHY_TIME)
    {
        return 1;
    }
    return 0;
}

void DMX::uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    for(;;)
    {
        // wait for data in the dmx_queue
        if(xQueueReceive(dmx_rx_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, BUF_SIZE);
            switch(event.type)
            {
                case UART_DATA:
                    // read the received data
                    uart_read_bytes(DMX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    // check if break detected
                    if(dmx_state == DMX_BREAK)
                    {
                        // if not 0, then RDM or custom protocol
                        if(dtmp[0] == 0)
                        {
                        dmx_state = DMX_DATA;
                        // reset dmx adress to 0
                        current_rx_addr = 0;
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
                        // store received timestamp
                        last_dmx_packet = xTaskGetTickCount();
                        xSemaphoreGive(sync_dmx);
                        }
                    }
                    // check if in data receive mode
                    if(dmx_state == DMX_DATA)
                    {
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
                        // copy received bytes to dmx data array
                        for(int i = 0; i < event.size; i++)
                        {
                        dmx_data[current_rx_addr++] = dtmp[i];
                        }
                        xSemaphoreGive(sync_dmx);
                    }
                    break;
                case UART_BREAK:
                    // break detected
                    // check if there are bytes left in the queue
                    if(dmx_state == DMX_DATA && event.size > 0 && current_rx_addr > 0)
                    {
                        uart_read_bytes(DMX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
                        // copy received bytes to dmx data array
                        for(int i = 0; i < event.size; i++)
                        {
                            dmx_data[current_rx_addr++] = dtmp[i];
                        }
                        xSemaphoreGive(sync_dmx);
                    }
                    // clear queue und flush received bytes                    
                    uart_flush_input(DMX_UART_NUM);
                    xQueueReset(dmx_rx_queue);
                    dmx_state = DMX_BREAK;
                    break;
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                case UART_BUFFER_FULL:
                case UART_FIFO_OVF:
                default:
                    // error recevied, going to idle mode
                    uart_flush_input(DMX_UART_NUM);
                    xQueueReset(dmx_rx_queue);
                    dmx_state = DMX_IDLE;
                    break;
            }
        }
    }
}
