/* 
 * This file is part of the ESP32-DMX distribution (https://github.com/luksal/ESP32-DMX).
 * Copyright (c) 2021 Lukas Salomon.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <dmx.h>

#define DMX_SERIAL_INPUT_PIN    GPIO_NUM_16 // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN   GPIO_NUM_17 // pin for dmx tx
#define DMX_SERIAL_IO_PIN       GPIO_NUM_4  // pin for dmx rx/tx change

#define DMX_UART_NUM            UART_NUM_2  // dmx uart

#define HEALTHY_TIME            500         // timeout in ms 

#define BUF_SIZE                1024        //  buffer size for rx events

#define DMX_CORE                1           // select the core the rx/tx thread should run on

#define DMX_IGNORE_THREADSAFETY 0           // set to 1 to disable all threadsafe mechanisms

QueueHandle_t DMX::dmx_rx_queue;

SemaphoreHandle_t DMX::sync_dmx;

DMXState DMX::dmx_state = DMX_IDLE;

uint16_t DMX::current_rx_addr = 0;

long DMX::last_dmx_packet = 0;

uint8_t DMX::dmx_data[513];

DMX::DMX()
{

}

void DMX::Initialize(DMXDirection direction)
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

    // create mutex for syncronisation
    sync_dmx = xSemaphoreCreateMutex();

    // set gpio for direction
    gpio_pad_select_gpio(DMX_SERIAL_IO_PIN);
    gpio_set_direction(DMX_SERIAL_IO_PIN, GPIO_MODE_OUTPUT);

    // depending on parameter set gpio for direction change and start rx or tx thread
    if(direction == output)
    {
        gpio_set_level(DMX_SERIAL_IO_PIN, 1);
        dmx_state = DMX_OUTPUT;
        
        // create send task
        xTaskCreatePinnedToCore(DMX::uart_send_task, "uart_send_task", 1024, NULL, 1, NULL, DMX_CORE);
    }
    else
    {    
        gpio_set_level(DMX_SERIAL_IO_PIN, 0);
        dmx_state = DMX_IDLE;

        // create receive task
        xTaskCreatePinnedToCore(DMX::uart_event_task, "uart_event_task", 2048, NULL, 1, NULL, DMX_CORE);
    }
}

uint8_t DMX::Read(uint16_t channel)
{
    // restrict acces to dmx array to valid values
    if(channel < 1 || channel > 512)
    {
        return 0;
    }

    // take data threadsafe from array and return
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    uint8_t tmp_dmx = dmx_data[channel];
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
    return tmp_dmx;
}

void DMX::ReadAll(uint8_t * data, uint16_t start, size_t size)
{
    // restrict acces to dmx array to valid values
    if(start < 1 || start > 512 || start + size > 513)
    {
        return;
    }
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    memcpy(data, (uint8_t *)dmx_data + start, size);
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

void DMX::Write(uint16_t channel, uint8_t value)
{
    // restrict acces to dmx array to valid values
    if(channel < 1 || channel > 512)
    {
        return;
    }

#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    dmx_data[channel] = value;
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

void DMX::WriteAll(uint8_t * data, uint16_t start, size_t size)
{
    // restrict acces to dmx array to valid values
    if(start < 1 || start > 512 || start + size > 513)
    {
        return;
    }
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    memcpy((uint8_t *)dmx_data + start, data, size);
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
}

uint8_t DMX::IsHealthy()
{
    // get timestamp of last received packet
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
    long dmx_timeout = last_dmx_packet;
#ifndef DMX_IGNORE_THREADSAFETY
    xSemaphoreGive(sync_dmx);
#endif
    // check if elapsed time < defined timeout
    if(xTaskGetTickCount() - dmx_timeout < HEALTHY_TIME)
    {
        return 1;
    }
    return 0;
}

void DMX::uart_send_task(void*pvParameters)
{
    uint8_t start_code = 0x00;
    for(;;)
    {
        // wait till uart is ready
        uart_wait_tx_done(DMX_UART_NUM, 1000);
        // set line to inverse, creates break signal
        uart_set_line_inverse(DMX_UART_NUM, UART_INVERSE_TXD);
        // wait break time
        ets_delay_us(184);
        // disable break signal
        uart_set_line_inverse(DMX_UART_NUM,  0);
        // wait mark after break
        ets_delay_us(24);
        // write start code
        uart_write_bytes(DMX_UART_NUM, (const char*) &start_code, 1);
#ifndef DMX_IGNORE_THREADSAFETY
        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
        // transmit the dmx data
        uart_write_bytes(DMX_UART_NUM, (const char*) dmx_data+1, 512);
#ifndef DMX_IGNORE_THREADSAFETY
        xSemaphoreGive(sync_dmx);
#endif
    }
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
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
                        // store received timestamp
                        last_dmx_packet = xTaskGetTickCount();
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreGive(sync_dmx);
#endif
                        }
                    }
                    // check if in data receive mode
                    if(dmx_state == DMX_DATA)
                    {
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreTake(sync_dmx, portMAX_DELAY);
#endif
                        // copy received bytes to dmx data array
                        for(int i = 0; i < event.size; i++)
                        {
                            if(current_rx_addr < 513)
                            {
                                dmx_data[current_rx_addr++] = dtmp[i];
                            }
                        }
#ifndef DMX_IGNORE_THREADSAFETY
                        xSemaphoreGive(sync_dmx);
#endif
                    }
                    break;
                case UART_BREAK:
                    // break detected
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
