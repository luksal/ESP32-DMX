#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#define DMX_SERIAL_INPUT_PIN    16          // pin for dmx rx
#define DMX_SERIAL_OUTPUT_PIN   17          // pin for dmx tx
#define DMX_UART_NUM            UART_NUM_2  // dmx uart

#define HEALTHY_TIME            500         // timeout in ms 

#define BUF_SIZE                1024        //  buffer size for rx events

static QueueHandle_t dmx_rx_queue;          // queue for uart rx events
SemaphoreHandle_t sync_dmx = NULL;          // semaphore for syncronising access to dmx array

typedef enum
{
    IDLE = 0,
    BREAK = 1,
    DATA = 2,
} dmx_state_t;

dmx_state_t dmx_state = IDLE;               // status, in which recevied state we are
uint16_t current_rx_addr = 0;               // 

long last_dmx_packet = 0;                   // timestamp for the last received packet
uint8_t dmx_data[513];                      // stores the received dmx data

bool DMX_healthy()
{
  // get timestamp of last received packet
  xSemaphoreTake(sync_dmx, portMAX_DELAY);
  long dmx_timeout = last_dmx_packet;
  xSemaphoreGive(sync_dmx);
  // check if elapsed time < defined timeout
  if(millis() - dmx_timeout < HEALTHY_TIME)
  {
    return true;
  }
  return false;
}

static void uart_event_task(void *pvParameters)
{
  uart_event_t event;
  size_t buffered_size;
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
          if(dmx_state == BREAK)
          {
            // if not 0, then RDM or custom protocol
            if(dtmp[0] == 0)
            {
              dmx_state = DATA;
              // reset dmx adress to 0
              current_rx_addr = 0;
              xSemaphoreTake(sync_dmx, portMAX_DELAY);
              // store received timestamp
              last_dmx_packet = millis();
              xSemaphoreGive(sync_dmx);
            }
          }
          // check if in data receive mode
          if(dmx_state == DATA)
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
          // break detected, clear queue und flush received bytes
          // results in missing the last few bytes... needs fix
          uart_flush_input(DMX_UART_NUM);
          xQueueReset(dmx_rx_queue);
          dmx_state = BREAK;
          break;
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
        case UART_BUFFER_FULL:
        case UART_FIFO_OVF:
        default:
          // error recevied, going to idle mode
          uart_flush_input(DMX_UART_NUM);
          xQueueReset(dmx_rx_queue);
          dmx_state = IDLE;
          break;
      }
    }
  }
  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);
}


void DMX_init()
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

uint8_t DMX_read(int channel)
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

// -------------------
// Example
// -------------------

int readcycle = 0;

void setup()
{
  Serial.begin(115200);
  DMX_init();
}

void loop()
{
  if(millis() - readcycle > 1000)
  {
    readcycle = millis();

    Serial.print(readcycle);
      
    if(DMX_healthy())
    {
      Serial.print(": ok - ");
    }
    else
    {
      Serial.print(": fail - ");
    }
    Serial.print(DMX_read(1));
    Serial.print(" - ");
    Serial.print(DMX_read(110));
    Serial.print(" - ");
    Serial.println(DMX_read(256));
  }
}
