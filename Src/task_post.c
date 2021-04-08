/**
  ******************************************************************************
  * Flora
  ******************************************************************************
  * @author Roman Trub
  * @file   task_post.c
  * @brief  Post task (runs after the communication round)
  *
  *
  ******************************************************************************
  */

#include "main.h"


extern QueueHandle_t xQueueHandle_tx;
extern QueueHandle_t xQueueHandle_rx;
extern TaskHandle_t xTaskHandle_com;
extern TaskHandle_t xTaskHandle_post;
extern TaskHandle_t xTaskHandle_idle;

uint32_t data_period = DATA_GENERATION_PERIOD;


/* Private define ------------------------------------------------------------*/

#ifndef POST_TASK_RESUMED
#define POST_TASK_RESUMED()
#define POST_TASK_SUSPENDED()
#endif /* POST_TASK_IND */


/* Private variables ---------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/

void vTask_post(void const * argument)
{
  static dpp_message_t msg_buffer;
  static uint32_t      last_pkt = 0;

  LOG_VERBOSE("post task started");

  /* Infinite loop */
  for(;;)
  {
    POST_TASK_SUSPENDED();
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    POST_TASK_RESUMED();

    /* process all packets rcvd from the network (regardless of whether there is space in the BOLT queue) */
    uint16_t rcvd = 0;
    while (xQueueReceive(xQueueHandle_rx, (void*)&msg_buffer, 0)) {
      //TODO process packets
      rcvd++;
    }
    if (rcvd) {
      LOG_INFO("%u msg rcvd from network", rcvd);
    }

    /* generate a node info message if necessary (must be here) */
    if (data_period) {
      /* only send other messages once the node info msg has been sent! */
      uint64_t network_time = 0;
      lwb_get_last_syncpoint(&network_time, 0);
      uint32_t div = (network_time / (1000000 * data_period));
      if (div != last_pkt) {
        /* generate a dummy packet (header only, no data) */
        if (!ps_compose_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_INVALID, 0, 0, &msg_buffer) ||
            !xQueueSend(xQueueHandle_tx, &msg_buffer, 0)) {
          LOG_WARNING("failed to insert message into transmit queue");
        }
        last_pkt = div;
      }
    }

    /* check for critical stack usage or overflow */
    rtos_check_stack_usage();

    /* print some stats */
    LOG_INFO("CPU duty cycle:  %u%%    radio duty cycle (rx/tx):  %uppm/%uppm", (uint16_t)rtos_get_cpu_dc() / 100, radio_get_rx_dc(), radio_get_tx_dc());

    /* flush the log print queue */
#if !LOG_PRINT_IMMEDIATELY
    log_flush();
#endif /* LOG_PRINT_IMMEDIATELY */

    /* before telling the state machine to enter low-power mode, wait for the UART transmission to complete (must be done here, NOT in lpm_prepare) */
#if LOG_USE_DMA
    uart_wait_tx_complete(100);     // 100ms timeout
#endif /* LOG_USE_DMA */

    /* round finished, prepare for low-power mode */
    lpm_update_opmode(OP_MODE_EVT_DONE);
  }
}
