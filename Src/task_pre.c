/*
 * task_pre.c
 *
 * generates data packets
 */

#include "main.h"


extern QueueHandle_t xQueueHandle_tx;

uint32_t data_period = DATA_GENERATION_PERIOD;


/* Private define ------------------------------------------------------------*/

#ifndef PRE_TASK_RESUMED
#define PRE_TASK_RESUMED()
#define PRE_TASK_SUSPENDED()
#endif /* PRE_TASK_RESUMED */


/* Private variables ---------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/

void generate_data_pkt(void)
{
  static dpp_message_t msg_buffer;

  /* generate a dummy packet (header only, no data) */
  if (ps_compose_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_INVALID, 0, 0, &msg_buffer) &&
      xQueueSend(xQueueHandle_tx, &msg_buffer, 0)) {
    LOG_INFO("data packet generated");
  } else {
    LOG_WARNING("failed to insert message into transmit queue");
  }
}


void vTask_pre(void const * argument)
{
  static uint32_t      last_pkt = 0;

  LOG_VERBOSE("pre task started");

  generate_data_pkt();

  /* Infinite loop */
  for(;;)
  {
    PRE_TASK_SUSPENDED();
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    PRE_TASK_RESUMED();

    /* generate a node info message if necessary (must be here) */
    if (data_period) {
      /* only send other messages once the node info msg has been sent! */
      uint64_t network_time = 0;
      lwb_get_last_syncpoint(&network_time, 0);
      uint32_t div = (network_time / (1000000 * data_period));
      if (div != last_pkt) {
        /* generate a dummy packet (header only, no data) */
        generate_data_pkt();
        last_pkt = div;
      }
    }
  }
}
