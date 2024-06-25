/*
 * os_log_task.c
 *
 *  Created on: 2024/5/16
 *      Author: ch.wang
 */

#include "ampak_wl72917/os_log_task.h"
#include "ampak_wl72917/ampak_util.h"

static uint8_t os_log_read_buffer[OS_LOG_BUFFER_SIZE + STRING_BUFFER_END];
static uint8_t os_log_write_buffer[OS_LOG_BUFFER_SIZE];
osMessageQueueId_t os_log_msg_queue = NULL;
osThreadId_t os_log_thread_id = NULL;

const osThreadAttr_t os_log_thread_attributes =
    {
        .name       = "os_log_thread",
        .attr_bits  = 0,
        .cb_mem     = 0,
        .cb_size    = 0,
        .stack_mem  = 0,
        .stack_size = 256,
        .priority   = osPriorityNormal,
        .tz_module  = 0,
        .reserved   = 0,
    };

/**
 *  Local functions
 */

inline static osStatus_t os_log_write(const void* write_buff);

/**
 * Function implementation
 */

void os_log_init(void)
{
  os_log_msg_queue = osMessageQueueNew(OS_LOG_MSGQUEUE_SLOTS, OS_LOG_BUFFER_SIZE, NULL);
  if (os_log_msg_queue == NULL)
  {
    printf("Failed to new os log message queue\r\n"); // Message Queue object not created, handle failure
    return;
  }
  os_log_thread_id = osThreadNew((osThreadFunc_t)os_log_task, NULL, &os_log_thread_attributes);
  if (os_log_thread_id == NULL)
  {
    printf("Failed to new os log thread\r\n");
    if(osMessageQueueDelete(os_log_msg_queue) == osOK)
    { printf("Delete os log message queue\r\n"); }
    else
    { printf("Failed to delete os log message queue\r\n"); }
    return;
  }
  printf("os log thread init OK\r\n");
}

void os_log_deinit(void)
{
  osStatus_t os_status;
  os_status = osThreadSuspend(os_log_thread_id);
  osKernelState_t kernel_state = osKernelGetState();

  printf("OS kernel state: %d\r\n", kernel_state);

  if(os_status != osOK)
  {
    printf("Thread suspend error: %d\r\n", os_status);
    return;
  }

  while(1)
  {
    if(kernel_state == osKernelRunning)
    {
      os_status = osMessageQueueGet(os_log_msg_queue, os_log_read_buffer, NULL, OS_LOG_READ_DELAY);
    }
    else
    {
      os_status = osMessageQueueGet(os_log_msg_queue, os_log_read_buffer, NULL, OS_LOG_NO_DELAY);
    }
    if(os_status == osOK)
    { printf("%s",os_log_read_buffer); }
    else
    {
      printf("Message queue cleared\r\n");
      break;
    }
  }

  os_status = osMessageQueueDelete(os_log_msg_queue);
  if(os_status != osOK)
  {
    printf("Queue delete error: %d\r\n", os_status);
    return;
  }
  os_log_msg_queue = NULL;
  os_status = osThreadTerminate(os_log_thread_id);
  if(os_status != osOK)
  {
    printf("Thread terminate error: %d\r\n", os_status);
    return;
  }
  os_log_thread_id = NULL;

  printf("os log thread deinited\r\n");
}

void os_log_task(void* args)
{
  UNUSED_PARAMETER(args);
  osStatus_t os_status;

  while(1)
  {
    os_status = osMessageQueueGet(os_log_msg_queue, os_log_read_buffer, NULL, OS_LOG_READ_DELAY);
    if(os_status == osOK)
    { printf("%s",os_log_read_buffer); }
    else
    { osThreadYield(); }
  }
}

inline static osStatus_t os_log_write(const void* write_buff)
{
  osStatus_t os_status = osOK;
  os_status = osMessageQueuePut(os_log_msg_queue, write_buff, 0U, OS_LOG_WRITE_DELAY);

  if(os_status == osErrorParameter)
  { /* IS_IRQ()  time out should be 0 */
    os_status = osMessageQueuePut(os_log_msg_queue, write_buff, 0U, OS_LOG_NO_DELAY);
  }

  if(os_status != osOK)
  {
    printf("[%s]", (const char *)write_buff);
    printf("mq%d\r\n", os_status);
  }
  return os_status;
}

inline osStatus_t os_log_sprint_write(const char* format, ...)
{
  if(!os_log_ready())
  { return osError; }

  va_list args;
  va_start(args, format);
  vsprintf((char *)os_log_write_buffer, format, args);
  va_end(args);

  return os_log_write(os_log_write_buffer);
}

bool os_log_ready(void)
{
  return (os_log_msg_queue == NULL)? false : ((os_log_thread_id == NULL)? false : true);
}
