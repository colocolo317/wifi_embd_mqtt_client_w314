/*
 * os_log_task.h
 *
 *  Created on: 2024/5/16
 *      Author: ch.wang
 */

#ifndef AMPAK_WL72917_OS_LOG_TASK_H_
#define AMPAK_WL72917_OS_LOG_TASK_H_


#include "cmsis_os2.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"

#define USE_OS_LOG 1

#define OS_LOG_MSGQUEUE_SLOTS 50U
#define OS_LOG_BUFFER_SIZE 200U
#define OS_LOG_READ_DELAY 100U
#define STRING_BUFFER_END 1U

#define OS_LOG_WRITE_DELAY 300U
#define OS_LOG_NO_DELAY 0U

typedef enum {
  os_log_ok = 0,
  os_log_faild = 1,
}osLogStatus_t;

extern osThreadId_t os_log_thread_id;

#ifndef os_log
#if USE_OS_LOG
#define os_log(...) os_log_sprint_write(__VA_ARGS__)
#else
#define os_log(...)
#endif
#endif

void os_log_init(void);
void os_log_deinit(void);
void os_log_task(void* args);
osStatus_t os_log_sprint_write(const char* format, ...);
bool os_log_ready(void);

#endif /* AMPAK_WL72917_OS_LOG_TASK_H_ */
