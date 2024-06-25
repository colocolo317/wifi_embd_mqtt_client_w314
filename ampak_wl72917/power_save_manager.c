/*
 * power_save_manager.c
 *
 *  Created on: 2024/6/19
 *      Author: ch.wang
 */

/** reference tcp_tx_on_periodic_wakeup_soc\app.c **/


#include "sl_si91x_m4_ps.h"
#include "rsi_ps_config.h"
#include "sl_si91x_host_interface.h"
#include "sl_wifi.h"

#define SL_SI91X_MCU_ALARM_BASED_WAKEUP 0
#define ALARM_PERIODIC_TIME 30 /*<! periodic alarm configuration in SEC */

#define WIRELESS_WAKEUP_IRQHandler          NPSS_TO_MCU_WIRELESS_INTR_IRQn
#define WIRELESS_WAKEUP_IRQHandler_Priority 8

#define BROADCAST_DROP_THRESHOLD        5000
#define BROADCAST_IN_TIM                1
#define BROADCAST_TIM_TILL_NEXT_COMMAND 1

void ampak_m4_sleep_wakeup(void)
{
  sl_status_t status = SL_STATUS_OK;

  status = sl_wifi_filter_broadcast(BROADCAST_DROP_THRESHOLD, BROADCAST_IN_TIM, BROADCAST_TIM_TILL_NEXT_COMMAND);
  if (status != SL_STATUS_OK) {
    printf("\r\nsl_wifi_filter_broadcast Failed, Error Code : 0x%lX\r\n", status);
    return;
  }

  sl_wifi_performance_profile_t performance_profile = { .profile = ASSOCIATED_POWER_SAVE };
  // set performance profile
  status = sl_wifi_set_performance_profile(&performance_profile);
  if (status != SL_STATUS_OK) {
    printf("\r\nPower save configuration Failed, Error Code : 0x%lX\r\n", status);
    return;
  }

#ifndef SLI_SI91X_MCU_ENABLE_FLASH_BASED_EXECUTION
  /* LDOSOC Default Mode needs to be disabled */
  sl_si91x_disable_default_ldo_mode();

  /* bypass_ldorf_ctrl needs to be enabled */
  sl_si91x_enable_bypass_ldo_rf();

  sl_si91x_disable_flash_ldo();

  /* Configure RAM Usage and Retention Size */
  sl_si91x_configure_ram_retention(WISEMCU_48KB_RAM_IN_USE, WISEMCU_RETAIN_DEFAULT_RAM_DURING_SLEEP);

  /* Trigger M4 Sleep */
  sl_si91x_trigger_sleep(SLEEP_WITH_RETENTION,
                         DISABLE_LF_MODE,
                         0,
                         (uint32_t)RSI_PS_RestoreCpuContext,
                         0,
                         RSI_WAKEUP_WITH_RETENTION_WO_ULPSS_RAM);

#else

#if SL_SI91X_MCU_ALARM_BASED_WAKEUP
  /* Update the alarm time interval, when to get next interrupt  */
  set_alarm_interrupt_timer(ALARM_PERIODIC_TIME);

#endif

  /* Configure Wakeup-Source */
  RSI_PS_SetWkpSources(WIRELESS_BASED_WAKEUP);
  NVIC_SetPriority(WIRELESS_WAKEUP_IRQHandler, WIRELESS_WAKEUP_IRQHandler_Priority);

  /* Enable NVIC */
  NVIC_EnableIRQ(WIRELESS_WAKEUP_IRQHandler);

#if SL_SI91X_MCU_BUTTON_BASED_WAKEUP
  /*Configure the UULP GPIO 2 as wakeup source */
  wakeup_source_config();
#endif

  /* Configure RAM Usage and Retention Size */
  sl_si91x_configure_ram_retention(WISEMCU_192KB_RAM_IN_USE, WISEMCU_RETAIN_DEFAULT_RAM_DURING_SLEEP);

  printf("\r\nM4 Sleep\r\n");

  /* Trigger M4 Sleep*/
  sl_si91x_trigger_sleep(SLEEP_WITH_RETENTION,
                         DISABLE_LF_MODE,
                         WKP_RAM_USAGE_LOCATION,
                         (uint32_t)RSI_PS_RestoreCpuContext,
                         IVT_OFFSET_ADDR,
                         RSI_WAKEUP_FROM_FLASH_MODE);

  /* Enable M4_TA interrupt */
  sli_m4_ta_interrupt_init();

  /* Clear M4_wakeup_TA bit so that TA will go to sleep after M4 wakeup*/
  sl_si91x_host_clear_sleep_indicator();

  //  /*Start of M4 init after wake up  */
  printf("\r\nM4 Wake Up\r\n");
#endif
}
