/***************************************************************************/ /**
 * @file
 * @brief Access point Example Application
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdio.h>
#include "sl_net.h"
#include "sl_utility.h"
#include "cmsis_os2.h"
#include "sl_constants.h"
#include "sl_mqtt_client.h"
#include "sl_net_wifi_types.h"
#include "cacert.pem.h"
#include "sl_wifi.h"
#include "string.h"
#include "app.h"
#include "ampak_wl72917/ampak_util.h"
#include "ampak_wl72917/ble_config.h"
/******************************************************
 *                    Constants
 ******************************************************/
#define AMPAK_FW_VERSION   "v001.01.242701"
#define VERSION_GOAL        "MQTT long time test setting, get disconnect issue"
#define MODIFY_START_DATE   "2024.07.02"
#define AMPAK_USE_BLE 1
#define AMPAK_USE_SLEEP 0


#define MQTT_BROKER_IP   "10.10.28.233"
#define MQTT_BROKER_PORT 1883

#define CLIENT_PORT 1

#define CLIENT_ID "Ampak917"

#define TOPIC_TO_BE_SUBSCRIBED "Ampak/917/command"
#define QOS_OF_SUBSCRIPTION    SL_MQTT_QOS_LEVEL_1

#define PUBLISH_TOPIC          "Ampak/917/report"
#define PUBLISH_MESSAGE        "I am alive."
#define QOS_OF_PUBLISH_MESSAGE SL_MQTT_QOS_LEVEL_1

#define IS_DUPLICATE_MESSAGE 0
#define IS_MESSAGE_RETAINED  1
#define IS_CLEAN_SESSION     1

#define LAST_WILL_TOPIC       "Ampak/917/dismiss"
#define LAST_WILL_MESSAGE     "disconnect"
#define QOS_OF_LAST_WILL      SL_MQTT_QOS_LEVEL_1
#define IS_LAST_WILL_RETAINED 1

#define ENCRYPT_CONNECTION     0
#define KEEP_ALIVE_INTERVAL    2000
#define MQTT_CONNECT_TIMEOUT   5000
#define MQTT_KEEPALIVE_RETRIES 20

#define SEND_CREDENTIALS 1

#define USERNAME "mqttusr"
#define PASSWORD "88888888"

/******************************************************
 *               Variable Definitions
 ******************************************************/

const osThreadAttr_t mqtt_thread_attributes = {
  .name       = "mqtt_app",
  .attr_bits  = 0,
  .cb_mem     = 0,
  .cb_size    = 0,
  .stack_mem  = 0,
  .stack_size = 3072,
  .priority   = osPriorityLow,
  .tz_module  = 0,
  .reserved   = 0,
};

#if AMPAK_USE_DEFAULT_DEVICE_CONFIG
static const sl_wifi_device_configuration_t wifi_mqtt_client_configuration =
{
  .boot_option = LOAD_NWP_FW,
  .mac_address = NULL,
  .band        = SL_SI91X_WIFI_BAND_2_4GHZ,
  .boot_config = { .oper_mode                  = SL_SI91X_CLIENT_MODE,
                   .coex_mode                  = SL_SI91X_WLAN_ONLY_MODE,
                   .feature_bit_map            = (SL_SI91X_FEAT_SECURITY_PSK | SL_SI91X_FEAT_AGGREGATION),
                   .tcp_ip_feature_bit_map     = (SL_SI91X_TCP_IP_FEAT_DHCPV4_CLIENT | SL_SI91X_TCP_IP_FEAT_DNS_CLIENT
                                              | SL_SI91X_TCP_IP_FEAT_SSL | SL_SI91X_TCP_IP_FEAT_EXTENSION_VALID),
                   .custom_feature_bit_map     = (SL_SI91X_CUSTOM_FEAT_EXTENTION_VALID),
                   .ext_custom_feature_bit_map = (SL_SI91X_EXT_FEAT_SSL_VERSIONS_SUPPORT | SL_SI91X_EXT_FEAT_XTAL_CLK
                                                  | SL_SI91X_EXT_FEAT_UART_SEL_FOR_DEBUG_PRINTS | MEMORY_CONFIG
#ifdef SLI_SI917
                                                  | SL_SI91X_EXT_FEAT_FRONT_END_SWITCH_PINS_ULP_GPIO_4_5_0
#endif
                                                  ),
                   .bt_feature_bit_map = 0,
                   .ext_tcp_ip_feature_bit_map =
                     (SL_SI91X_EXT_TCP_IP_WINDOW_SCALING | SL_SI91X_EXT_TCP_IP_TOTAL_SELECTS(10)
                      | SL_SI91X_EXT_TCP_IP_FEAT_SSL_THREE_SOCKETS | SL_SI91X_EXT_TCP_IP_FEAT_SSL_MEMORY_CLOUD
                      | SL_SI91X_EXT_EMB_MQTT_ENABLE),
                   .ble_feature_bit_map     = 0,
                   .ble_ext_feature_bit_map = 0,
                   .config_feature_bit_map  = 0 }
};
#else
static const sl_wifi_device_configuration_t wifi_mqtt_client_configuration =
{
    .boot_option = LOAD_NWP_FW,
    .mac_address = NULL,
    .band        = SL_SI91X_WIFI_BAND_2_4GHZ,
    .region_code = US,
    .boot_config =
    {
      .oper_mode       = SL_SI91X_CLIENT_MODE,
#if AMPAK_USE_BLE
      .coex_mode       = SL_SI91X_WLAN_BLE_MODE,
#else
      .coex_mode       = SL_SI91X_WLAN_ONLY_MODE,
#endif
      .feature_bit_map =
          ( SL_SI91X_FEAT_SECURITY_PSK
          | SL_SI91X_FEAT_AGGREGATION
          | SL_SI91X_FEAT_ULP_GPIO_BASED_HANDSHAKE
          | SL_SI91X_FEAT_DEV_TO_HOST_ULP_GPIO_1
          | SL_SI91X_FEAT_SECURITY_OPEN
#ifdef SLI_SI91X_MCU_INTERFACE
          | SL_SI91X_FEAT_WPS_DISABLE
#endif
          ),
      .tcp_ip_feature_bit_map =
        ( SL_SI91X_TCP_IP_FEAT_DHCPV4_CLIENT
        | SL_SI91X_TCP_IP_FEAT_DNS_CLIENT
        | SL_SI91X_TCP_IP_FEAT_HTTP_CLIENT
        | SL_SI91X_TCP_IP_FEAT_SSL
        | SL_SI91X_TCP_IP_FEAT_EXTENSION_VALID
        ),
      .custom_feature_bit_map =
        ( SL_SI91X_CUSTOM_FEAT_EXTENTION_VALID
        ),
      .ext_custom_feature_bit_map =
        ( SL_SI91X_EXT_FEAT_LOW_POWER_MODE
        | SL_SI91X_EXT_FEAT_XTAL_CLK
        | MEMORY_CONFIG
        | SL_SI91X_EXT_FEAT_SSL_VERSIONS_SUPPORT
        | SL_SI91X_EXT_FEAT_UART_SEL_FOR_DEBUG_PRINTS
#ifdef SLI_SI917
        | SL_SI91X_EXT_FEAT_FRONT_END_SWITCH_PINS_ULP_GPIO_4_5_0
#endif // SLI_SI917
        | SL_SI91X_EXT_FEAT_BT_CUSTOM_FEAT_ENABLE
        ),
      .ext_tcp_ip_feature_bit_map =
        ( SL_SI91X_CONFIG_FEAT_EXTENTION_VALID
        | SL_SI91X_EXT_TCP_IP_WINDOW_SCALING
        | SL_SI91X_EXT_TCP_IP_TOTAL_SELECTS(10)
        | SL_SI91X_EXT_TCP_IP_FEAT_SSL_THREE_SOCKETS
        | SL_SI91X_EXT_TCP_IP_FEAT_SSL_MEMORY_CLOUD
        | SL_SI91X_EXT_EMB_MQTT_ENABLE
        ),
      .bt_feature_bit_map =
        (SL_SI91X_BT_RF_TYPE
        | SL_SI91X_ENABLE_BLE_PROTOCOL
        ),
      //!ENABLE_BLE_PROTOCOL in bt_feature_bit_map
      .ble_feature_bit_map =
        (
          (SL_SI91X_BLE_MAX_NBR_PERIPHERALS(RSI_BLE_MAX_NBR_PERIPHERALS)
          | SL_SI91X_BLE_MAX_NBR_CENTRALS(RSI_BLE_MAX_NBR_CENTRALS)
          | SL_SI91X_BLE_MAX_NBR_ATT_SERV(RSI_BLE_MAX_NBR_ATT_SERV)
          | SL_SI91X_BLE_MAX_NBR_ATT_REC(RSI_BLE_MAX_NBR_ATT_REC))
          | SL_SI91X_FEAT_BLE_CUSTOM_FEAT_EXTENTION_VALID
          | SL_SI91X_BLE_PWR_INX(RSI_BLE_PWR_INX
          )
          | SL_SI91X_BLE_PWR_SAVE_OPTIONS(RSI_BLE_PWR_SAVE_OPTIONS)
          | SL_SI91X_916_BLE_COMPATIBLE_FEAT_ENABLE
#if RSI_BLE_GATT_ASYNC_ENABLE
          | SL_SI91X_BLE_GATT_ASYNC_ENABLE
#endif
        ),
      .ble_ext_feature_bit_map =
        (
          (SL_SI91X_BLE_NUM_CONN_EVENTS(RSI_BLE_NUM_CONN_EVENTS)
          | SL_SI91X_BLE_NUM_REC_BYTES(RSI_BLE_NUM_REC_BYTES)
        )
#if RSI_BLE_INDICATE_CONFIRMATION_FROM_HOST
        | SL_SI91X_BLE_INDICATE_CONFIRMATION_FROM_HOST //indication response from app
#endif
#if RSI_BLE_MTU_EXCHANGE_FROM_HOST
        | SL_SI91X_BLE_MTU_EXCHANGE_FROM_HOST //MTU Exchange request initiation from app
#endif
#if RSI_BLE_SET_SCAN_RESP_DATA_FROM_HOST
        | (SL_SI91X_BLE_SET_SCAN_RESP_DATA_FROM_HOST) //Set SCAN Resp Data from app
#endif
#if RSI_BLE_DISABLE_CODED_PHY_FROM_HOST
        | (SL_SI91X_BLE_DISABLE_CODED_PHY_FROM_HOST) //Disable Coded PHY from app
#endif
#if BLE_SIMPLE_GATT
        | SL_SI91X_BLE_GATT_INIT
#endif

       ),
     .config_feature_bit_map =
       (SL_SI91X_FEAT_SLEEP_GPIO_SEL_BITMAP
       )
   }
};
#endif

sl_mqtt_client_t client = { 0 };

uint8_t is_execution_completed = 0;

sl_mqtt_client_credentials_t *client_credentails = NULL;

char mac_for_id[13] = {0};

sl_mqtt_client_configuration_t mqtt_client_configuration = { .is_clean_session = IS_CLEAN_SESSION,
                                                             .client_id        = (uint8_t *)CLIENT_ID,
                                                             .client_id_length = strlen(CLIENT_ID),
                                                             .client_port      = CLIENT_PORT };

sl_mqtt_broker_t mqtt_broker_configuration = {
  .port                    = MQTT_BROKER_PORT,
  .is_connection_encrypted = ENCRYPT_CONNECTION,
  .connect_timeout         = MQTT_CONNECT_TIMEOUT,
  .keep_alive_interval     = KEEP_ALIVE_INTERVAL,
  .keep_alive_retries      = MQTT_KEEPALIVE_RETRIES,
};

sl_mqtt_client_message_t message_to_be_published = {
  .qos_level            = QOS_OF_PUBLISH_MESSAGE,
  .is_retained          = IS_MESSAGE_RETAINED,
  .is_duplicate_message = IS_DUPLICATE_MESSAGE,
  .topic                = (uint8_t *)PUBLISH_TOPIC,
  .topic_length         = strlen(PUBLISH_TOPIC),
  .content              = (uint8_t *)PUBLISH_MESSAGE,
  .content_length       = strlen(PUBLISH_MESSAGE),
};

sl_mqtt_client_last_will_message_t last_will_message = {
  .is_retained         = IS_LAST_WILL_RETAINED,
  .will_qos_level      = QOS_OF_LAST_WILL,
  .will_topic          = (uint8_t *)LAST_WILL_TOPIC,
  .will_topic_length   = strlen(LAST_WILL_TOPIC),
  .will_message        = (uint8_t *)LAST_WILL_MESSAGE,
  .will_message_length = strlen(LAST_WILL_MESSAGE),
};

/******************************************************
 *               Function Declarations
 ******************************************************/
void mqtt_client_message_handler(void *client, sl_mqtt_client_message_t *message, void *context);
void mqtt_client_event_handler(void *client, sl_mqtt_client_event_t event, void *event_data, void *context);
void mqtt_client_error_event_handler(void *client, sl_mqtt_client_error_status_t *error);
void mqtt_client_cleanup();
void print_char_buffer(char *buffer, uint32_t buffer_length);
sl_status_t mqtt_client_setup();
sl_status_t mqtt_net_up(void);


osSemaphoreId_t mqtt_sem;


/******************************************************
 *               Function Definitions
 ******************************************************/
void mqtt_publish_message_api(char* message)
{
  if(client.state == SL_MQTT_CLIENT_DISCONNECTED)
  {
    printf("MQTT not connected yet.\r\n");
    return;
  }
  sl_status_t status;

  static char message_append_mac[200] = {0};
  memset(message_append_mac, 0, 200);

  sprintf(message_append_mac, "%s : %s",message, mac_for_id);
  message_to_be_published.content = (uint8_t *) message_append_mac;
  message_to_be_published.content_length = strlen(message_append_mac);
  status = sl_mqtt_client_publish(&client, &message_to_be_published, 0, &message_to_be_published);
  if (status != SL_STATUS_IN_PROGRESS)
  {
    printf("Failed to publish message: 0x%lx\r\n", status);
#if AMPAK_USE_FUNC_MQTT_CLIENT_CLEANUP
    mqtt_client_cleanup();
#endif
    return;
  }
}


sl_status_t mqtt_net_up(void)
{
  sl_status_t status;

  status = sl_net_init(SL_NET_WIFI_CLIENT_INTERFACE, &wifi_mqtt_client_configuration, NULL, NULL);
  if (status != SL_STATUS_OK && status != SL_STATUS_ALREADY_INITIALIZED) {
    printf("Failed to start Wi-Fi client interface: 0x%lx\r\n", status);
    return status;
  }
  printf("\r\nWi-Fi client interface up Success\r\n");

  status = sl_net_up(SL_NET_WIFI_CLIENT_INTERFACE, SL_NET_DEFAULT_WIFI_CLIENT_PROFILE_ID);
  if (status != SL_STATUS_OK) {
    printf("Failed to bring Wi-Fi client interface up: 0x%lx\r\n", status);
    return status;
  }
  printf("Wi-Fi client connected\r\n");

  //! print out IP addr
  sl_net_wifi_client_profile_t get_profile;
  status = sl_net_get_profile(SL_NET_WIFI_CLIENT_INTERFACE, SL_NET_DEFAULT_WIFI_CLIENT_PROFILE_ID, &get_profile);
  sl_ip_address_t ip;
  ip.type = get_profile.ip.type;
  ip.ip.v4 = get_profile.ip.ip.v4.ip_address;
  print_sl_ip_address(&ip);
  printf("\r\n");

  return status;
}

void mqtt_init(void)
{
  mqtt_sem = osSemaphoreNew(1,0,NULL);
  if (mqtt_sem == NULL){
      printf("Fail to new sem\r\n");
  }
  osThreadNew((osThreadFunc_t)mqtt_task, NULL, &mqtt_thread_attributes);


}

void mqtt_task(void *argument)
{
  UNUSED_PARAMETER(argument);
#if AMPAK_USE_MQTT_NET_UP
  mqtt_net_up();
#endif
  mqtt_client_setup();
  //sl_status_t status;
  while (1) {
      //printf("Come in task\r\n");
      //osSemaphoreAcquire(mqtt_sem, 5000);
      //printf(".");

  }
}

void mqtt_client_cleanup()
{
  SL_CLEANUP_MALLOC(client_credentails);
  is_execution_completed = 1;
}

void mqtt_client_message_handler(void *client, sl_mqtt_client_message_t *message, void *context)
{
  sl_status_t status;
  UNUSED_PARAMETER(context);
  UNUSED_PARAMETER(client);
  UNUSED_PARAMETER(status);

  printf("Message Received on Topic: ");
  print_char_buffer((char *)message->topic, message->topic_length);
  printf(", Content: ");
  print_char_buffer((char *)message->content , message->content_length);
  printf("\r\n");

  static char report[200] = {0};
  memset(report, 0, 200);

  if(strcmp((char *)message->content, "http_get") == 0)
  {
    sprintf(report, "Action: %s", (char *)message->content);
    mqtt_publish_message_api(report);
  }
  else
  {
    strcpy(report, "Ack: ");
    strncpy(report+strlen("Ack: "), (char *)message->content, message->content_length);
    mqtt_publish_message_api(report);
  }
}

void print_char_buffer(char *buffer, uint32_t buffer_length)
{
  for (uint32_t index = 0; index < buffer_length; index++) {
    printf("%c", buffer[index]);
  }
}

void mqtt_client_error_event_handler(void *client, sl_mqtt_client_error_status_t *error)
{
  UNUSED_PARAMETER(client);
  printf("Terminating program, Error: %d\r\n", *error);
#if AMPAK_USE_FUNC_MQTT_CLIENT_CLEANUP
  mqtt_client_cleanup();
#endif
}

void mqtt_client_event_handler(void *client, sl_mqtt_client_event_t event, void *event_data, void *context)
{
  switch (event) {
    case SL_MQTT_CLIENT_CONNECTED_EVENT: {
      printf("SL_MQTT_CLIENT_CONNECTED_EVENT\r\n");
      sl_status_t status;

      status = sl_mqtt_client_subscribe(client,
                                        (uint8_t *)TOPIC_TO_BE_SUBSCRIBED,
                                        strlen(TOPIC_TO_BE_SUBSCRIBED),
                                        QOS_OF_SUBSCRIPTION,
                                        0,
                                        mqtt_client_message_handler,
                                        TOPIC_TO_BE_SUBSCRIBED);
      if (status != SL_STATUS_IN_PROGRESS) {
        printf("Failed to subscribe : 0x%lx\r\n", status);

        mqtt_client_cleanup();
        return;
      }
#if 1
      mqtt_publish_message_api("MQTT connect ok");
#endif
      break;
    }

    case SL_MQTT_CLIENT_MESSAGE_PUBLISHED_EVENT: {
      printf("SL_MQTT_CLIENT_MESSAGE_PUBLISHED_EVENT\r\n");
      sl_mqtt_client_message_t *published_message = (sl_mqtt_client_message_t *)context;

      printf("Published message successfully on topic: ");
      print_char_buffer((char *)published_message->topic, published_message->topic_length);
      printf("\r\n");
      break;
    }

    case SL_MQTT_CLIENT_SUBSCRIBED_EVENT: {

      char *subscribed_topic = (char *)context;

      printf("Subscribed to Topic: %s\r\n", subscribed_topic);
      break;
    }

    case SL_MQTT_CLIENT_UNSUBSCRIBED_EVENT: {
      char *unsubscribed_topic = (char *)context;

      printf("Unsubscribed from topic: %s\r\n", unsubscribed_topic);
#if AMPAK_MQTT_DISCONNECT_ON_UNSUBSCRIBE
      sl_mqtt_client_disconnect(client, 0);
#endif
      break;
    }

    case SL_MQTT_CLIENT_DISCONNECTED_EVENT: {
      printf("Disconnected from MQTT broker\r\n");
#if AMPAK_USE_FUNC_MQTT_CLIENT_CLEANUP
      mqtt_client_cleanup();
#endif
      break;
    }

    case SL_MQTT_CLIENT_ERROR_EVENT: {
      mqtt_client_error_event_handler(client, (sl_mqtt_client_error_status_t *)event_data);
      break;
    }
    default:
      break;
  }
}

sl_status_t mqtt_client_setup()
{
  sl_status_t status;

  /** get mac addr for ID **/

  sl_mac_address_t get_mac;
  status = sl_wifi_get_mac_address(SL_WIFI_CLIENT_2_4GHZ_INTERFACE, &get_mac);
  if (status != SL_STATUS_OK)
  {
    printf("Failed to get ap MAC: 0x%lx\r\n", status);
    return status;
  }

  memset(mac_for_id, 0, 13);
  char mac_str[13] = {0};
  sprintf(mac_str, "%2x%2x%2x%2x%2x%2x"
          ,get_mac.octet[0]
          ,get_mac.octet[1]
          ,get_mac.octet[2]
          ,get_mac.octet[3]
          ,get_mac.octet[4]
          ,get_mac.octet[5]
         );
  strncpy(mac_for_id, mac_str, 12);

  printf("MAC %s\r\n",mac_for_id);

  mqtt_client_configuration.client_id = (uint8_t*) mac_for_id;
  mqtt_client_configuration.client_id_length = strlen(mac_for_id);

  char will_topic_append_mac[200];
  sprintf(will_topic_append_mac,"%s/%s",last_will_message.will_topic, mac_for_id);
  last_will_message.will_topic = (uint8_t*)will_topic_append_mac;
  last_will_message.will_topic_length = strlen(will_topic_append_mac);

  if (ENCRYPT_CONNECTION) {
    // Load SSL CA certificate
    status =
      sl_net_set_credential(SL_NET_TLS_SERVER_CREDENTIAL_ID(0), SL_NET_SIGNING_CERTIFICATE, cacert, sizeof(cacert) - 1);
    if (status != SL_STATUS_OK) {
      printf("\r\nLoading TLS CA certificate in to FLASH Failed, Error Code : 0x%lX\r\n", status);
      return status;
    }
    printf("\r\nLoad TLS CA certificate at index %d Success\r\n", 0);
  }

  if (SEND_CREDENTIALS) {
    uint16_t username_length, password_length;

    username_length = strlen(mac_for_id);
    password_length = strlen(PASSWORD);

    uint32_t malloc_size = sizeof(sl_mqtt_client_credentials_t) + username_length + password_length;

    client_credentails = malloc(malloc_size);
    if (client_credentails == NULL)
      return SL_STATUS_ALLOCATION_FAILED;
    memset(client_credentails, 0, malloc_size);
    client_credentails->username_length = username_length;
    client_credentails->password_length = password_length;

    memcpy(&client_credentails->data[0], mac_for_id, username_length);
    memcpy(&client_credentails->data[username_length], PASSWORD, password_length);

    status = sl_net_set_credential(SL_NET_MQTT_CLIENT_CREDENTIAL_ID(0),
                                   SL_NET_MQTT_CLIENT_CREDENTIAL,
                                   client_credentails,
                                   malloc_size);

    if (status != SL_STATUS_OK) {
      mqtt_client_cleanup();
      printf("Failed to set credentials: 0x%lx\r\n ", status);

      return status;
    }
    printf("Set credentials Success \r\n ");

    free(client_credentails);
    mqtt_client_configuration.credential_id = SL_NET_MQTT_CLIENT_CREDENTIAL_ID(0);
  }

  status = sl_mqtt_client_init(&client, mqtt_client_event_handler);
  if (status != SL_STATUS_OK) {
    printf("Failed to init mqtt client: 0x%lx\r\n", status);

    mqtt_client_cleanup();
    return status;
  }
  printf("Init mqtt client Success \r\n");

  status = sl_net_inet_addr(MQTT_BROKER_IP, &mqtt_broker_configuration.ip.ip.v4.value);
  if (status != SL_STATUS_OK) {
    printf("Failed to convert IP address \r\n");

    mqtt_client_cleanup();
    return status;
  }

  status =
    sl_mqtt_client_connect(&client, &mqtt_broker_configuration, &last_will_message, &mqtt_client_configuration, 0);
  if (status != SL_STATUS_IN_PROGRESS) {
    printf("Failed to connect to mqtt broker: 0x%lx\r\n", status);

    mqtt_client_cleanup();
    return status;
  }
  printf("Connect to mqtt broker Success \r\n");

  //ampak_switch_device_profile_startover();
#if AMPAK_USE_SLEEP
  osDelay(1000);
  printf("Go DTIM 10\r\n");
  ampak_m4_sleep_wakeup();
#endif

  while (!is_execution_completed) {
    osThreadYield(); // TODO: keep running in this while loop
  }

  printf("Example execution completed \r\n");

  return SL_STATUS_OK;
}
