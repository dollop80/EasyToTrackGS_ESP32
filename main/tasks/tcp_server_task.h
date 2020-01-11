#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CONFIG_EXAMPLE_IPV4
#define PORT 23

#define EXAMPLE_ESP_WIFI_SSID "ETT-GS"
#define EXAMPLE_ESP_WIFI_PASS "ett-gs"
#define EXAMPLE_MAX_STA_CONN 2


void initWiFi(void);
void tcp_server_task(void *pvParameters);
void cb_connection_ok(void *pvParameter);
void cb_ap_ok(void *pvParameter);
void cb_sta_disconnected(void *pvParameter);

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data);
void wifi_init_softap(void);
									