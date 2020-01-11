#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "../define.h"
#include "../tracker.h"

#include "components/wifi-manager/src/wifi_manager.h"

#include "tcp_server_task.h"


extern int16_t gWFsock;
extern wifi_mode gWifi_m;
extern TaskHandle_t xHandleTCP;

static const char TAG[] = "ETT-TCP";

void initWiFi()
{
	#if 0
	wifi_init_softap();
#else
	/* start the wifi manager */
	wifi_manager_start();

	/* register a callback as an example to how you can integrate your code with the wifi manager */
	wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_ok);
	wifi_manager_set_callback(ORDER_START_AP, &cb_ap_ok);
	wifi_manager_set_callback(EVENT_STA_DISCONNECTED, &cb_sta_disconnected);
#endif
}

void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");	

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        gWFsock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (gWFsock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");

        while (1) {
            int len = recv(gWFsock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
				//ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
				//uart_write_bytes(UART_NUM_1, (const char *) rx_buffer, len);
				decode_packet_and_send_to_gs((const char *) rx_buffer, len);
			}
        }

        if (gWFsock != -1) {

            ESP_LOGE(TAG, "Closing socket and restarting...");

			close(listen_sock);
			shutdown(listen_sock,0);

            close(gWFsock);
            shutdown(gWFsock, 0);
			
			gWFsock = -1;

			vTaskDelay(50);
        }
    }
    vTaskDelete(NULL);
}



void cb_connection_ok(void *pvParameter){
	ESP_LOGI(TAG, "I have a connection!");
	gWifi_m = STA;
	if(xHandleTCP == NULL)
		xTaskCreate(tcp_server_task, "tcp_server", 1024*2, NULL, 10, &xHandleTCP);
}

void cb_ap_ok(void *pvParameter){
	ESP_LOGI(TAG, "I'm in AP mode");
	gWifi_m = AP;
	if(xHandleTCP == NULL)
		xTaskCreate(tcp_server_task, "tcp_server", 1024*2, NULL, 10, &xHandleTCP);
}

void cb_sta_disconnected(void *pvParameter){
	gWifi_m = NONE;
}

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
				 
		if(xHandleTCP == NULL)
			xTaskCreate(tcp_server_task, "tcp_server", 1024*2, NULL, 10, &xHandleTCP);
	
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}



void wifi_init_softap(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

