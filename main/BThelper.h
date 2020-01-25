
#define SPP_SERVER_NAME "SPP_SERVER"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_NOTHING 2
#define SPP_SHOW_MODE SPP_SHOW_NOTHING    /*Choose show mode: show data, speed or nothing*/


#define BT_DEV_NAME_PREFIX CONFIG_DEV_NAME_PREFIX
#define BT_DEV_NAME_PREFIX_LEN (sizeof(BT_DEV_NAME_PREFIX) - 1)

#define BT_LED_CONNECTED    0
#define BT_LED_DISCONNECTED 1

#define SPP_BUFF_SZ 128

void initBT(void);
void bt_set_device_name(void);
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);