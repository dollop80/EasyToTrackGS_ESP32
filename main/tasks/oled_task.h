
// SDA - GPIO21
#define PIN_SDA 21

// SCL - GPIO22
#define PIN_SCL 22

#define U_OLED_TYPE CONFIG_DEFAULT_OLED_TYPE
#define U_OLED_FLIP CONFIG_DEFAULT_OLED_FLIP

void oled_task(void *pvParameter);
void initOled(void);
void oledShowProgramming(void *pvParameter);
void setProgrammingRes(uint8_t res);