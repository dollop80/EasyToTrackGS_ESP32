
// SDA - GPIO21
#define PIN_SDA 21

// SCL - GPIO22
#define PIN_SCL 22


void oled_task(void *pvParameter);
void initOled(void);
void oledShowProgramming(uint8_t done);