
#ifndef TPL0101_ESP32_HAL_H_
#define TPL0101_ESP32_HAL_H_

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#define TPL0401X_SENSOR_ADDR 0x2E //or 0x7D

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define I2C_MASTER_SCL_IO 22               /*!< gpio number for i2c slave clock */
#define I2C_MASTER_SDA_IO 21               /*!< gpio number for i2c slave data */

#define I2C_MASTER_NUM I2C_NUM_1           //  I2C port number for master dev
#define I2C_MASTER_TX_BUF_DISABLE   0      //  I2C master do not need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0      //  I2C master do not need buffer
#define I2C_MASTER_FREQ_HZ          400000  //  I2C master clock frequency

esp_err_t i2c_tpl0401_init(void);
esp_err_t i2c_tpl0401_set(uint8_t val);

#endif //TPL0101_ESP32_HAL_H_