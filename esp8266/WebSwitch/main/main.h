#include "freertos/FreeRTOS.h"
#define SW_VER "v1.2"
#define GPIO_OUTPUT_0    2
#define GPIO_OUTPUT_1    16
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_0) | (1ULL<<GPIO_OUTPUT_1))
#define GPIO_INPUT_IO_0     0
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)


#define I2C_EXAMPLE_MASTER_SCL_IO           5                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           4                /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define TTL_PACKET_TIME                     45              //ogni 45 secondi
#define USEOLED //abilita OLED
