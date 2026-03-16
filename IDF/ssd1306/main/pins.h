#include "driver/gpio.h"

#define BUTTON_PAGE GPIO_NUM_13
//#define PULSE_LED_OUT GPIO_NUM_16
#define WHEEL_PULSE GPIO_NUM_17
#define PULSE_OUT GPIO_NUM_16
#define I2C_MASTER_SCL_IO GPIO_NUM_22 /*!&lt; gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_21 /*!&lt; gpio number for I2C master data  */
// ADC setup: ADC1 channel 0 = GPIO36
#define ADC_CHANNEL ADC1_CHANNEL_0
//SD Card
#define SD_CS GPIO_NUM_5
#define SD_MOSI GPIO_NUM_23
#define SD_MISO GPIO_NUM_19
#define SD_SCK GPIO_NUM_18
