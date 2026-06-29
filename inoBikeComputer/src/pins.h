/**
 * @file pins.h
 * @brief Pin definitions for the project.  
 * For clean INPUT_PULLUP button inputs, the safest picks are 4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33.
 */
#define EPD_SCL_PIN 18  // (SCK)
#define EPD_SDA_PIN 23  // (MOSI)

#define EPD_CS_PIN 5
#define EPD_BUSY_PIN 4
#define EPD_RES_PIN 16
#define EPD_DC_PIN 17

#define SDA_PIN 21  // (SDA)
#define SCL_PIN 22  // (SCL)

#define BUTTON_RECORD_PIN 15
#define BUTTON_RESET_PIN 14

#define WHEEL_SENSOR_PIN 27

#define WHEEL_CIRCUMFERANCE 2.199