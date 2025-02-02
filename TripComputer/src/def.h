#ifndef DEF_H_
#define DEF_H_
#define GPS 1
// pinout https://microcontrollerslab.com/esp32-pinout-use-gpio-pins/
//#define GPS_BAUD 9600
// #define GPS_BAUD 115200
#define GPS_BAUD 57600
//#define GPS_BAUD 38400
// #define GPS_BAUD 19200
#define MON_BAUD 115200
#define RXBUFFER 500
#define TXBUFFER 500
#define SPEED_SAMPLES 10  // used to average speed
// uncomment to enable emulator connection
// PINS

// First Side
#define WIFI_ON 36 // IN Wifi On Switch
#define REC_ON 39  // IN Record Mode
#define ECU_ON 34  // IN ECU Connect
#define GPIO35 35  // IN
#define K_IN 32
#define K_OUT 33
#define GPIO25 25
#define GPIO26 26
#define GPIO27 27
#define xBOARD_LED 14
#define GPIO12 12
#define GPIO13 13
#define GPIO9 9
#define GPIO10 10
// Second Side
#define GPIO11 11
#define GPIO6 6
#define GPIO7 7
#define GPIO8 8
#define MON_SERIAL 0
#define GPIO15 15 //TFT_CS
#define GPIO2 2 // TFT_DC
#define GPIO4 4 //TFT_RST
#define RXD2 16 // U2_TDX for GPS
#define TXD2 17 // U2_TDX for GPS
#define GPIO5 5
#define GPIO18 18 // TFT_SCLK
#define GPIO19 19 // TFT_MISO
#define GPIO21 21 // SDA
#define GPIO3 3   // U0_RDX
#define GPIO1 1   // U0_TDX
#define GPIO22 22 // SCL
#define GPIO23 23 // TFT_MOSI

#define PL_DEBUG_GPS 1
#endif