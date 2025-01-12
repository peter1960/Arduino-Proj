#ifndef DEF_H_
#define DEF_H_
#define GPS 1
// pinout https://microcontrollerslab.com/esp32-pinout-use-gpio-pins/
// #define GPS_BAUD 9600
// #define GPS_BAUD 115200
// #define GPS_BAUD 57600
#define GPS_BAUD 38400
// #define GPS_BAUD 19200
#define MON_BAUD 115200
#define RXBUFFER 500
#define TXBUFFER 500

// uncomment to enable emulator connection
#define EMULATE 1
// PINS

// First Side
#define WIFI_ON 36 // IN Wifi On Switch
#define GPIO39 39  // IN
#define GPIO34 34  // IN
#define GPIO35 35  // IN
#define GPIO32 32
#define GPIO33 33
#define GPIO25 25
#define GPIO26 26
#define GPIO27 27
#define BOARD_LED 14
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
#define xGPS_SERIAL 2 // TFT_DC
#define GPIO4 4 //TFT_RST
#define RXD2 16 // U2_TDX for GPS
#define TXD2 17 // U2_TDX for GPS
#define GPIO5 5
#define GPIO18 18 // SCK
#define GPIO19 19 // MISO
#define GPIO21 21 // SDA
#define GPIO3 3   // U0_RDX
#define GPIO1 1   // U0_TDX
#define GPIO22 22 // SCL
#define GPIO23 23 // MOSI

// #define PL_DEBUG 1
#endif