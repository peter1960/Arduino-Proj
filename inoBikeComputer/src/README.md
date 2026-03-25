**Design Notes:
| Device  | Purpose | Interface | Pins ESP |
| ------------- | ------------- |------------- |------------- |
| esp32  | core  |||
| toggle  | record on/off  |1||
| button | reset |1||
| GPS| time/location |Serial||
| RTC| Store time|SDA/SCL|21/22|
| ePaper|Display|SDL/SDA/DC/CS/RES|4,5,16,17,18,23|
| SD|Record Data|MISO/MOSI/CS/SCK||
| LED|hearbeat|1||

https://www.nextpcb.com/blog/esp32-pinout-the-ultimate-guide
EPD Code: https://github.com/WeActStudio/WeActStudio.EpaperModule/blob/master/Example/EpaperModuleTest_Arduino_ESP32/EpaperModuleTest_Arduino_ESP32.ino