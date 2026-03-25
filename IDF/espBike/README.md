**Design Notes:
| Device  | Purpose | Interface | Pins ESP |
| ------------- | ------------- |------------- |------------- |
| esp32  | core  |||
| toggle  | record on/off  |1||
| button | reset |1||
| GPS| time/location |Serial||
| RTC| Store time|SDA/SCL||
| ePaper|Display|SDL/SDA/DC/CS/RES||
| SD|Record Data|MISO/MOSI/CS/SCK||
| LED|hearbeat|1|GPIO16|

https://www.nextpcb.com/blog/esp32-pinout-the-ultimate-guide
