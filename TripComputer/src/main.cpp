#include <Arduino.h>
#include <display.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <def.h>
#include <serial.h>
#include <gps.h>
#include <kline.h>
#include <wifi-mqtt.h>
#include <mysecrets.h>

Display *dis;
gps *TheGPS;
uint32_t updateTime = 0; // time for next update

int d = 0;

// void plotSpeed(int value, byte ms_delay);
// void analogMeter();
// void plotLinearSat(const char *label);
// void plotPointer(void);
// void plotTriangle(int x, int y, uint32_t color);

// put function declarations here:

void setup()
{
#ifdef PL_DEBUG
  Serial.begin(115200);
#endif
  Serial.begin(115200);
  dis = new Display();
  TheGPS = new gps();
  // Serial.println("TFT_BL is on pin: " + String(TFT_BL));

  //pinMode(BOARD_LED, OUTPUT);
  pinMode(REC_ON, INPUT);
  pinMode(ECU_ON, INPUT);
  pinMode(WIFI_ON, INPUT);

#ifdef PL_DEBUG

  Serial.println("");
  Serial.println("Serial Txd is on pin: " + String(TXD2));
  Serial.println("Serial Rxd is on pin: " + String(RXD2));
  Serial.println("Setup Display");
  Serial.println("TFT_CS is on pin: " + String(TFT_CS));
  Serial.println("TFT_DC is on pin: " + String(TFT_DC));
  Serial.println("TFT_RSTG is on pin: " + String(TFT_RST));
  Serial.println("TFT_SCLK is on pin: " + String(TFT_SCLK));

#endif

#ifdef PL_DEBUG
  Serial.println("Setup Display");
#endif

  dis->screenLayout();
  // dis->analogMeter(); // Draw analogue meter

  // dis.plotLinearSat("Sat");
  //  plotLinear("A1", 1 * d, 160);
  //  plotLinear("A2", 2 * d, 160);
  //  plotLinear("A3", 3 * d, 160);
  //  plotLinear("A4", 4 * d, 160);
  //  plotLinear("A5", 5 * d, 160);
  // dis->WiFiOff();
  StartWifi();
  // dis->WiFiOn();

  OTAsetup();
  updateTime = millis(); // Next update time

#ifdef PL_DEBUG
  Serial.println("Display Done");
  Serial.println("Setup GPS");
#endif

  // GPS_Serial2Init();

#ifdef PL_DEBUG
  Serial.println("GPS Done");
#endif
}

bool connected = false;
bool LastECU = true;
bool REC_MODE = false;
bool ECU_MODE = false;
bool WIFI_MODE = false;
uint32_t ecuFlashTime = 0;
bool ecuToggle = false;
void loop()
{
  OTAcheck();
  /*
  Get the switch status
  and set the display
    */
  REC_MODE = digitalRead(REC_ON);
  dis->Rec(REC_MODE);
  ECU_MODE = digitalRead(ECU_ON);
  /*
  Check if ECU should connect and
  if connected, if not then  will flash
  */
  if (ECU_MODE && !connected)
  {
    if (ecuFlashTime <= millis())
    {
      ecuFlashTime = millis() + 500;
      ecuToggle = ! ecuToggle;
      dis->ECUConnect(ecuToggle);
    }
  }
  else if (ECU_MODE){
      dis->ECUConnect(true);

   }
   else if (!ECU_MODE){
      dis->ECUConnect(false);

   }
  WIFI_MODE = digitalRead(WIFI_ON);
  dis->Wifi(WIFI_MODE);

  // dis->ipAdress(WiFi.localIP().toString().c_str());
  dis->HasLock(TheGPS->HasLock());
  dis->speed(TheGPS->Speed());

  if (updateTime <= millis())
  {
    updateTime = millis() + 1000;
    if (ECU_MODE) {
    if (!connected)
    {
      //  Start KDS comms
      // if (updateTime <= millis())
      //{
      Serial.print(millis());
      Serial.println(" Try Start ECU");
      connected = fastInit();
      //}
    }
    connected = keepAlive();
    }
    else {
      if (connected) {
        connected = !stopComm();
      }
    }
    if (connected)
    {
      dis->ecu_speed(ECU_speed());
      dis->rpm(ECU_RPM());
    }
    else
    {
      dis->ecu_speed(-2);
      dis->rpm(-2);
    }
  }
  uint8_t c,
      cc;

  while (TheGPS->SerialAvailable())
  {
    int inByte = TheGPS->ReadByte(); // SerialRead(GPS_SERIAL);
#ifdef PL_DEBUG
    Serial.print(String(inByte));
#endif
    if (TheGPS->GPS_newFrame(inByte))
    {
#ifdef PL_DEBUG
      Serial.print("\n");
#endif
    }
  }

}
