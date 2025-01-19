#include <Arduino.h>
#include <display.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <def.h>
#include <serial.h>
#include <gps.h>
#include <kawa.h>
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

  pinMode(BOARD_LED, OUTPUT);
  pinMode(REC_ON, INPUT_PULLUP);

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
  dis->WiFiOff();
  StartWifi();
  dis->WiFiOn();

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

bool ECUconnected = false;
bool LastECU = true;
bool REC_MODE = false;
void loop()
{
  REC_MODE = digitalRead(REC_ON);
  dis->Rec(REC_MODE);

  dis->ipAdress(WiFi.localIP().toString().c_str());
  OTAcheck();
  dis->HasLock(TheGPS->HasLock());
  dis->speed(TheGPS->Speed());

  if (!ECUconnected)
  {
    //  Start KDS comms
    if (updateTime <= millis())
    {
      ECUconnected = initPulse();
      Serial.print(millis());
      Serial.println(" Start ECU");
    }
  }
  ECUconnected = alive();
  dis->ECUConnect(ECUconnected);

  if (ECUconnected)
  {
    {
      if (updateTime <= millis())
      {
      }
      dis->avg_speed(speed());
      dis->rpm(RPM());
      updateTime = millis() + 1000;
    }
  }
  else {
    dis->avg_speed(-2);
    dis->rpm(-2);
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

  if (updateTime <= millis())
  {
    updateTime = millis() + 250; // Delay to limit speed of update

    d += 4;
    if (d >= 360)
      d = 0;

    // value[0] = map(analogRead(A0), 0, 1023, 0, 100); // Test with value form Analogue 0
    // value[1] = map(analogRead(A1), 0, 1023, 0, 100); // Test with value form Analogue 1
    // value[2] = map(analogRead(A2), 0, 1023, 0, 100); // Test with value form Analogue 2
    // value[3] = map(analogRead(A3), 0, 1023, 0, 100); // Test with value form Analogue 3
    // value[4] = map(analogRead(A4), 0, 1023, 0, 100); // Test with value form Analogue 4
    // value[5] = map(analogRead(A5), 0, 1023, 0, 100); // Test with value form Analogue 5

    // Create a Sine wave for testing
    /*
        value[1] = 9; // 50 + 50 * sin((d + 60) * 0.0174532925);
        value[2] = 50 + 50 * sin((d + 120) * 0.0174532925);
        value[3] = 50 + 50 * sin((d + 180) * 0.0174532925);
        value[4] = 50 + 50 * sin((d + 240) * 0.0174532925);
        value[5] = 50 + 50 * sin((d + 300) * 0.0174532925);
    */
    // unsigned long t = millis();
    // dis->plotPointer(); // It takes aout 3.5ms to plot each gauge for a 1 pixel move, 21ms for 6 gauges

    // dis.plotSpeed(value[0], 0); // It takes between 2 and 12ms to replot the needle with zero delay
    //  Serial.println(millis()-t); // Print time taken for meter update
  }
  // put your main code here, to run repeatedly:
}
