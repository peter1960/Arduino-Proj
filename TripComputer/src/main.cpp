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
// TFT_eSPI xtft = TFT_eSPI(); // Invoke custom library
Display *dis;
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
  dis = new Display();
/*  
  Serial.println("Setup Display");
  Serial.println("Serial Txd is on pin: " + String(TXD2));
  Serial.println("Serial Rxd is on pin: " + String(RXD2));
  Serial.println("TFT_CS is on pin: " + String(TFT_CS));
  Serial.println("TFT_DC is on pin: " + String(TFT_DC));
  Serial.println("TFT_RSTG is on pin: " + String(TFT_RST));
  Serial.println("TFT_SCLK is on pin: " + String(TFT_SCLK));
  //Serial.println("TFT_BL is on pin: " + String(TFT_BL));
*/

  pinMode(BOARD_LED, OUTPUT);

#ifndef EMULATE
  Serial.begin(115200);
#ifdef PL_DEBUG

  Serial.println("");
  Serial.println("Serial Txd is on pin: " + String(TXD2));
  Serial.println("Serial Rxd is on pin: " + String(RXD2));

#endif

#ifdef PL_DEBUG
  Serial.println("Setup Display");
#endif
#else
  /* Speed used by emulator */

#endif

  dis->screenLayout();
  //dis->analogMeter(); // Draw analogue meter

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

#ifndef EMULATE
#ifdef PL_DEBUG
  Serial.println("Display Done");
  Serial.println("Setup GPS");
#endif
#endif

  GPS_Serial2Init();

#ifndef EMULATE
#ifdef PL_DEBUG
  Serial.println("GPS Done");
#endif
#endif
}

bool ECUconnected = false;

void loop()
{
  dis->ipAdress(WiFi.localIP().toString().c_str());
  OTAcheck();
  // if (!ECUconnected)
  //{
  //  Start KDS comms
  // ECUconnected = initPulse();

  if (ECUconnected)
  {
    digitalWrite(BOARD_LED, LOW);
  }
  else
  {
    digitalWrite(BOARD_LED, HIGH);
  }
  //}
  uint8_t c, cc;

  while (Serial2.available() > 0)
  {
    int inByte = Serial2.read(); // SerialRead(GPS_SERIAL);
#ifndef EMULATE
#ifdef PL_DEBUG
    Serial.print(String(inByte));
#endif
#endif
    if (GPS_newFrame(inByte))
    {
#ifndef EMULATE
#ifdef PL_DEBUG
      Serial.print("\n");
#endif
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
    //dis->plotPointer(); // It takes aout 3.5ms to plot each gauge for a 1 pixel move, 21ms for 6 gauges

    // dis.plotSpeed(value[0], 0); // It takes between 2 and 12ms to replot the needle with zero delay
    //  Serial.println(millis()-t); // Print time taken for meter update
  }
  // put your main code here, to run repeatedly:
}
