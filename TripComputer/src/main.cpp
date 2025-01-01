#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <def.h>
#include <serial.h>
#include <gps.h>

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#define TFT_GREY 0x5AEB

//* these are button dims
#define BX 240
#define BY 5
#define BWIDE 49
#define BHIGH 29

// linear bar sizes
#define BAR_HEIGHT 155
#define BAR_OFFSET 19
#define BAR_WIDTH 36

// Bar Pos
#define BAR_SAT_X 0
#define BAR_SAT_Y 160
#define BAR_SAT_POINTS 9

float ltx = 0;                 // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update

int old_analog = -999;  // Value last displayed
int old_digital = -999; // Value last displayed

int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = {0, -1, -1, -1, -1, -1};
int plotSatLines[BAR_SAT_POINTS + 1];
int d = 0;
void plotSpeed(int value, byte ms_delay);
void analogMeter();
void plotLinearSat(const char *label);
void plotPointer(void);
void plotTriangle(int x, int y, uint32_t color);
enum
{
  SAT_COUNT = 0
};
// put function declarations here:
int myFunction(int, int);

void setup()
{
  Serial.begin(115200);
#ifdef PL_DEBUG

  Serial.println("");
  Serial.println("Serial Txd is on pin: " + String(TXD2));
  Serial.println("Serial Rxd is on pin: " + String(RXD2));

#endif

#ifdef PL_DEBUG
  Serial.println("Setup Display");
#endif
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  // put your setup code here, to run once:
  tft.fillRect(0, 0, 239, 126, TFT_GREY);
  tft.fillRect(5, 3, 230, 119, TFT_WHITE);

  analogMeter(); // Draw analogue meter

  // Draw 6 linear meters
  byte d = 40;

  plotLinearSat("Sat");
  // plotLinear("A1", 1 * d, 160);
  // plotLinear("A2", 2 * d, 160);
  // plotLinear("A3", 3 * d, 160);
  // plotLinear("A4", 4 * d, 160);
  // plotLinear("A5", 5 * d, 160);

  updateTime = millis(); // Next update time
  int result = myFunction(2, 3);

  int bx = BX;
  tft.drawRect(bx, BY, BWIDE, BHIGH, TFT_WHITE); // Draw bezel line
  tft.fillRect(bx + 1, BY + 1, BWIDE - 2, BHIGH - 2, TFT_GREY);

  bx = bx + BWIDE + 2;
  tft.drawRect(bx, BY, BWIDE, BHIGH, TFT_WHITE); // Draw bezel line
  tft.fillRect(bx + 1, BY + 1, BWIDE - 2, BHIGH - 2, TFT_GREY);
#ifdef PL_DEBUG
  Serial.println("Display Done");
  Serial.println("Setup GPS");
#endif

  GPS_SerialInit();

#ifdef PL_DEBUG
  Serial.println("GPS Done");
#endif
  value[SAT_COUNT] = 0;
}
void loop()
{
  uint8_t c, cc;

  while (Serial2.available() > 0)
  {
    int inByte = SerialRead(GPS_SERIAL);
#ifdef PL_DEBUG
    Serial.print(String(inByte));
#endif
    if (GPS_newFrame(inByte))
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

    value[1] = 9; // 50 + 50 * sin((d + 60) * 0.0174532925);
    value[2] = 50 + 50 * sin((d + 120) * 0.0174532925);
    value[3] = 50 + 50 * sin((d + 180) * 0.0174532925);
    value[4] = 50 + 50 * sin((d + 240) * 0.0174532925);
    value[5] = 50 + 50 * sin((d + 300) * 0.0174532925);

    // unsigned long t = millis();
    plotPointer(); // It takes aout 3.5ms to plot each gauge for a 1 pixel move, 21ms for 6 gauges

    plotSpeed(value[0], 0); // It takes between 2 and 12ms to replot the needle with zero delay
    // Serial.println(millis()-t); // Print time taken for meter update
  }
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}
void DisplayStat(int sat)
{
  value[SAT_COUNT] = sat;
}
void DisplayTime(const char *time)
{
  static String xx = "xx";
  if (xx.compareTo(time) == 0)
  {
    return;
  }
  tft.setTextColor(TFT_BLACK);
  tft.drawString(xx, 10, 135, 2);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(time, 10, 135, 2);
  xx = time;
}
void analogMeter()
{
  // Meter outline
  tft.fillRect(0, 0, 239, 126, TFT_GREY);
  tft.fillRect(5, 3, 230, 119, TFT_WHITE);

  tft.setTextColor(TFT_BLACK); // Text colour

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  // will use 99% range to max of 33kmh
  for (int i = 0; i < 100; i += 3)
  {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - SPEED_ROT) * 0.0174532925);
    float sy = sin((i - SPEED_ROT) * 0.0174532925);
    uint16_t x0 = sx * (100 + tl) + 120;
    uint16_t y0 = sy * (100 + tl) + 140;
    uint16_t x1 = sx * 100 + 120;
    uint16_t y1 = sy * 100 + 140;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - SPEED_ROT) * 0.0174532925);
    float sy2 = sin((i + 5 - SPEED_ROT) * 0.0174532925);
    int x2 = sx2 * (100 + tl) + 120;
    int y2 = sy2 * (100 + tl) + 140;
    int x3 = sx2 * 100 + 120;
    int y3 = sy2 * 100 + 140;

    // Yellow zone limits
    // if (i >= -50 && i < 0) {10
    //  tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
    //  tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    //}

    // Green zone limits
    if (i >= 15 && i < 25)
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
    }

    // Orange zone limits
    if (i >= 25 && i < 33)
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
    }

    // Short scale tick length
    if (i % 25 != 0)
      tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (100 + tl) + 120;
    y0 = sy * (100 + tl) + 140;
    x1 = sx * 100 + 120;
    y1 = sy * 100 + 140;

    // Draw tick
    tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

    // Check if labels should be drawn, with position tweaks
    if (i % 25 == 0)
    {
      // Calculate label positions
      x0 = sx * (100 + tl + 10) + 120;
      y0 = sy * (100 + tl + 10) + 140;
      switch (i / 10)
      {
      case 0:
        tft.drawCentreString("0", x0, y0 - 12, 2);
        break;
      case 1:
        tft.drawCentreString("25", x0, y0 - 9, 2);
        break;
      case 2:
        tft.drawCentreString("50", x0, y0 - 6, 2);
        break;
      case 3:
        tft.drawCentreString("75", x0, y0 - 9, 2);
        break;
      case 4:
        tft.drawCentreString("100", x0, y0 - 12, 2);
        break;
      }
    }

    // Now draw the arc of the scale
    sx = cos((i + 5 - SPEED_ROT) * 0.0174532925);
    sy = sin((i + 5 - SPEED_ROT) * 0.0174532925);
    x0 = sx * 100 + 120;
    y0 = sy * 100 + 140;
    // Draw scale arc, don't draw the last part
    if (i < 50)
      tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }

  tft.drawString("%RH", 5 + 230 - 40, 119 - 20, 2); // Units at bottom right
  tft.drawCentreString("Kmh", 120, 70, 4);          // Comment out to avoid font 4
  tft.drawRect(5, 3, 230, 119, TFT_BLACK);          // Draw bezel line

  plotSpeed(0, 0); // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotSpeed(int value, byte ms_delay)
{
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  char buf[8];
  dtostrf(value, 4, 0, buf);
  tft.drawRightString(buf, 40, 119 - 20, 2);

  if (value < 0)
    value = 0; // Limit value to emulate needle end stops
  if (value > 35)
    value = 35;

  // Move the needle util new value reached
  while (!(value == old_analog))
  {
    if (old_analog < value)
      old_analog++;
    else
      old_analog--;

    if (ms_delay == 0)
    {
      old_analog = value; // Update immediately id delay is 0
    }
    float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
    // Calcualte tip of needle coords
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((sdeg + 90) * 0.0174532925);

    // Erase old needle image
    tft.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_WHITE);
    tft.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_WHITE);
    tft.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_WHITE);

    // Re-plot text under needle
    tft.setTextColor(TFT_BLACK);
    tft.drawCentreString("Kmh", 120, 70, 4); // // Comment out to avoid font 4

    // Store new needle end coords for next erase
    ltx = tx;
    osx = sx * 98 + 120;
    osy = sy * 98 + 140;

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
    tft.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_RED);
    tft.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_MAGENTA);
    tft.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_RED);

    // Slow needle down slightly as it approaches new postion
    if (abs(old_analog - value) < 10)
      ms_delay += ms_delay / 5;

    // Wait before next update
    delay(ms_delay);
  }
}

// #########################################################################
//  Draw a linear meter on the screen
// #########################################################################
void plotLinearSat(const char *label)
{

  tft.drawRect(BAR_SAT_X, BAR_SAT_Y, BAR_WIDTH, BAR_HEIGHT, TFT_GREY);
  tft.fillRect(BAR_SAT_X + 2, BAR_SAT_Y + BAR_OFFSET, BAR_WIDTH - 3, BAR_HEIGHT - (BAR_OFFSET * 2), TFT_WHITE);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawCentreString(label, BAR_SAT_X + BAR_WIDTH / 2, BAR_SAT_Y + 2, 2);

  for (int i = 0; i <= BAR_SAT_POINTS; i += 1)
  {
    tft.drawFastHLine(BAR_SAT_X + 20, BAR_SAT_Y + 27 + (i * 11), 6, TFT_BLACK);
    // save in reverse order so vale matches location
    plotSatLines[BAR_SAT_POINTS - i] = BAR_SAT_Y + 27 + (i * 11);
  }

  tft.drawFastHLine(BAR_SAT_X + 20, BAR_SAT_Y + 27 + (0 * 11), 10, TFT_BLACK);
  tft.drawFastHLine(BAR_SAT_X + 20, BAR_SAT_Y + 27 + (9 * 11), 10, TFT_BLACK);

  // plotTriangle(BAR_SAT_X, BAR_SAT_Y);

  tft.drawCentreString("---", BAR_SAT_X + BAR_WIDTH / 2, BAR_SAT_Y + BAR_HEIGHT - (BAR_OFFSET), 2);
}

void plotTriangle(int x, int y, uint32_t color)
{

  tft.fillTriangle(x + 3, y, x + 3 + 16, y, x + 3, y - 5, color);
  tft.fillTriangle(x + 3, y, x + 3 + 16, y, x + 3, y + 5, color);
}
// #########################################################################
//  Adjust 6 linear meter pointer positions
// #########################################################################
void plotPointer(void)
{
  int dy = 187;
  byte pw = 16;

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  char buf[8];
  dtostrf(value[SAT_COUNT], 4, 0, buf);
  tft.drawCentreString(buf, BAR_SAT_X + BAR_WIDTH / 2, BAR_SAT_Y + BAR_HEIGHT - (BAR_OFFSET), 2);
  if (value[SAT_COUNT] != old_value[SAT_COUNT])
  {
    plotTriangle(BAR_SAT_X, plotSatLines[old_value[SAT_COUNT]], TFT_WHITE);
    plotTriangle(BAR_SAT_X, plotSatLines[value[SAT_COUNT]], TFT_RED);
    old_value[SAT_COUNT] = value[SAT_COUNT];
  }
  // plotTriangle(BAR_SAT_X, plotSatLines[0]);

  return;
  // Move the 6 pointers one pixel towards new value
  for (int i = 0; i < 6; i++)
  {
    char buf[8];
    dtostrf(value[i], 4, 0, buf);
    tft.drawRightString(buf, i * 40 + 36 - 5, 187 - 27 + 155 - 18, 2);

    int dx = 3 + 40 * i;
    if (value[i] < 0)
    {
      value[i] = 0; // Limit value to emulate needle end stops
    }
    if (i == SAT_COUNT)
    {
      // pw = 25;
      if (value[i] > 9)
      {
        value[i] = 9;
      }
    }
    else
    {
      // pw =16;
      if (value[i] > 100)
      {
        value[i] = 100;
      }
    }
    while (!(value[i] == old_value[i]))
    {
      dy = 187 + 100 - old_value[i];
      if (old_value[i] > value[i])
      {
        tft.drawLine(dx, dy - 5, dx + pw, dy, TFT_WHITE);
        old_value[i]--;
        tft.drawLine(dx, dy + 6, dx + pw, dy + 1, TFT_RED);
      }
      else
      {
        tft.drawLine(dx, dy + 5, dx + pw, dy, TFT_WHITE);
        old_value[i]++;
        tft.drawLine(dx, dy - 6, dx + pw, dy - 1, TFT_RED);
      }
    }
  }
}