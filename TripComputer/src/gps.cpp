#include <Arduino.h>
#include "def.h"
#include <SoftwareSerial.h>
// #include "serial.h"
#include "gps.h"
#define GPS_MAIN
#define SPEEDS 5

HardwareSerial gpsSerial(2); // ( RXD2, TXD2) ;

flags_struct_t fc;
uint32_t init_speed[SPEEDS] = {9600, 19200, 38400, 57600, 115200};
// Force 38400
// uint32_t init_speed[1] = {38400};
int GPS_numSat;
bool updateTimex = true;
unsigned long last_seconds;

// void DisplayStat(int sat);
// void DisplayTime(const char *time);
void SerialWriteHex(uint8_t c);
uint16_t grab_fields(char *src, uint8_t mult);
uint8_t hex_c(uint8_t n);
#ifdef PL_DEBUG_GPS
uint32_t display_lock = 0;
#endif
gps::gps()
{
  for (int x = 0; x < SPEED_SAMPLES; x++)
  {
    avg_GPS_speed[x] = 0;
  }
#ifdef PL_DEBUG_GPS
  Serial.println("GPS Setup Start =================");
#endif
  GPS_Serial2Init();
#ifdef PL_DEBUG_GPS
  Serial.println("GPS Done =====================");
#endif
}

bool gps::HasLock()
{
#ifdef PL_DEBUG_GPS
  if (display_lock < millis())
  {
    Serial.println("Check Lock");
    display_lock = millis() + 10000;
  }
#endif
  return fc.GPS_FIX == 1;
}
float gps::Speed()
{
  if (HasLock())
  {
    return GPS_speed;
  }
  return -2.0;
}

void gps::SerialGpsPrint(const char *str)
{
}
void gps::SerialGpsPrintPROGMEM(const char PROGMEM *str)
{
  char b;
  while (str && (b = pgm_read_byte(str++)))
  {
    // Serial2WriteGPS(b);
    gpsSerial.write(b);
    gpsSerial.flush();
    delay(5);
  }
  Serial.println("-------------");
}

void gps::GPS_Serial2Init(void)
{
  // gpsSerial.begin(GPS_SERIAL, GPS_BAUD);

#ifdef PL_DEBUG_GPS
  Serial.print("Start GPS INIT\n");
#endif

  for (uint8_t i = 0; i < SPEEDS; i++)
  {
    // SerialWriteString(MON_SERIAL, "Speed\n");
#ifdef PL_DEBUG_GPS
    Serial.print("Try ");
    Serial.println(init_speed[i]);
#endif

    // Serial2Open(init_speed[i]); // switch UART speed for sending SET BAUDRATE command (NMEA mode)
    gpsSerial.begin(init_speed[i], SERIAL_8N1, RXD2, TXD2);

#if (GPS_BAUD == 9600)
    SerialGpsPrintPROGMEM(PSTR("$PUBX,41,1,0003,0001,9600,0*23\r\n")); // 19200 baud - minimal speed for 5Hz update rate
#endif
#if (GPS_BAUD == 19200)
    SerialGpsPrintPROGMEM(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n")); // 19200 baud - minimal speed for 5Hz update rate
#endif
#if (GPS_BAUD == 38400)
    SerialGpsPrintPROGMEM(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n")); // 38400 baud
#endif
#if (GPS_BAUD == 57600)
    SerialGpsPrintPROGMEM(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n")); // 57600 baud
#endif
#if (GPS_BAUD == 115200)
    SerialGpsPrintPROGMEM(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n")); // 115200 baud
#endif
    gpsSerial.flush();

    while (!gpsSerial.availableForWrite())
    {
      delay(10);
#ifdef PL_DEBUG_GPS
      Serial.print("X");
#endif
    }

#ifdef PL_DEBUG_GPS
    Serial.print("Buffer empty\n");
#endif
    // Serial2Close();
    gpsSerial.end();
  }
  delay(50);
  // Serial2Open(GPS_BAUD);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

#ifdef PL_DEBUG_GPS
  Serial.print(String(__LINE__));
  Serial.print(" Send UBLOX config\n");
#endif
  for (uint8_t i = 0; i < sizeof(UBLOX_INIT); i++)
  { // send configuration data in UBX protocol
    // Serial2WriteGPS(pgm_read_byte(UBLOX_INIT + i));
    gpsSerial.write(pgm_read_byte(UBLOX_INIT + i));
    // gpsSerial.flush();
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
#ifdef PL_DEBUG_GPS
  Serial.print(String(__LINE__));
  Serial.print(" Send UBLOX complete\n");
#endif
}

uint16_t gps::grab_fields(char *src, uint8_t mult)
{ // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;

  for (i = 0; src[i] != 0; i++)
  {
    if (src[i] == '.')
    {
      i++;
      if (mult == 0)
      {
        break;
      }
      else
      {
        src[i + mult] = 0;
      }
    }
    tmp *= 10;
    if (src[i] >= '0' && src[i] <= '9')
    {
      tmp += src[i] - '0';
    }
  }
  return tmp;
}
uint8_t hex_c(uint8_t n)
{ // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if (n > 9)
  {
    n -= 7;
  }
  n &= 0x0F;
  return n;
}

bool gps::SerialAvailable()
{
  return (gpsSerial.available() > 0);
}

uint8_t gps::ReadByte()
{
  return gpsSerial.read();
}
bool gps::GPS_newFrame(uint8_t data)
{

  uint8_t st = _step + 1;
  bool ret = false;
  // Serial.println(st);

  if (st == 2)
  {
    if (PREAMBLE2 != data)
    {
      /*
      Serial.print("Step ");
      Serial.print(st);
      Serial.print(" wrong data ");
      Serial.println(data);
      */
      st--; // in case of faillure of the 2nd header byte, still test the first byte
    }
    else
    {
      /*
      Serial.print("Step ");
      Serial.print(st);
      Serial.print(" data ");
      Serial.println(data);
      */
    }
  }
  if (st == 1)
  {
    if (PREAMBLE1 != data)
    {
      /*
      Serial.print("Step ");
      Serial.print(st);
      Serial.print(" wrong data ");
      Serial.println(data);
      */
      st--;
    }
    else
    {
      /*
      Serial.print("Step ");
      Serial.print(st);
      Serial.print(" data ");
      Serial.println(data);
      */
    }
  }
  else if (st == 3)
  { // CLASS byte, not used, assume it is CLASS_NAV
    // Serial.println(st);
    _ck_b = _ck_a = data; // reset the checksum accumulators
  }
  else if (st > 3 && st < 8)
  {
// SerialWriteHex(MON_SERIAL, GPS_numSat);
#ifdef PL_DEBUG_GPS
    // Serial.print(st);
#endif
    _ck_b += (_ck_a += data); // checksum byte
    if (st == 4)
    {
      // Serial.println(st);
      _msg_id = data;
    }
    else if (st == 5)
    {
      // Serial.println(st);
      _payload_length = data; // payload length low byte
    }
    else if (st == 6)
    {
      // Serial.println(st);
      _payload_length += (uint16_t)(data << 8);
      if (_payload_length > 512)
      {
        st = 0;
      }
      _payload_counter = 0; // prepare to receive payload
    }
    else
    {
      if (_payload_counter + 1 < _payload_length)
      {
        st--; // stay in the same state while data inside the frame
      }
      if (_payload_counter < sizeof(_buffer))
      {
        _buffer.bt.bytes[_payload_counter] = data;
      }
      _payload_counter++;
    }
  }
  else if (st == 8)
  {
    // Serial.println(st);
    if (_ck_a != data)
    {
      st = 0; // bad checksum
              //      SerialWriteString(MON_SERIAL, "Bad Sum\n");
    }
  }
  else if (st == 9)
  {
    st = 0;
    if (_ck_b == data)
    { // good checksum
      //Serial.println(_msg_id);
      if (_msg_id == MSG_POSLLH)
      {
#ifdef PL_DEBUG_GPS
        Serial.print("MSG_POSLLH Pos ");
        Serial.print(String(_buffer.posllh.latitude));
        Serial.print(",");
        Serial.print(String(_buffer.posllh.longitude));
        Serial.print(" \n");
#endif
        if (fc.GPS_FIX)
        {

          // SerialWriteString(MON_SERIAL, "Have Fix\n");
          uint32_t GPS_coordLON = _buffer.posllh.longitude;
          uint32_t GPS_coordLAT = _buffer.posllh.latitude;
          //  p GPS_altitude = _buffer.posllh.altitude_msl / 1000; // alt in m
          uint32_t gpsTimeOfWeek = _buffer.posllh.time; // not used for the moment
          unsigned long gpsWeekNumber = 2100;           // Your GPS week number

          unsigned long totalSeconds = gpsTimeOfWeek / 1000;
          unsigned long seconds = totalSeconds % 60;
          updateTimex = (seconds != last_seconds);
          last_seconds = seconds;
          unsigned long totalMinutes = totalSeconds / 60;
          unsigned long minutes = totalMinutes % 60;
          unsigned long hours = (totalMinutes / 60 + gpsWeekNumber * 168) % 24; // Each GPS week is 168 hours
          hours += 8;

          if (hours > 24)
          {
            hours -= 24;
          }

#ifdef PL_DEBUG_GPS
          Serial.print(" Hours:" + String(hours));
#endif

          char buffer[40]; // Buffer to hold the converted string
          //  Convert the numerical value to a string using sprintf
          sprintf(buffer, "%02lu : %02lu : %02lu", hours, minutes, seconds);

#ifdef PL_DEBUG_GPS
          Serial.print(" Time:" + String(buffer));
#endif

          // PL DisplayTime(buffer);
          /*
          myGLCD.setColor(255, 255, 255);
          display(D_TIME, "Time:");
          display(D_TIME_H, hours);
          display(D_TIME_M, minutes);
          display(D_TIME_S, seconds);
          if (updateTime)
          {
            codisplay(D_LAT, GPS_coordLAT);
            codisplay(D_LON, GPS_coordLON);
            updateTime = false;
          }

          */
        }
        else
        {
          // PL DisplayTime("No Lock");
          //  display(D_TIME, "Time:");
        }
        ret = true; // POSLLH message received, allow blink GUI icon and LED, frame available for nav computation
      }
      else if (_msg_id == MSG_SOL)
      {
#ifdef PL_DEBUG_GPS
        Serial.print("Sol ");
        Serial.print(String(_buffer.solution.fix_type));
        Serial.print(" Time ");
        Serial.print(String(_buffer.solution.time));
        Serial.print(" Week ");
        Serial.print(String(_buffer.solution.week));
        Serial.print(" Type ");
        switch (_buffer.solution.fix_type)
        {
        case 0x00:
          Serial.print(String("'No Fix'"));
          break;
        case 0x01:
          Serial.print(String("'Dead Recon'"));
          break;
        case 0x02:
          Serial.print(String("'2D'"));
          break;
        case 0x03:
          Serial.print(String("'3D'"));
          break;
        case 0x04:
          Serial.print(String("'GPS+Dead Recon'"));
          break;
        case 0x05:
          Serial.print(String("'Time Only'"));
          break;
        default:
          Serial.print(String("'?????'"));
        }
        Serial.print(" Status ");
        Serial.print(String(_buffer.solution.fix_status));
        Serial.print("  Sat Count ");
        Serial.println(String(_buffer.solution.satellites));
#endif
        // SerialWriteString(MON_SERIAL, "Have Sol\n");
        fc.GPS_FIX = 0;
        if ((_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D || _buffer.solution.fix_type == FIX_2D))
        {
#ifdef PL_DEBUG_GPS
          Serial.print("\n=====Fix======\n");
#endif
          fc.GPS_FIX = 1;

#ifdef PL_DEBUG_GPS
          SerialWriteHex(_buffer.solution.fix_type);
#endif
        }
        else
        {
          // display(D_STATUS, "No Lock");
        }

        int tmpGPS_numSat = _buffer.solution.satellites;
        // PL DisplayStat(tmpGPS_numSat);
        if (tmpGPS_numSat != GPS_numSat)
        {
          // display(D_SAT, "Sat Count:");
          char buffer[10]; // Buffer to hold the converted string
          sprintf(buffer, "%d", tmpGPS_numSat);
#ifdef PL_DEBUG_GPS
          Serial.print("Sat Count: ");
          Serial.println(buffer);
#endif
          // display(D_SAT_COUNT, buffer);
          GPS_numSat = tmpGPS_numSat;
        }

        if (GPS_numSat < 3)
        {
          digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
        }
        else
        {
          digitalWrite(LED_BUILTIN, LOW); // turn the LED on (HIGH is the voltage level)
        }
      }
      else if (_msg_id == MSG_VELNED)
      {

        GPS_speed = _buffer.velned.speed_2d;
        GPS_speed = (GPS_speed * 60 * 60) / 100 / 1000;
        float tSpeed = 0;
        for (int x = 0; x < SPEED_SAMPLES - 1; x++)
        {
          // move them down by 1
          avg_GPS_speed[x] = avg_GPS_speed[x + 1];
          // sum the values
          tSpeed += avg_GPS_speed[x];
        }
        // add on new last value
        avg_GPS_speed[SPEED_SAMPLES] = GPS_speed;
        tSpeed += avg_GPS_speed[SPEED_SAMPLES];
        // get average
        GPS_speed = tSpeed / SPEED_SAMPLES;
        //  p GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000); // Heading 2D deg * 100000 rescaled to deg * 10 //not used for the moment
      }
    }
  }
  _step = st;
  return ret;
}

void gps::insertPeriod(String &number, unsigned int position)
{
  if (position >= 0 && position < number.length())
  {
    number = number.substring(0, position) + "." + number.substring(position);
  }
}

// bool digitChanged(int index)
//{
//   // Logic to check if the digit at the given index has changed
// }

void gps::codisplay(int key, uint32_t num)
{

  char formattedString[20];             // Adjust the size accordingly
  sprintf(formattedString, "%ld", num); // Add commas as thousands separators
  String buffer = String(formattedString);
  insertPeriod(buffer, 3);
  // display(key, buffer);
  updateTimex = false;
}
void display(int key, unsigned long num)
{
  char buffer[20]; // Buffer to hold the converted string
  sprintf(buffer, "%02lu", num);
  // display(key, buffer);
}
void display(int key, String text)
{
  // if (text.compareTo(values[key]) != 0)
  {
    int r = 255;
    int g = 255;
    int b = 255;
    //    myGLCD.setColor(0, 0, 0);
    //    myGLCD.print(values[key], pos[key][D_X], pos[key][D_Y]);
    if (key == D_TIME)
    {
      if (fc.GPS_FIX)
      {
        b = 0;
      }
      else
      {
        b = 0;
        g = 0;
      }
    }
    //    myGLCD.setColor(r, g, b);
    //    myGLCD.print(text, pos[key][D_X], pos[key][D_Y]);
    //    values[key] = text;
  }
}

long Update(long now, long last, int x, int y)
{
  long nRet = last;
  if (now != last)
  {
    //   myGLCD.setColor(0, 0, 0);
    //   myGLCD.printNumI(last, x, y, 2, '0');
    //   myGLCD.setColor(255, 255, 255);
    //   myGLCD.printNumI(now, x, y, 2, '0');
  }
  return now;
}

void SerialWriteHex(uint8_t c)
{
#ifdef PL_DEBUG_GPS

  char hexBuffer[5]; // To hold the hex representation (e.g., "0x12\0")
  snprintf(hexBuffer, sizeof(hexBuffer), "0x%02X", (uint8_t)(c));
  // Send each character of the hexBuffer individually
  for (int i = 0; hexBuffer[i] != '\0'; i++)
  {
    Serial.print((uint8_t)(hexBuffer[i]));
  }
  Serial.print(" ");
#endif
}