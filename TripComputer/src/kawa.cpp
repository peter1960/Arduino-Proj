#include <Arduino.h>
#include <def.h>
#include <kawa.h>
#include "SoftwareSerial.h"
EspSoftwareSerial::UART xcpuSerial;
constexpr uint32_t CPUBPS = 10400;

bool ECU_alive()
{
  uint8_t rLen;
  uint8_t req[2];
  uint8_t resp[5];
  req[0] = 0x21;
  req[1] = 0x0B;
  rLen = xECU_sendRequest(req, resp, 2, 3);
  /*
  Serial.print("Bytes Reply");
  Serial.print(rLen);
  Serial.print(" ");
  Serial.print(resp[0]);
  Serial.print(" ");
  Serial.print(resp[1]);
  Serial.print(" ");
  Serial.print(resp[2]);
  Serial.print(" ");
  */
  Serial.println(" See if ECU connected");
  
  if (resp[0] == 0x7f && resp[1] == 0x21 && resp[2] == 0x10)
  {
    Serial.println(" Error from ECU");
    return false;
  }
  return rLen > 0;
}
float xECU_RPM()
{
  uint8_t rLen;
  uint8_t req[2];
  uint8_t resp[4];
  req[0] = 0x21;
  req[1] = 0x09;
  rLen = xECU_sendRequest(req, resp, 2, 4);
  float _speed = ((resp[0] * 100) + resp[1]);
  Serial.print("RPM Ask - > Len ");
  Serial.print(rLen);
  Serial.print(" ");
  Serial.print(resp[0]);
  Serial.print(" ");
  Serial.print(resp[1]);
  Serial.print(" ");
  Serial.print(resp[2]);
  Serial.print(" ");
  if (rLen > 3)
  {
    Serial.print(resp[3]);
    Serial.print(" ");
  }
  Serial.print(" Calculated: ");
  Serial.println(_speed);
  return _speed;
}
float xECU_speed()
{
  uint8_t rLen;
  uint8_t req[2];
  uint8_t resp[5];
  req[0] = 0x21;
  req[1] = 0x0C;
  rLen = xECU_sendRequest(req, resp, 2, 3);
  float _speed = 100.0;
  return _speed;
}
bool ECU_initPulse()
{
  uint8_t rLen;
  uint8_t req[2];
  uint8_t resp[3];

  xcpuSerial.end();
  // pinMode(K_OUT, OUTPUT);

  // This is the ISO 14230-2 "Fast Init" sequence.

  digitalWrite(K_OUT, HIGH);
  delay(300);
  digitalWrite(K_OUT, LOW);
  delay(25);
  digitalWrite(K_OUT, HIGH);
  delay(25);

  // Serial.begin(10400);
  xcpuSerial.begin(CPUBPS, SWSERIAL_8N1, K_IN, K_OUT, false);
  // Start Communication is a single byte "0x81" packet.
  req[0] = 0x81;
  rLen = xECU_sendRequest(req, resp, 1, 3);

  delay(ISORequestDelay);
  // Response should be 3 bytes: 0xC1 0xEA 0x8F
  if ((rLen == 3) && (resp[0] == 0xC1) && (resp[1] == 0xEA) && (resp[2] == 0x8F))
  {
    // Success, so send the Start Diag frame
    // 2 bytes: 0x10 0x80
    req[0] = 0x10;
    req[1] = 0x80;
    rLen = xECU_sendRequest(req, resp, 2, 3);

    // OK Response should be 2 bytes: 0x50 0x80
    if ((rLen == 2) && (resp[0] == 0x50) && (resp[1] == 0x80))
    {
      Serial.println("Good Reply");
      return true;
    }
  }
  // Otherwise, we failed to init.
  return false;
}

// Send a request to the ECU and wait for the response
// request = buffer to send
// response = buffer to hold the response
// reqLen = length of request
// maxLen = maximum size of response buffer
//
// Returns: number of bytes of response returned.
uint8_t ECU_sendRequest(const uint8_t *request, uint8_t *response, uint8_t reqLen, uint8_t maxLen)
{
  uint8_t buf[16], rbuf[16];
  uint8_t bytesToSend;
  uint8_t bytesSent = 0;
  uint8_t bytesToRcv = 0;
  uint8_t bytesRcvd = 0;
  uint8_t rCnt = 0;
  uint8_t c, z;
  bool forMe = false;
  char radioBuf[32];
  uint32_t startTime;

  for (uint8_t i = 0; i < 16; i++)
  {
    buf[i] = 0;
  }

  // Zero the response buffer up to maxLen
  for (uint8_t i = 0; i < maxLen; i++)
  {
    response[i] = 0;
  }

  // Form the request:
  if (reqLen == 1)
  {
    buf[0] = 0x81;
  }
  else
  {
    buf[0] = 0x80;
  }
  buf[1] = ECUaddr;
  buf[2] = myAddr;

  if (reqLen == 1)
  {
    buf[3] = request[0];
    buf[4] = xECU_calcChecksum(buf, 4);
    bytesToSend = 5;
  }
  else
  {
    buf[3] = reqLen;
    for (z = 0; z < reqLen; z++)
    {
      buf[4 + z] = request[z];
    }
    buf[4 + z] = xECU_calcChecksum(buf, 4 + z);
    bytesToSend = 5 + z;
  }

  // Now send the command...
  for (uint8_t i = 0; i < bytesToSend; i++)
  {
    bytesSent += xcpuSerial.write(buf[i]);
    delay(ISORequestByteDelay);
  }

  // Wait required time for response.
  // PLdelayLeds(ISORequestDelay, false);
  delay(ISORequestDelay);

  startTime = millis();

  // Wait for and deal with the reply
  while ((bytesRcvd <= maxLen) && ((millis() - startTime) < MAXSENDTIME))
  {
    if (xcpuSerial.available())
    {
      c = xcpuSerial.read();
      startTime = millis(); // reset the timer on each byte received

      // PLdelayLeds(ISORequestByteDelay, true);
      delay(ISORequestDelay);

      rbuf[rCnt] = c;
      switch (rCnt)
      {
      case 0:
        // should be an addr packet either 0x80 or 0x81
        if (c == 0x81)
        {
          bytesToRcv = 1;
        }
        else if (c == 0x80)
        {
          bytesToRcv = 0;
        }
        rCnt++;
        break;
      case 1:
        // should be the target address
        if (c == myAddr)
        {
          forMe = true;
        }
        rCnt++;
        break;
      case 2:
        // should be the sender address
        if (c == ECUaddr)
        {
          forMe = true;
        }
        else if (c == myAddr)
        {
          forMe = false; // ignore the packet if it came from us!
        }
        rCnt++;
        break;
      case 3:
        // should be the number of bytes, or the response if its a single byte packet.
        if (bytesToRcv == 1)
        {
          bytesRcvd++;
          if (forMe)
          {
            response[0] = c; // single byte response so store it.
          }
        }
        else
        {
          bytesToRcv = c; // number of bytes of data in the packet.
        }
        rCnt++;
        break;
      default:
        if (bytesToRcv == bytesRcvd)
        {
          // must be at the checksum...
          if (forMe)
          {
            // Only check the checksum if it was for us - don't care otherwise!
            if (xECU_calcChecksum(rbuf, rCnt) == rbuf[rCnt])
            {
              // Checksum OK.
              return (bytesRcvd);
            }
            else
            {
              // Checksum Error.
              return (0);
            }
          }
          // Reset the counters
          rCnt = 0;
          bytesRcvd = 0;

          // ISO 14230 specifies a delay between ECU responses.
          // PLdelayLeds(ISORequestDelay, true);
          delay(ISORequestDelay);
        }
        else
        {
          // must be data, so put it in the response buffer
          // rCnt must be >= 4 to be here.
          if (forMe)
          {
            response[bytesRcvd] = c;
          }
          bytesRcvd++;
          rCnt++;
        }
        break;
      }
    }
  }
  return bytesRcvd;
}

uint8_t ECU_calcChecksum(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;

  for (uint8_t i = 0; i < len; i++)
  {
    crc = crc + data[i];
  }
  return crc;
}
