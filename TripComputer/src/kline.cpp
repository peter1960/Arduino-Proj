#include <arduino.h>
#include <def.h>
#include "SoftwareSerial.h"
#include <serial.h>
#include <kline.h>

uint8_t format = 0x81;
bool ECUconnected = false;
const uint8_t ISORequestByteDelay = 10;
const uint8_t ISORequestDelay = 40; // Time between requests.
uint8_t ecuResponse[12];
uint32_t lastKresponse;
uint8_t translatedSID = 0x21;
const uint8_t ECUaddr = 0x11;
const uint8_t MyAddr = 0xF1;
#define MAXSENDTIME 2000 // 2 second timeout on KDS comms.

EspSoftwareSerial::UART cpuSerial;
constexpr uint32_t CPUBPS = 10400;
char gear = 'N';

int16_t ECU_RPM()
{
    uint8_t rLen;
    uint8_t req[2];
    uint8_t resp[4];
    req[0] = 0x21;
    req[1] = 0x09  ;
    rLen = sendRequest(req, resp, 2, 4);
    float _speed = ((resp[2] * 100) + resp[3]);
    /*
    Serial.print("RPM Ask - > Len ");
    Serial.print(rLen);
    Serial.print(" ");
    */
    if (rLen == 0){
        ECUconnected = false;
        return -2;
    }
    /*
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
    */
    return _speed;
}

int16_t ECU_speed()
{
    uint8_t rLen;
    uint8_t req[2];
    uint8_t resp[4];
    req[0] = 0x21;
    req[1] = 0x0C;
    rLen = sendRequest(req, resp, 2, 4);
    float _speed = ((resp[2] * 100) + resp[3])/2.0f;
    
    Serial.print("Speed Ask - > Len ");
    Serial.print(rLen);
    Serial.print(" ");
    
    if (rLen == 0){
        ECUconnected = false;
        return -2;
    }
    
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


bool fastInit()
{
    uint8_t rLen;
    uint8_t req[2];
    uint8_t resp[3];

    cpuSerial.end();

    // This is the ISO 14230-2 "Fast Init" sequence.
    digitalWrite(K_OUT, HIGH);
    delay(300);
    digitalWrite(K_OUT, LOW);
    delay(25);
    digitalWrite(K_OUT, HIGH);
    delay(25);
    // Should be 10417
    cpuSerial.begin(CPUBPS, SWSERIAL_8N1, K_IN, K_OUT, false);

    // Start Communication is a single byte "0x81" packet.
    // 81 means Format without Header Information (Sender / Receiver)
    format = 0x81;
    req[0] = 0x81;
    rLen = sendRequest(req, resp, 1, 3);
    delay(ISORequestDelay);
    // Response should be 3 bytes: 0xC1 0xEA 0x8F
    if ((rLen == 3) && (resp[0] == 0xC1) && (resp[1] == 0xEA) && (resp[2] == 0x8F))
    {
        // Reset Format to Header:
        format = 0x80;
        // Success, so send the Start Diag frame
        // 2 bytes: 0x10 0x80
        req[0] = 0x10;
        req[1] = 0x80;
        rLen = sendRequest(req, resp, 2, 3);
        // OK Response should be 2 bytes: 0x50 0x80
        if ((rLen == 2) && (resp[0] == 0x50) && (resp[1] == 0x80))
        {
            // digitalWrite(BOARD_LED, HIGH);
            ECUconnected = true;
            return true;
        }
    }
    // Otherwise, we failed to init.
    // digitalWrite(BOARD_LED, LOW);
    ECUconnected = false;
    Serial.println("Start Failed, wait 2 secs");
    delay(2000);
    return false;
}
bool stopComm()
{
    uint8_t rLen;
    uint8_t req[2];
    uint8_t resp[3];

    // Stop Diag frame
    // 2 bytes: 0x20 0x80
    req[0] = 0x20;
    req[1] = 0x80;
    rLen = sendRequest(req, resp, 2, 3);
    // OK Response should be 2 bytes: 0x60 0x80
    if ((rLen != 2) || (resp[0] != 0x60) || (resp[1] != 0x80))
    {
        // return false;
    }
    // Stop Communication is a single byte "0x82" packet.
    format = 0x80;
    req[0] = 0x82;
    rLen = sendRequest(req, resp, 1, 3);
    ECUconnected = false;
    return true;
}

bool processRequest(uint8_t pid)
{
    uint8_t cmdSize;
    uint8_t cmdBuf[6];
    uint8_t resultBufSize;

    cmdSize = 2;

    // Zero the response buffer up to maxLen
    for (uint8_t i = 0; i < 12; i++)
    {
        ecuResponse[i] = 0;
    }

    if (ECUconnected)
    {
        // Status:
        // cmdBuf[0] = 0x21;
        cmdBuf[0] = translatedSID;
        // PID
        cmdBuf[1] = pid;

        // resultBufSize enthaellt die länge der antwort der ECU
        resultBufSize = sendRequest(cmdBuf, ecuResponse, cmdSize, 12);

        // Buffer is empty?
        return resultBufSize > 0;
    }
    return false;
}

// Tom Mitchell https://bitbucket.org/tomnz/kawaduino
// (Kawaduino)
// https://www.youtube.com/watch?v=ie-Pfxzt-yQ
uint8_t sendRequest(const uint8_t *request, uint8_t *response, uint8_t reqLen, uint8_t maxLen)
{
    // Send a request to the ECU and wait for the response
    // request = buffer to send
    // response = buffer to hold the response
    // reqLen = length of request
    // maxLen = maximum size of response buffer
    //
    // Returns: number of bytes of response returned.

    uint8_t buf[16], rbuf[16];
    uint8_t bytesToSend;
    uint8_t bytesSent = 0;
    uint8_t bytesToRcv = 0;
    uint8_t bytesRcvd = 0;
    uint8_t rCnt = 0;
    uint8_t c, z;
    bool forMe = false;
    // char radioBuf[32];
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
    //  if (reqLen == 1)
    //  {
    //    buf[0] = 0x81;
    //  }
    //  else
    //  {
    //    buf[0] = 0x80;
    //  }
    // Request 80 can also have a single byte!
    buf[0] = format;

    buf[1] = ECUaddr;
    buf[2] = MyAddr;

    // Ignore Length on Format 81
    // if (reqLen == 1)
    if (format == 0x81)
    {
        buf[3] = request[0];
        buf[4] = calcChecksum(buf, 4);
        bytesToSend = 5;
    }
    else
    {
        buf[3] = reqLen;
        for (z = 0; z < reqLen; z++)
        {
            buf[4 + z] = request[z];
        }
        buf[4 + z] = calcChecksum(buf, 4 + z);
        bytesToSend = 5 + z;
    }

    // Now send the command...
    for (uint8_t i = 0; i < bytesToSend; i++)
    {
        bytesSent += cpuSerial.write(buf[i]);
        delay(ISORequestByteDelay);
    }
    // Wait required time for response.
    // as no LEDs do standard delay
    // delayLeds(ISORequestDelay, false);
    delay(ISORequestDelay);
    startTime = millis();

    // Wait for and deal with the reply
    while ((bytesRcvd <= maxLen) && ((millis() - startTime) < MAXSENDTIME))
    {
        if (cpuSerial.available())
        {
            c = cpuSerial.read();
            startTime = millis(); // reset the timer on each byte received

            // delayLeds(ISORequestByteDelay, true);
            delay(ISORequestByteDelay);
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
                if (c == MyAddr)
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
                else if (c == MyAddr)
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
                        if (calcChecksum(rbuf, rCnt) == rbuf[rCnt])
                        {
                            lastKresponse = millis();
                            // Checksum OK.
                            return (bytesRcvd);
                        }
                        else
                        {
                            // ToDo:
                            //  Checksum Error.
                            return (0);
                        }
                    }
                    // Reset the counters
                    rCnt = 0;
                    bytesRcvd = 0;

                    // ISO 14230 specifies a delay between ECU responses.
                    // as we have no leds, imply standard delay
                    // delayLeds(ISORequestDelay, true);
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
    return false;
}

// Checksum is simply the sum of all data bytes modulo 0xFF
// (same as being truncated to one byte)
uint8_t calcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        crc = crc + data[i];
    }
    return crc;
}

void ErrorAppeard()
{
    ECUconnected = false;
    // ECU ignores requests for 2 seconds, after error appeard
    delay(2000);
}

bool keepAlive()
{
    if (ECUconnected)
    {
        // ToDo: Replace with "Tester Present"
        // 0x3E (SID)

        // Check Gear to keep KDS alive
        if (processRequest(0x0B))
        {
            if (ecuResponse[2] == 0x00)
            {
                gear = 'N';
            }
            else
            {
                gear = String(ecuResponse[2])[0];
            }
            return true;
        }
        // Error (Unconnected)
        if (ecuResponse[0] == 0x7F && ecuResponse[1] == 0x21 && ecuResponse[2] == 0x10)
        {
            // Error responded
            ErrorAppeard();
            return false;
        }
    }
    return false;
}