#include "Arduino.h"
#include "def.h"
#include "serial.h"

static volatile uint8_t serialHeadRX[UART_NUMBER];
static volatile uint8_t serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER];
static volatile uint8_t serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];

void SerialOpen(uint32_t baud)
{
#ifdef PL_DEBUG
    Serial.print("Speed Open ");
    Serial.print(String(baud));
    Serial.print("\n");
#endif
    Serial2.setRxBufferSize(RXBUFFER);
    Serial2.setTxBufferSize(TXBUFFER);
    Serial2.begin(baud, SERIAL_8N1, RXD2, TXD2, false);
}
void SerialClose()
{
    Serial2.flush(true);
    Serial2.end();
#ifdef PL_DEBUG
    Serial.println("Close Port");
#endif
}
bool xxxx = false;
bool SerialTXfree()
{
#ifdef PL_DEBUG
    if (!xxxx)
    {
        log_d("Test");
        Serial.print("Write ");
        Serial.print(String(Serial2.availableForWrite()));
        Serial.print("\n");
        xxxx = true;
    }
#endif

    return TXBUFFER - Serial2.availableForWrite();
}

uint8_t SerialRead(uint8_t port)
{
    uint8_t c = Serial2.read();
    return c;
}

uint8_t SerialAvailable(uint8_t port)
{
    return ((uint8_t)(serialHeadRX[port] - serialTailRX[port])) % RX_BUFFER_SIZE;
}

uint8_t SerialUsedTXBuff(uint8_t port)
{
    return ((uint8_t)(serialHeadTX[port] - serialTailTX[port])) % TX_BUFFER_SIZE;
}
/*
void SerialSerialize(uint8_t port, uint8_t a)
{
    uint8_t t = serialHeadTX[port];
    if (++t >= TX_BUFFER_SIZE)
        t = 0;
    serialBufferTX[t][port] = a;
    serialHeadTX[port] = t;
}
*/
void SerialWriteGPS(uint8_t c)
{
    Serial2.write(c);
    Serial2.flush(true);
}

void SerialWriteHex(uint8_t port, uint8_t c)
{
    char hexBuffer[5]; // To hold the hex representation (e.g., "0x12\0")
    snprintf(hexBuffer, sizeof(hexBuffer), "0x%02X", (uint8_t)(c));
    // Send each character of the hexBuffer individually
    for (int i = 0; hexBuffer[i] != '\0'; i++)
    {
        // SerialWrite((uint8_t)(hexBuffer[i]));
    }
    // SerialWrite(' ');
}

void SerialWriteString(uint8_t port, const char *c)
{
    while (*c != '\0')
    {
        SerialWriteGPS((uint8_t)(*c));
        c++;
    }
}