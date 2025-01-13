#include "Arduino.h"
#include "def.h"
#include "serial.h"

static volatile uint8_t serialHeadRX[UART_NUMBER];
static volatile uint8_t serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER];
static volatile uint8_t serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];

void Serial2Open(uint32_t baud)
{
#ifdef PL_DEBUG
    Serial.print("Speed Open ");
    Serial.print(String(baud));
    Serial.print("\n");
#endif
    Serial2.setRxBufferSize(RXBUFFER);
    Serial2.setTxBufferSize(TXBUFFER);
    Serial2.begin(baud, SERIAL_8N1, RXD2, TXD2, false);
    Serial2.flush();
}
void Serial2Close()
{
    Serial2.flush(true);
    Serial2.end();
#ifdef PL_DEBUG
    Serial.println("Close Port");
#endif
}
bool lBuffferIsEmpty = false;
bool Serial2TXfree()
{
    /*
#ifdef PL_DEBUG
    if (!lBuffferIsEmpty)
    {
        log_d("Test");
        Serial.print("Write ");
        Serial.print(String(Serial2.availableForWrite()));
        Serial.print("\n");
        lBuffferIsEmpty = true;
    }
#endif

    return TXBUFFER - Serial2.availableForWrite();
    */
      if (Serial2.availableForWrite() == TXBUFFER) {
        log_d("TX buffer is empty.");
        return true; // Buffer is completely free
    } else {
        log_d("TX buffer is not empty.");
        return false; // Buffer still has data
    }
}

uint8_t Serial2Read(uint8_t port)
{
    uint8_t c = Serial2.read();
    return c;
}

uint8_t Serial2Available(uint8_t port)
{
    return ((uint8_t)(serialHeadRX[port] - serialTailRX[port])) % RX_BUFFER_SIZE;
}

uint8_t Serial2UsedTXBuff(uint8_t port)
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
void Serial2WriteGPS(uint8_t c)
{
    Serial2.write(c);
    Serial2.flush(true);
}

void Serial2WriteHex(uint8_t port, uint8_t c)
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

void Serial2WriteString(uint8_t port, const char *c)
{
    while (*c != '\0')
    {
        Serial2WriteGPS((uint8_t)(*c));
        c++;
    }
}