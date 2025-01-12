#ifndef SERIAL_H_
#define SERIAL_H_

#if defined(MEGA)
#define UART_NUMBER 4
#elif defined(PROMICRO)
#define UART_NUMBER 2
#else
#define UART_NUMBER 1
#endif
#define RX_BUFFER_SIZE 256 // 256 RX buffer is needed for GPS communication (64 or 128 was too short)
#define TX_BUFFER_SIZE 128

void Serial2Open(uint32_t baud);
void Serial2Close();
uint8_t Serial2Read(uint8_t port);
void Serial2WriteGPS(uint8_t c);
uint8_t Serial2Available(uint8_t port);
void SerialEnd(uint8_t port);
uint8_t SerialPeek(uint8_t port);
bool Serial2TXfree();
uint8_t Serial2UsedTXBuff(uint8_t port);
void SerialSerialize(uint8_t port, uint8_t a);
// void UartSendData(uint8_t port);

void SerialWrite16(uint8_t port, int16_t val);
//void SerialWriteStringGPS(const char *c);
void Serial2WriteHex(uint8_t port, uint8_t c);

#endif /* SERIAL_H_ */