#include <Arduino.h>

#define MAXSENDTIME 2000 // 2 second timeout on KDS comms.
const uint32_t ISORequestByteDelay = 10;
const uint32_t ISORequestDelay = 40; // Time between requests.
const uint8_t ECUaddr = 0x11;
const uint8_t myAddr = 0xF2;
const uint8_t validRegs[] = {0x00, 0x01, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08,
                             0x09, 0x0A, 0x0B, 0x0C, 0x20, 0x27, 0x28, 0x29, 0x2A, 0x2E, 0x31, 0x32,
                             0x33, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x44, 0x54, 0x56, 0x5B, 0x5C, 0x5D,
                             0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x6E,
                             0x6F, 0x80, 0x9B, 0xA0, 0xB4};

const uint8_t numValidRegs = (uint8_t)(sizeof(validRegs));
bool initPulse();
uint8_t sendRequest(const uint8_t *request, uint8_t *response, uint8_t reqLen, uint8_t maxLen);
uint8_t calcChecksum(uint8_t *data, uint8_t len);
float speed();
float RPM();
bool alive();