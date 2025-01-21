#include <arduino.h>

#ifndef KLINE_H_
#define KLINE_H_


bool fastInit();
bool keepAlive();
int16_t ECU_RPM();
uint8_t sendRequest(const uint8_t *request, uint8_t *response, uint8_t reqLen, uint8_t maxLen);
uint8_t calcChecksum(uint8_t *data, uint8_t len);
void ErrorAppeard();

#endif