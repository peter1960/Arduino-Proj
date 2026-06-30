#include <Arduino.h>
bool isRecord();
void recordISR();
void resetISR();
void wheelISR();
void addDistance(uint8_t distance);
uint32_t getDistance();
void addTripDistance(uint8_t distance);
uint32_t getTripDistance();
void resetTripDistance();
void loadDistance() ;

void saveTask(void *pvParameters);