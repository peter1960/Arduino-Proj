#include <Arduino.h>
bool isRecord();
void recordISR();
void resetISR();
void wheelISR();
void addDistance(int16_t distance);
int16_t getDistance();
void addTripDistance(int16_t distance);
int16_t getTripDistance();
void resetTripDistance();
void loadDistance() ;

void saveTask(void *pvParameters);