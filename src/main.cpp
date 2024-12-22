#include <Arduino.h>
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = D0; // 16; // GPIO16
const int LOADCELL_SCK_PIN = D1;  // 5;   // GPIO5
const int LED_BLUE = D4;          // 2;
// STEPPER
const int STEP_PIN = D2;
const int DIR_PIN = D3;
const int SLEEP_PIN = D6; // 12;

HX711 scale;

#include <Bonezegei_A4988.h>
#define FORWARD 1
#define REVERSE 0

Bonezegei_A4988 stepper(DIR_PIN, STEP_PIN);

bool LED = false;

void xstep()
{
  float st = 1;
  for (int x = 0; x < 800; x++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delay((int)st);
    digitalWrite(STEP_PIN, LOW);
    delay((int)st);
    // st -= .01;
  }
}

void xstep_enable()
{
  digitalWrite(SLEEP_PIN, LOW);
}
void step_disable()
{
  digitalWrite(SLEEP_PIN, HIGH);
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BLUE, OUTPUT);
  // pinMode(STEP_PIN, OUTPUT);
  // pinMode(DIR_PIN, OUTPUT);
  // pinMode(SLEEP_PIN, OUTPUT);
  //   digitalWrite(DIR_PIN, HIGH);

  stepper.begin();
  stepper.setSpeed(1000);
  // step_disable();
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(-188489 / 170);
  scale.tare();
}

void loop()
{
  long reading = 100;
  /* Read value*/
  if (scale.wait_ready_retry(10))
  {
    reading = scale.get_units(10);
    // long reading = scale.read();
    Serial.print("HX711 reading: ");
    Serial.println(reading);
  }
  else
  {
    /* May not be ready*/
    // Serial.println("HX711 not found.");
  }

  if (LED)
  {
    digitalWrite(LED_BLUE, LOW);
    LED = false;
  }
  else
  {
    digitalWrite(LED_BLUE, HIGH);
    LED = true;
    if (reading < 60)
    {
      stepper.step(FORWARD, 2000);
      // stepper.disable();

      // stepper.rotate(360);
      //  step_enable();
      //  step();
      //  step_disable();
    }
  }
  delay(1500);
}
