#include <ssd1306.h>
void display_task(void *args);
int speed_read_reset(void);
void speed_kph(float speed);
void speed_pulse_lock(void);
void speed_pulse_unlock(void);
void speed_pulse(void);
void speed_lock(void);
void speed_unlock(void);
void distance_set(float s_distance);
void distance_lock(void);
void distance_unlock(void);
void Ticker(ssd1306_handle_t dev, int left, int top, int right, int bottom, bool on);
