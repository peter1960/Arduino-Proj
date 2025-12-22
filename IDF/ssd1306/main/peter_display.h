#include <ssd1306.h>
void display_task(void *args);
void speed_mutex_init(void);
void speed_lock(void);
void speed_unlock(void);
void speed_set(float s_speed, float s_distance);
void Ticker(ssd1306_handle_t dev, int left, int top, int right, int bottom, bool on);
