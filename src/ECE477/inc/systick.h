#ifndef __SYSTICK_H__
#define __SYSTICK_H__

void init_SYSTICK(void);
void systick_clear_watchdog();
uint32_t millis();
uint32_t ahb_clock_ticks();
float ftime();
void nano_wait(uint32_t count);
void wait(float seconds);

#endif /* __SYSTICK_H__ */
