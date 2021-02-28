#ifndef _ZTASK_H
#define _ZTASK_H

#define ZT_MAX_TASKS 12

typedef void(*zt_func_t)(void);

// should be called in main loop
void zt_poll(void);

// timeout: repeat inteval;   en: start immediately or not
int zt_bind(zt_func_t func, int repeat, int en);

// should be called in systick_irqhandler
void zt_tick(void);

void zt_start(int id, int reset);
void zt_stop(int id);
void zt_setInterval(int id, int interval);
void zt_reset(int id);

#endif
