#ifndef __CONTROLLER_APP_H__
#define __CONTROLLER_APP_H__

#include "stm32f4xx_hal.h"

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void send_data_float(const float* values, uint8_t count);
void change_legend(uint8_t index, const char *str);
void change_title(const char *str);
void erase_graph(void);

#endif