#ifndef __CONTROLLER_APP_H__
#define __CONTROLLER_APP_H__

#include "stm32f4xx_hal.h"

extern char usb_send_buff[128];
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#define usb_print(fmt, ...)                      \
    do {                                          \
        snprintf(usb_send_buff, sizeof(usb_send_buff), fmt, ##__VA_ARGS__); \
        CDC_Transmit_FS((uint8_t *)(usb_send_buff), sizeof(usb_send_buff) - 1);             \
    } while (0)

void send_data_float(const float* values, uint8_t count);
void change_legend(uint8_t index, const char *str);
void change_title(const char *str);
void erase_graph(void);

void parse_piano(uint8_t *buf);
void get_pid_param(void);
void parse_command(char *cmd);

#endif