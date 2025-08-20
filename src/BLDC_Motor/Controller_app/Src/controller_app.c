#include "controller_app.h"
#include <stdio.h>
#include <string.h>

void send_data_float(const float* values, uint8_t count) {
  if (count == 0 || values == NULL) return;

  // Total frame: 2 byte header + 4*count float + 2 byte footer
  uint16_t total_size = 2 + count * 4 + 2;
  uint8_t frame[256];

  if (total_size > sizeof(frame)) return;

  // Header: 0x55, 0xAA
  frame[0] = 0x55;
  frame[1] = 0xAA;

  // Copy floats
  for (uint8_t i = 0; i < count; i++) {
    union { float f; uint8_t b[4]; } u;
    u.f = values[i];
    frame[2 + i * 4 + 0] = u.b[0];
    frame[2 + i * 4 + 1] = u.b[1];
    frame[2 + i * 4 + 2] = u.b[2];
    frame[2 + i * 4 + 3] = u.b[3];
  }

  // Footer: 0xAA, 0x55
  frame[2 + count * 4 + 0] = 0xAA;
  frame[2 + count * 4 + 1] = 0x55;

  // Transmit
  CDC_Transmit_FS(frame, total_size);
}

void change_legend(uint8_t index, const char *str) {
  uint8_t frame[64];
  uint16_t total_size = 0;

  frame[0] = 0x03;
  frame[1] = index;  

  size_t len = strlen(str);
  if (len > sizeof(frame) - 2) len = sizeof(frame) - 2;
  memcpy(&frame[2], str, len);

  total_size = 2 + len;

  CDC_Transmit_FS(frame, total_size);
}

void change_title(const char *str) {
  uint8_t frame[64];
  uint16_t total_size = 0;

  frame[0] = 0x04;
  frame[1] = 0x00;  

  size_t len = strlen(str);
  if (len > sizeof(frame) - 2) len = sizeof(frame) - 2;
  memcpy(&frame[2], str, len);

  total_size = 2 + len;

  CDC_Transmit_FS(frame, total_size);
}

void erase_graph(void) {
  uint8_t frame[2];
  frame[0] = 0x02;
  frame[1] = 0x00;  
  CDC_Transmit_FS(frame, 2);
}
