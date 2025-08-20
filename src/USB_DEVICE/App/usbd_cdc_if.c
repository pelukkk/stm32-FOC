/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include "FOC_utils.h"
#include "flash.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

#define usb_print(str)  CDC_Transmit_FS((uint8_t *)(str), sizeof(str) - 1)

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern float sp_input;

extern foc_t hfoc;

extern int note_piano[];

extern int start_cal;
extern _Bool com_init_flag;
extern _Bool calibration_flag;
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

#define CDC_TIMEOUT_MS 100  // Timeout dalam milidetik

uint8_t CDC_Transmit_Blocking(uint8_t* buf, uint16_t len) {
  uint32_t start = HAL_GetTick();

  while (CDC_Transmit_FS(buf, len) == USBD_BUSY) {
    if ((HAL_GetTick() - start) > CDC_TIMEOUT_MS) {
      return USBD_FAIL;  // Timeout
    }
  }
  return USBD_OK;
}

int parse_float_value(const char *input, char separator, float *out_value) {
    if (!input || !out_value) return 0;

    // Cari posisi separator
    const char *sep_pos = strchr(input, separator);
    if (!sep_pos) return 0;

    // Lompat ke karakter setelah separator
    const char *val_str = sep_pos + 1;

    // Lewati whitespace
    while (isspace((unsigned char)*val_str)) {
        val_str++;
    }

    // Cek apakah setelah spasi masih ada karakter valid
    if (*val_str == '\0') return 0;

    // Parsing float
    char *endptr;
    float value = strtof(val_str, &endptr);

    if (val_str == endptr) {
        // Tidak ada konversi yang terjadi
        return 0;
    }

    *out_value = value;
    return 1;
}

int parse_int_value(const char *input, char separator, int *out_value) {
    if (!input || !out_value) return 0;

    const char *sep_pos = strchr(input, separator);
    if (!sep_pos) return 0;

    const char *val_str = sep_pos + 1;

    while (isspace((unsigned char)*val_str)) {
        val_str++;
    }

    if (*val_str == '\0') return 0;

    char *endptr;
    long value = strtol(val_str, &endptr, 10);

    // Validasi: endptr harus berada di akhir angka atau hanya whitespace
    while (isspace((unsigned char)*endptr)) {
        endptr++;
    }
    if (*endptr != '\0') return 0; // Ada karakter tidak valid setelah angka

    *out_value = (int)value;
    return 1;
}

void parse_piano(uint8_t *buf) {
	int tone = buf[1] - 1;
	if (tone < 0) tone = 0;

	if (buf[0] == 0xD3) note_piano[tone] = 1;
	else if (buf[0] == 0xD2) note_piano[tone] = 0;
}

void set_pid_param(void) {
  char write_buffer[64];
  uint16_t len = 0;

  switch (hfoc.control_mode) {
  case TORQUE_CONTROL_MODE:
    len = sprintf(write_buffer, "Current Control param:\n"
                                "Kp:%f\n"
                                "Ki:%f\n", 
                                hfoc.id_ctrl.kp, hfoc.id_ctrl.ki);
    break;
  case SPEED_CONTROL_MODE:
    len = sprintf(write_buffer, "Speed Control param:\n"
                                "Kp:%f\n"
                                "Ki:%f\n", 
                                hfoc.speed_ctrl.kp, hfoc.speed_ctrl.ki);
    break;
  case POSITION_CONTROL_MODE:
    len = sprintf(write_buffer, "Position Control param:\n"
                                "Kp:%f\n"
                                "Ki:%f\n"
                                "Kd:%f\n", 
                                hfoc.pos_ctrl.kp, hfoc.pos_ctrl.ki, hfoc.pos_ctrl.kd);
    break;
  default:
    len = sprintf(write_buffer, "Wrong Mode\n");
    break;
  }
  CDC_Transmit_FS((uint8_t*)write_buffer, len);
}

void print_mode(motor_mode_t mode) {
  char write_buffer[64];
  uint16_t len = 0;

  switch (mode) {
  case TORQUE_CONTROL_MODE:
    len = sprintf(write_buffer, "Control Mode[0]: Current Control\n");
    break;
  case SPEED_CONTROL_MODE:
    len = sprintf(write_buffer, "Control Mode[1]: Speed Control\n");
    break;
  case POSITION_CONTROL_MODE:
    len = sprintf(write_buffer, "Control Mode[2]: Position Control\n");
    break;
  case CALIBRATION_MODE:
    len = sprintf(write_buffer, "Control Mode[3]: calibration\n");
    break;
  default:
    len = sprintf(write_buffer, "Wrong Mode\n");
    break;
  }
  CDC_Transmit_FS((uint8_t*)write_buffer, len);
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */

	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);

	_Bool set_pid_detected = 0;
	Buf[*Len] = '\0';

	char *cmd = (char *)Buf;

  // init flag
  if (Buf[0] == 0x01 && Buf[1] == 0xAA) {
    com_init_flag = 1;
  }

	if (strstr(cmd, "cal")) {
    usb_print("start callibration\r\n");
    hfoc.control_mode = CALIBRATION_MODE;
    calibration_flag = 1;
  }
	else if (strstr(cmd, "sp")) {
		parse_float_value(cmd, '=', &sp_input);
	}
	else if (strstr(cmd, "kp")) {
    float kp;
		parse_float_value(cmd, '=', &kp);

    switch (hfoc.control_mode) {
    case TORQUE_CONTROL_MODE:
      hfoc.id_ctrl.kp = kp;
      m_config.id_kp = kp;
      hfoc.iq_ctrl.kp = kp;
      m_config.iq_kp = kp;
      break;
    case SPEED_CONTROL_MODE:
      hfoc.speed_ctrl.kp = kp;
      m_config.speed_kp = kp;
      break;
    case POSITION_CONTROL_MODE:
      hfoc.pos_ctrl.kp = kp;
      m_config.pos_kp = kp;
      break;
    default:
      break;
    }
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "ki")) {
    float ki;
		parse_float_value(cmd, '=', &ki);
    
    switch (hfoc.control_mode) {
    case TORQUE_CONTROL_MODE:
      hfoc.id_ctrl.ki = ki;
      m_config.id_ki = ki;
      hfoc.iq_ctrl.ki = ki;
      m_config.iq_ki = ki;
      break;
    case SPEED_CONTROL_MODE:
      hfoc.speed_ctrl.ki = ki;
      m_config.speed_ki = ki;
      break;
    case POSITION_CONTROL_MODE:
      hfoc.pos_ctrl.ki = ki;
      m_config.pos_ki = ki;
      break;
    default:
      break;
    }
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "kd")) {
    float kd;
		parse_float_value(cmd, '=', &kd);
    
    switch (hfoc.control_mode) {
    case POSITION_CONTROL_MODE:
      hfoc.pos_ctrl.kd = kd;
      m_config.pos_kd = kd;
      break;
    default:
      break;
    }
		set_pid_detected = 1;
	}
	else if (strstr(cmd, "mode") != NULL) {
    motor_mode_t mode = TORQUE_CONTROL_MODE;

		parse_int_value(cmd, '=', (int*)&mode);
    print_mode(mode);

    if (mode <= CALIBRATION_MODE) {
      hfoc.control_mode = mode;
      sp_input = 0;
    }
	}
  else if (strstr(cmd, "save") != NULL) {
    save_config_to_flash(&m_config);
    usb_print("success save configuration\r\n");
  }
  else if (strstr(cmd, "set_default") != NULL) {
    default_config(&m_config);
    usb_print("success reset configuration\r\n");
  }

	if (set_pid_detected) {
    set_pid_param();
	}

	if (*Len == 2) {
		parse_piano(Buf);
	}

	return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
