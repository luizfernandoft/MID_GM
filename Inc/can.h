/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

typedef struct
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint32_t timestamp_ms;
} CAN_RxFrame_t;

typedef struct
{
  /* status */
  uint8_t  initialized;
  uint8_t  bus_off;
  uint8_t  error_passive;
  uint8_t  error_warning;

  /* baud */
  uint32_t baudrate;

  /* error stats */
  uint32_t err_bus_off_cnt;
  uint32_t err_passive_cnt;
  uint32_t err_warning_cnt;
  uint32_t err_total_cnt;
  uint32_t last_error_code;

  /* traffic stats */
  uint32_t rx_frames;
  uint32_t tx_frames;

  /* busload */
  float busload_percent;

} CAN_Status_t;


/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

void CAN_ConfigFilter_All(void);
HAL_StatusTypeDef CAN_SetBaudrate(uint32_t baudrate);
uint8_t CAN_AutoBaud(void);
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data, uint8_t len, uint8_t ext);
uint8_t CAN_RxAvailable(void);
uint8_t CAN_RxPop(CAN_RxFrame_t *frame);
void CAN_CheckAndRecover(void);
void CAN_UpdateBusload(uint32_t bits);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

