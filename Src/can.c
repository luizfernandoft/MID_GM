#include "can.h"

/* ================= CONFIG ================= */

#define CAN_RX_BUFFER_SIZE       32
#define CAN_MONITOR_WINDOW_MS  1000
#define CAN_RECOVERY_DELAY_MS   100

/* ================= TYPES ================= */

/* ================= GLOBALS ================= */

CAN_HandleTypeDef hcan;
CAN_Status_t CAN_Status;

/* RX circular buffer */
static CAN_RxFrame_t rxBuffer[CAN_RX_BUFFER_SIZE];
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;

/* busload calc */
static uint32_t busload_start_ms = 0;
static uint32_t busload_bits_acc = 0;

/* ================= PROTOTYPES ================= */


/* ================= INIT ================= */

void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.AutoRetransmission = ENABLE;

  CAN_SetBaudrate(125000);
  CAN_ConfigFilter_All();

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,
    CAN_IT_RX_FIFO0_MSG_PENDING |
    CAN_IT_BUSOFF |
    CAN_IT_ERROR_WARNING |
    CAN_IT_ERROR_PASSIVE |
    CAN_IT_LAST_ERROR_CODE);

  CAN_Status.initialized = 1;
  busload_start_ms = HAL_GetTick();
}

/* ================= FILTER ================= */

void CAN_ConfigFilter_All(void)
{
  CAN_FilterTypeDef f = {0};

  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &f);
}

/* ================= BAUD ================= */

HAL_StatusTypeDef CAN_SetBaudrate(uint32_t baudrate)
{
  HAL_CAN_Stop(&hcan);
  HAL_CAN_DeInit(&hcan);

  switch (baudrate)
  {
    case 125000:
      hcan.Init.Prescaler = 18;
      hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
      hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
      break;

    case 250000:
      hcan.Init.Prescaler = 9;
      hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
      hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
      break;

    case 500000:
      hcan.Init.Prescaler = 6;
      hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
      hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
      break;

    case 1000000:
      hcan.Init.Prescaler = 3;
      hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
      hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
      break;

    default:
      return HAL_ERROR;
  }

  if (HAL_CAN_Init(&hcan) != HAL_OK)
    return HAL_ERROR;

  CAN_Status.baudrate = baudrate;
  return HAL_OK;
}

/* ================= SEND ================= */

HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data, uint8_t len, uint8_t ext)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mailbox;

  if (len > 8) return HAL_ERROR;
  if (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)) return HAL_BUSY;

  tx.DLC = len;
  tx.RTR = CAN_RTR_DATA;
  tx.IDE = ext ? CAN_ID_EXT : CAN_ID_STD;
  if (ext) tx.ExtId = id;
  else     tx.StdId = id;

  if (HAL_CAN_AddTxMessage(&hcan, &tx, data, &mailbox) == HAL_OK)
  {
    CAN_Status.tx_frames++;
    CAN_UpdateBusload(128); // frame médio
    return HAL_OK;
  }
  return HAL_ERROR;
}

/* ================= RX BUFFER ================= */

uint8_t CAN_RxAvailable(void)
{
  return rxHead != rxTail;
}

uint8_t CAN_RxPop(CAN_RxFrame_t *frame)
{
  if (rxHead == rxTail) return 0;

  *frame = rxBuffer[rxTail];
  rxTail = (rxTail + 1) % CAN_RX_BUFFER_SIZE;
  return 1;
}

/* ================= RX CALLBACK ================= */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
  CAN_RxFrame_t *f = &rxBuffer[rxHead];

  if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &f->header, f->data) == HAL_OK)
  {
    f->timestamp_ms = HAL_GetTick();
    CAN_Status.rx_frames++;
    CAN_UpdateBusload(128);

    uint8_t next = (rxHead + 1) % CAN_RX_BUFFER_SIZE;
    if (next != rxTail)
      rxHead = next;
  }
}

/* ================= ERROR CALLBACK ================= */

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h)
{
  uint32_t err = HAL_CAN_GetError(h);

  CAN_Status.last_error_code = err;
  CAN_Status.err_total_cnt++;

  if (err & HAL_CAN_ERROR_BOF)
  {
    CAN_Status.bus_off = 1;
    CAN_Status.err_bus_off_cnt++;
  }

  if (err & HAL_CAN_ERROR_EPV)
  {
    CAN_Status.error_passive = 1;
    CAN_Status.err_passive_cnt++;
  }

  if (err & HAL_CAN_ERROR_EWG)
  {
    CAN_Status.error_warning = 1;
    CAN_Status.err_warning_cnt++;
  }
}

/* ================= BUSLOAD ================= */

void CAN_UpdateBusload(uint32_t bits)
{
  busload_bits_acc += bits;

  uint32_t now = HAL_GetTick();
  if ((now - busload_start_ms) >= CAN_MONITOR_WINDOW_MS)
  {
    float max_bits = (float)CAN_Status.baudrate;
    CAN_Status.busload_percent =
      (busload_bits_acc / max_bits) * 100.0f;

    busload_bits_acc = 0;
    busload_start_ms = now;
  }
}

/* ================= RECOVERY ================= */

void CAN_CheckAndRecover(void)
{
  if (CAN_Status.bus_off)
  {
    HAL_Delay(CAN_RECOVERY_DELAY_MS);
    HAL_CAN_Stop(&hcan);
    HAL_CAN_Start(&hcan);
    CAN_Status.bus_off = 0;
  }
}
