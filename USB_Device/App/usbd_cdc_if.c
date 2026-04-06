/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v3.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "stm32g0xx.h"
#include <string.h>

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
#define CDC_MIRROR_TX_QUEUE_SIZE 4096U
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

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
static uint8_t cdc_mirror_tx_queue[CDC_MIRROR_TX_QUEUE_SIZE];
static volatile uint16_t cdc_mirror_tx_head = 0U;
static volatile uint16_t cdc_mirror_tx_tail = 0U;
static volatile uint16_t cdc_mirror_tx_in_flight = 0U;

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
static void CDC_Mirror_Reset_FS(void);
static uint16_t CDC_Mirror_QueuedCount_FS(void);
static uint16_t CDC_Mirror_FreeSpace_FS(void);
static void CDC_Mirror_TryStartTx_FS(void);

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
  CDC_Mirror_Reset_FS();
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
  CDC_Mirror_Reset_FS();
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
  if ((Buf == NULL) || (Len == 0U))
  {
    return USBD_OK;
  }

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return USBD_BUSY;
  }

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc == NULL)
  {
    return USBD_BUSY;
  }

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
  uint32_t primask;

  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  primask = __get_PRIMASK();
  __disable_irq();

  if (cdc_mirror_tx_in_flight != 0U)
  {
    cdc_mirror_tx_tail = (uint16_t)((cdc_mirror_tx_tail + cdc_mirror_tx_in_flight) %
                                    CDC_MIRROR_TX_QUEUE_SIZE);
    cdc_mirror_tx_in_flight = 0U;
  }

  __set_PRIMASK(primask);

  CDC_Mirror_TryStartTx_FS();
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void CDC_Mirror_Write_FS(const uint8_t *Buf, uint16_t Len)
{
  uint16_t queued_len;
  uint16_t first_chunk;
  uint32_t primask;

  if ((Buf == NULL) || (Len == 0U))
  {
    return;
  }

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    CDC_Mirror_Reset_FS();
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  queued_len = CDC_Mirror_FreeSpace_FS();
  if (queued_len > Len)
  {
    queued_len = Len;
  }

  if (queued_len != 0U)
  {
    first_chunk = queued_len;
    if ((uint16_t)(CDC_MIRROR_TX_QUEUE_SIZE - cdc_mirror_tx_head) < first_chunk)
    {
      first_chunk = (uint16_t)(CDC_MIRROR_TX_QUEUE_SIZE - cdc_mirror_tx_head);
    }

    memcpy(&cdc_mirror_tx_queue[cdc_mirror_tx_head], Buf, first_chunk);

    if (queued_len > first_chunk)
    {
      memcpy(&cdc_mirror_tx_queue[0], &Buf[first_chunk], queued_len - first_chunk);
    }

    cdc_mirror_tx_head = (uint16_t)((cdc_mirror_tx_head + queued_len) %
                                    CDC_MIRROR_TX_QUEUE_SIZE);
  }

  __set_PRIMASK(primask);

  CDC_Mirror_TryStartTx_FS();
}

static void CDC_Mirror_Reset_FS(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  cdc_mirror_tx_head = 0U;
  cdc_mirror_tx_tail = 0U;
  cdc_mirror_tx_in_flight = 0U;

  __set_PRIMASK(primask);
}

static uint16_t CDC_Mirror_QueuedCount_FS(void)
{
  if (cdc_mirror_tx_head >= cdc_mirror_tx_tail)
  {
    return (uint16_t)(cdc_mirror_tx_head - cdc_mirror_tx_tail);
  }

  return (uint16_t)(CDC_MIRROR_TX_QUEUE_SIZE - cdc_mirror_tx_tail + cdc_mirror_tx_head);
}

static uint16_t CDC_Mirror_FreeSpace_FS(void)
{
  return (uint16_t)((CDC_MIRROR_TX_QUEUE_SIZE - 1U) - CDC_Mirror_QueuedCount_FS());
}

static void CDC_Mirror_TryStartTx_FS(void)
{
  USBD_CDC_HandleTypeDef *hcdc;
  uint16_t tx_len;
  uint8_t *tx_ptr;
  uint32_t primask;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    CDC_Mirror_Reset_FS();
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if ((hcdc == NULL) || (hcdc->TxState != 0U) ||
      (cdc_mirror_tx_in_flight != 0U) ||
      (cdc_mirror_tx_head == cdc_mirror_tx_tail))
  {
    __set_PRIMASK(primask);
    return;
  }

  tx_ptr = &cdc_mirror_tx_queue[cdc_mirror_tx_tail];
  if (cdc_mirror_tx_head > cdc_mirror_tx_tail)
  {
    tx_len = (uint16_t)(cdc_mirror_tx_head - cdc_mirror_tx_tail);
  }
  else
  {
    tx_len = (uint16_t)(CDC_MIRROR_TX_QUEUE_SIZE - cdc_mirror_tx_tail);
  }

  cdc_mirror_tx_in_flight = tx_len;

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_ptr, tx_len);
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
  {
    cdc_mirror_tx_in_flight = 0U;
  }

  __set_PRIMASK(primask);
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

