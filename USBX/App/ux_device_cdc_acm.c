/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device CDC ACM applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "om.h"
#include "crc.hpp"
#include "magicmsgs.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;
UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint32_t UserRxBufPtrIn;
uint32_t UserRxBufPtrOut;

uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

volatile UINT USB_TX_BUSY;
volatile UINT USB_TX_SUCCESS;
volatile UINT USB_RX_SUCCESS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;

  /* Set device class_cdc_acm with default parameters */
  if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                    &CDC_VCP_LineCoding) != UX_SUCCESS)
  {
    Error_Handler();
  }
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  ULONG request;
  UX_SLAVE_TRANSFER *transfer_request;
  UX_SLAVE_DEVICE *device;

  /* Get the pointer to the device */
  device = &_ux_system_slave -> ux_system_slave_device;

  /* Get the pointer to the transfer request associated with the control endpoint */
  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request)
  {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING :

      /* Get the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      /* Check if baudrate < 9600) then set it to 9600 */
      if (CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate < 9600)
      {
        CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate = 9600;
      }
      break;

    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING :

      /* Set the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE :
    default :
      break;
  }
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 2 */
uint32_t new_data_ = 0;

typedef struct
{
  volatile ULONG thread_input;
  volatile ULONG entry_count;
  volatile ULONG loop_count;
  volatile ULONG configured_count;
  volatile ULONG not_configured_count;
  volatile ULONG before_transfer_count;
  volatile ULONG after_transfer_count;
  volatile ULONG sleep_count;
  volatile ULONG device_state;
  volatile ULONG cdc_acm_ptr;
  volatile ULONG actual_length;
  volatile UINT transfer_status;
  volatile UINT phase;
} USBX_CDC_ACM_ThreadWatch_t;

typedef struct
{
  USBX_CDC_ACM_ThreadWatch_t read;
  USBX_CDC_ACM_ThreadWatch_t write;
} USBX_CDC_ACM_Watch_t;

volatile USBX_CDC_ACM_Watch_t usbx_cdc_acm_watch;

struct msg_visionrx_t debug_visionrx;
/**
  * @brief  Function implementing USBX_DEVICE_CDC_ACM_Read_TASK.
  * @param  thread_input: Not used.
  * @retval none
  */
VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input)
{
  ULONG actual_length;
  UINT read_status;
  UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;

  UX_PARAMETER_NOT_USED(thread_input);
  usbx_cdc_acm_watch.read.thread_input = thread_input;
  usbx_cdc_acm_watch.read.entry_count++;
  usbx_cdc_acm_watch.read.phase = 1;

  struct msg_visionrx_t msg_visionrx;

  om_topic_t *visionrx_topic = om_config_topic(NULL, "ca", "visionrx", sizeof(msg_visionrx));
  // tx_thread_sleep(TX_WAIT_FOREVER);
  while (1)
  {
    usbx_cdc_acm_watch.read.loop_count++;
    usbx_cdc_acm_watch.read.device_state = device->ux_slave_device_state;
    usbx_cdc_acm_watch.read.cdc_acm_ptr = (ULONG)cdc_acm;
    usbx_cdc_acm_watch.read.phase = 2;

    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
    {
      usbx_cdc_acm_watch.read.configured_count++;
      usbx_cdc_acm_watch.read.phase = 3;
      // cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
      actual_length = 0;
      usbx_cdc_acm_watch.read.before_transfer_count++;
      usbx_cdc_acm_watch.read.phase = 4;
      read_status = ux_device_class_cdc_acm_read(cdc_acm,
                                                 (UCHAR *)UserRxBufferFS,
                                                 64, &actual_length);
      usbx_cdc_acm_watch.read.transfer_status = read_status;
      usbx_cdc_acm_watch.read.actual_length = actual_length;
      usbx_cdc_acm_watch.read.after_transfer_count++;
      usbx_cdc_acm_watch.read.phase = 5;

      if (actual_length >= sizeof(msg_visionrx))
      {
        memcpy(&msg_visionrx, (UCHAR *)UserRxBufferFS, sizeof(msg_visionrx));
        memcpy(&debug_visionrx, (UCHAR *)UserRxBufferFS, sizeof(msg_visionrx));

      }
        // tx_thread_sleep(1); //测试过有没有这个1都能正常收发
    }
    else
    {
      usbx_cdc_acm_watch.read.not_configured_count++;
      usbx_cdc_acm_watch.read.phase = 10;
    }
    usbx_cdc_acm_watch.read.phase = 6;
    om_publish(visionrx_topic, &msg_visionrx, sizeof(msg_visionrx), true, false);
    usbx_cdc_acm_watch.read.sleep_count++;
    tx_thread_sleep(2);

  }
}


struct msg_visiontx_t msg_visiontx;
struct msg_visiontx_t debug_visiontx;
struct logger_t msg_log;
struct logger_t debug_log;

/**
  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input) 
{
  ULONG actual_length;
  UINT write_status;
  UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;

  om_suber_t *visiontx_suber;
  om_suber_t *log_suber;

  UX_PARAMETER_NOT_USED(thread_input);
  usbx_cdc_acm_watch.write.thread_input = thread_input;
  usbx_cdc_acm_watch.write.entry_count++;
  usbx_cdc_acm_watch.write.phase = 1;
  usbx_cdc_acm_watch.write.phase = 11;
  visiontx_suber = om_subscribe(om_find_topic("visiontx",UINT32_MAX));
  log_suber = om_subscribe(om_find_topic("log",UINT32_MAX));
  usbx_cdc_acm_watch.write.phase = 12;
  tx_thread_sleep(10);
  while (1)
  {
    usbx_cdc_acm_watch.write.loop_count++;
    usbx_cdc_acm_watch.write.device_state = device->ux_slave_device_state;
    usbx_cdc_acm_watch.write.cdc_acm_ptr = (ULONG)cdc_acm;
    usbx_cdc_acm_watch.write.phase = 2;

    om_suber_export(visiontx_suber, &msg_visiontx, false);
    om_suber_export(log_suber, &msg_log, false);
    usbx_cdc_acm_watch.write.phase = 3;
    tx_thread_sleep(1);
    usbx_cdc_acm_watch.write.sleep_count++;
    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
    {
      usbx_cdc_acm_watch.write.configured_count++;
      Append_CRC16_Check_Sum((uint8_t *)&msg_visiontx, sizeof(msg_visiontx));

      memcpy(&debug_visiontx, &msg_visiontx, sizeof(msg_visiontx));

      actual_length = 0;
      usbx_cdc_acm_watch.write.before_transfer_count++;
      usbx_cdc_acm_watch.write.phase = 4;
      write_status = ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)&msg_visiontx , sizeof(msg_visiontx) , &actual_length);
      usbx_cdc_acm_watch.write.transfer_status = write_status;
      usbx_cdc_acm_watch.write.actual_length = actual_length;
      usbx_cdc_acm_watch.write.after_transfer_count++;
      usbx_cdc_acm_watch.write.phase = 5;

      Append_CRC16_Check_Sum((uint8_t *)&msg_log, sizeof(msg_log));
      memcpy(&debug_log, &msg_log, sizeof(msg_log));

      actual_length = 0;
      usbx_cdc_acm_watch.write.before_transfer_count++;
      usbx_cdc_acm_watch.write.phase = 6;
      write_status = ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)&msg_log , sizeof(msg_log) , &actual_length);
      usbx_cdc_acm_watch.write.transfer_status = write_status;
      usbx_cdc_acm_watch.write.actual_length = actual_length;
      usbx_cdc_acm_watch.write.after_transfer_count++;
      usbx_cdc_acm_watch.write.phase = 7;
      // new_data_ = 10;
      // if (new_data_) {
      //   ux_device_class_cdc_acm_write(cdc_acm, UserRxBufferFS , new_data_ , &actual_length);
      //   new_data_ = 0;
      // }
      // else {
      //   uint8_t test_data_ = 10;
      //   ux_device_class_cdc_acm_write(cdc_acm, UserRxBufferFS , test_data_ , &actual_length);
      // }
    }
    else
    {
      usbx_cdc_acm_watch.write.not_configured_count++;
      usbx_cdc_acm_watch.write.phase = 10;
    }
  }
}
/* USER CODE END 2 */
