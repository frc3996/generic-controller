/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v2.0_Cube
  * @brief          : USB Device Custom HID interface file.
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
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
	/* USER CODE BEGIN 0 */
	0x05, 0x01,        // Usage Page (Generic Desktop) - Specifies the type of device (e.g., joystick, mouse)
	0x09, 0x04,        // Usage (Joystick) - Device type is a joystick
	0xA1, 0x01,        // Collection (Application) - Begins the joystick's application-level description

	// Axes Definitions
	0x09, 0x01,      // Usage (Pointer) - Sub-collection for joystick axes
	0xA1, 0x00,      // Collection (Physical) - Physical collection for axes

	// X, Y, Z, Rx, Ry, Rz, Slider, Dial (8 axes)
	0x09, 0x30,    // Usage (X)
	0x09, 0x31,    // Usage (Y)
	0x09, 0x32,    // Usage (Z)
	0x09, 0x33,    // Usage (Rx)

	// Logical range for all axes (-127 to 127)
	0x15, 0x81,    // Logical Minimum (-127)
	0x25, 0x7F,    // Logical Maximum (127)

	// Report format: 8 bytes (8 axes, 8 bits each)
	0x75, 0x08,    // Report Size (8 bits per axis)
	0x95, 0x04,    // Report Count (8 axes)

	// Input report for axes: Data, Variable, Absolute (input values)
	0x81, 0x02,    // Input (Data, Variable, Absolute)

	0xC0,            // End Collection - End of axes collection

	// Buttons Definitions
	0x05, 0x09,      // Usage Page (Button) - Button controls
	0x19, 0x01,      // Usage Minimum (Button 1) - First button
	0x29, 0x10,      // Usage Maximum (Button 22) - Defines up to 22 buttons

	// Logical range for buttons (0: not pressed, 1: pressed)
	0x15, 0x00,      // Logical Minimum (0)
	0x25, 0x01,      // Logical Maximum (1)

	// Report format: 22 buttons (1 bit per button)
	0x75, 0x01,      // Report Size (1 bit per button)
	0x95, 0x10,      // Report Count (22 buttons)

	// Input report for buttons: Data, Variable, Absolute (input values)
	0x81, 0x02,      // Input (Data, Variable, Absolute)

	// LED Output Definitions (4 LEDs)
	0x05, 0x08,      // Usage Page (LED) - Output controls for LEDs
	0x19, 0x01,      // Usage Minimum (LED 1) - First LED
	0x29, 0x04,      // Usage Maximum (LED 12) - Defines up to 12 LEDs

	// Logical range for LEDs (0: off, 1: on)
	0x15, 0x00,      // Logical Minimum (0)
	0x25, 0x01,      // Logical Maximum (1)

	// Report format: 12 LEDs (1 bit per LED)
	0x75, 0x01,      // Report Size (1 bit per LED)
	0x95, 0x04,      // Report Count (12 LEDs)

	// Output report for LEDs: Data, Variable, Absolute (output values)
	0x91, 0x02,      // Output (Data, Variable, Absolute)

	// Padding: 4 bits to align the output data to byte boundaries
	0x95, 0x01,      // Report Count (1)
	0x75, 0x04,      // Report Size (4 bits of padding)
	0x91, 0x03,      // Output (Constant) - Padding

	/* USER CODE END 0 */
	0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

