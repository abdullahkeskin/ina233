/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "LTC4100.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define send_debug(x) printf(x)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define LM5069_PWR_GOOD_Pin GPIO_PIN_4
#define LM5069_PWR_GOOD_GPIO_Port GPIOF
#define FT813_INT_Pin GPIO_PIN_5
#define FT813_INT_GPIO_Port GPIOF
#define FTDI_EN_Pin GPIO_PIN_6
#define FTDI_EN_GPIO_Port GPIOF
#define FTDI_SLEEP_Pin GPIO_PIN_7
#define FTDI_SLEEP_GPIO_Port GPIOF
#define EXT_FLASH_CS_Pin GPIO_PIN_8
#define EXT_FLASH_CS_GPIO_Port GPIOF
#define E_FUSE_SHDN_Pin GPIO_PIN_10
#define E_FUSE_SHDN_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define FT813_PD_N_Pin GPIO_PIN_0
#define FT813_PD_N_GPIO_Port GPIOC
#define FT813_CS_N_Pin GPIO_PIN_1
#define FT813_CS_N_GPIO_Port GPIOC
#define SPI2_MISO_Pin GPIO_PIN_2
#define SPI2_MISO_GPIO_Port GPIOC
#define SPI2_MOSI_Pin GPIO_PIN_3
#define SPI2_MOSI_GPIO_Port GPIOC
#define GPIO_SHOOTING_DIODE_CONNECT_RELAY_Pin GPIO_PIN_4
#define GPIO_SHOOTING_DIODE_CONNECT_RELAY_GPIO_Port GPIOC
#define GPIO_WIRELINE_INDUCTOR_BYPASS_RELAY_Pin GPIO_PIN_5
#define GPIO_WIRELINE_INDUCTOR_BYPASS_RELAY_GPIO_Port GPIOC
#define AFE_GPIO_DAC_Pin GPIO_PIN_12
#define AFE_GPIO_DAC_GPIO_Port GPIOF
#define DIG_RES_SPI1__CS_Pin GPIO_PIN_13
#define DIG_RES_SPI1__CS_GPIO_Port GPIOF
#define ADC1_SPI1__CS_Pin GPIO_PIN_14
#define ADC1_SPI1__CS_GPIO_Port GPIOF
#define TX_AMP_SHUTDOWN_Pin GPIO_PIN_7
#define TX_AMP_SHUTDOWN_GPIO_Port GPIOE
#define GPIO_EN_RX_SWITCH_Pin GPIO_PIN_8
#define GPIO_EN_RX_SWITCH_GPIO_Port GPIOE
#define GPIO_EN_TX_SWITCH_Pin GPIO_PIN_10
#define GPIO_EN_TX_SWITCH_GPIO_Port GPIOE
#define AFE_SPI1_CS_Pin GPIO_PIN_12
#define AFE_SPI1_CS_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define _48V_BOOST_ENABLE_Pin GPIO_PIN_10
#define _48V_BOOST_ENABLE_GPIO_Port GPIOD
#define UC_ENB__24V_Pin GPIO_PIN_11
#define UC_ENB__24V_GPIO_Port GPIOD
#define UC_ENB__24VD12_Pin GPIO_PIN_12
#define UC_ENB__24VD12_GPIO_Port GPIOD
#define GPIO_REV_ENG_SW2_RELAY_Pin GPIO_PIN_13
#define GPIO_REV_ENG_SW2_RELAY_GPIO_Port GPIOD
#define AFE_GPIO_INT_Pin GPIO_PIN_14
#define AFE_GPIO_INT_GPIO_Port GPIOD
#define AFE_GPIO_SD_Pin GPIO_PIN_15
#define AFE_GPIO_SD_GPIO_Port GPIOD
#define GPIO_REV_ENG_SW_PCB_RELAY_Pin GPIO_PIN_2
#define GPIO_REV_ENG_SW_PCB_RELAY_GPIO_Port GPIOG
#define GPIO_AFE_RX_SWITCH_Pin GPIO_PIN_3
#define GPIO_AFE_RX_SWITCH_GPIO_Port GPIOG
#define GPIO_AFE_TX_SWITCH_Pin GPIO_PIN_4
#define GPIO_AFE_TX_SWITCH_GPIO_Port GPIOG
#define GPIO_DOWNLINK_CONNECTION_SSR_Pin GPIO_PIN_5
#define GPIO_DOWNLINK_CONNECTION_SSR_GPIO_Port GPIOG
#define GPIO_WIRELINE_DIODE_SHORT_Pin GPIO_PIN_6
#define GPIO_WIRELINE_DIODE_SHORT_GPIO_Port GPIOG
#define LD7_Pin GPIO_PIN_7
#define LD7_GPIO_Port GPIOG
#define GPIO_DOWNLINK_CONNECTION_RELAY_Pin GPIO_PIN_8
#define GPIO_DOWNLINK_CONNECTION_RELAY_GPIO_Port GPIOG
#define AFE_TX_FLAG_Pin GPIO_PIN_6
#define AFE_TX_FLAG_GPIO_Port GPIOC
#define AFE_RX_FLAG_Pin GPIO_PIN_7
#define AFE_RX_FLAG_GPIO_Port GPIOC
#define GPIO_SHOOTING_PANEL_GND_SSR_Pin GPIO_PIN_8
#define GPIO_SHOOTING_PANEL_GND_SSR_GPIO_Port GPIOC
#define GPIO_REV_ENG_SW1_RELAY_Pin GPIO_PIN_9
#define GPIO_REV_ENG_SW1_RELAY_GPIO_Port GPIOC
#define LD8_Pin GPIO_PIN_9
#define LD8_GPIO_Port GPIOA
#define ADC1_BUSY_Pin GPIO_PIN_12
#define ADC1_BUSY_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RX_METHOD_SELECTOR_Pin GPIO_PIN_9
#define RX_METHOD_SELECTOR_GPIO_Port GPIOG
#define FSK_SOURCE_SELECTOR_Pin GPIO_PIN_10
#define FSK_SOURCE_SELECTOR_GPIO_Port GPIOG
#define AD9834_FSELECT_Pin GPIO_PIN_11
#define AD9834_FSELECT_GPIO_Port GPIOG
#define AD9834_PSELECT_Pin GPIO_PIN_12
#define AD9834_PSELECT_GPIO_Port GPIOG
#define AD9834_SLEEP_Pin GPIO_PIN_13
#define AD9834_SLEEP_GPIO_Port GPIOG
#define AD9834_RESET_Pin GPIO_PIN_14
#define AD9834_RESET_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
