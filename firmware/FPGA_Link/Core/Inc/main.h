/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   FPGA_Link - Nucleo-G431KB <-> Basys 3 SPI register link.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define FPGA_TICK_Pin GPIO_PIN_0
#define FPGA_TICK_GPIO_Port GPIOB
#define FPGA_TICK_EXTI_IRQn EXTI0_IRQn
#define FPGA_FAULT_N_Pin GPIO_PIN_8
#define FPGA_FAULT_N_GPIO_Port GPIOA
#define FPGA_CS_Pin GPIO_PIN_11
#define FPGA_CS_GPIO_Port GPIOA
#define FPGA_IRQ_Pin GPIO_PIN_12
#define FPGA_IRQ_GPIO_Port GPIOA
#define FPGA_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define FPGA_SCK_Pin GPIO_PIN_3
#define FPGA_SCK_GPIO_Port GPIOB
#define FPGA_MISO_Pin GPIO_PIN_4
#define FPGA_MISO_GPIO_Port GPIOB
#define FPGA_MOSI_Pin GPIO_PIN_5
#define FPGA_MOSI_GPIO_Port GPIOB
#define FPGA_RST_N_Pin GPIO_PIN_6
#define FPGA_RST_N_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
