/* hal_stub.h - minimal STM32 HAL stand-in so fpga_link.c builds on a PC. */
#ifndef HAL_STUB_H
#define HAL_STUB_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { int dummy; } SPI_HandleTypeDef;
typedef struct { int dummy; } GPIO_TypeDef;

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx,
                                          uint8_t *rx, uint16_t len,
                                          uint32_t timeout);
void     HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s);
void     HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);

#endif
