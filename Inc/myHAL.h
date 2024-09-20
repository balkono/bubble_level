#ifndef MY_HAL_H
#define MY_HAL_H

#include "stdint.h"
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;
#define I2C_HANDLE &hi2c1
#define I2C_ADR 0x68

void SPI_TXBuffer(uint16_t* buffer, uint32_t len);

void SPI_TXByte(uint8_t data);

void GPIO_SetPin(GPIO_TypeDef * Port, uint16_t Pin);

void GPIO_ResetPin(GPIO_TypeDef * Port, uint16_t Pin);

void I2C_TXBuffer(uint8_t* buf, uint32_t len);

void I2C_RXBuffer(uint8_t* buf, uint32_t len);



#endif // MY_HAL_H
