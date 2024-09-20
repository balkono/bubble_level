#include "myHAL.h"

void SPI_TXBuffer(uint16_t* buffer, uint32_t len)
{
	uint8_t* buffer_8b = (uint8_t*)buffer;
    HAL_SPI_Transmit(&hspi3, buffer_8b, len * 2, HAL_MAX_DELAY);
}

void SPI_TXByte(uint8_t data)
{
    HAL_SPI_Transmit(&hspi3, &(uint8_t){data}, 1, HAL_MAX_DELAY);
}

void GPIO_SetPin(GPIO_TypeDef * Port, uint16_t Pin)
{
    HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
}

void GPIO_ResetPin(GPIO_TypeDef * Port, uint16_t Pin)
{
    HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
}

void I2C_TXBuffer(uint8_t* buf, uint32_t len )
{
    HAL_I2C_Master_Transmit(I2C_HANDLE, I2C_ADR<<1, buf, len, HAL_MAX_DELAY);
}

void I2C_RXBuffer(uint8_t* buf, uint32_t len)
{
    HAL_I2C_Master_Receive(I2C_HANDLE, I2C_ADR<<1, buf, len, HAL_MAX_DELAY );
}

