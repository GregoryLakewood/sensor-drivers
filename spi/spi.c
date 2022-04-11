#include "spi.h"

#include <stdio.h>

/*! Transmit only basic func for spi !*/
HAL_StatusTypeDef spi_write(struct spi_desc *desc,
		uint8_t *send, uint8_t send_len)
{
	uint8_t status;
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(desc->handle, send, send_len, 1000);
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_SET);
	return status;
}

/*! Receive only basic func for spi !*/
HAL_StatusTypeDef spi_read(struct spi_desc *desc,
		uint8_t *recv, uint8_t recv_len)
{
	uint8_t status;
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_RESET);
	status = HAL_SPI_Receive(desc->handle, recv, recv_len, 1000); //TODO nie bedzie floating? moze trzeba transmit 0x00
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_SET);
	return status;
}

/*! Function for full-duplex, simultaneous, spi transfer !*/
HAL_StatusTypeDef spi_write_and_read_duplex(struct spi_desc *desc,
		uint8_t *send, uint8_t send_len,
		uint8_t *recv, uint8_t recv_len)
{
	uint8_t status;
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(desc->handle, send, recv, send_len, 1000);
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_SET);
	return status;
}

/*! Basic function for semi-duplex spi transfer !*/
HAL_StatusTypeDef spi_write_and_read(struct spi_desc *desc,
		uint8_t *send, uint8_t send_len,
		uint8_t *recv, uint8_t recv_len)
{
	uint8_t status = HAL_OK;
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(desc->handle, send, send_len, 100);
	if (recv_len) {
		status = HAL_SPI_Receive(desc->handle, recv, recv_len, 100);
	}
	HAL_GPIO_WritePin(desc->port, desc->pin, GPIO_PIN_SET);
	return status;
}

/*! Simple function to print message on UART !*/
void print_error(UART_HandleTypeDef *huart, char *msg)
{
	char uart_buf[50];
	uint8_t uart_buf_len = sprintf(uart_buf, "%s\r\n", msg);
    HAL_UART_Transmit(huart, (uint8_t *)uart_buf, uart_buf_len, 100);
}

/*! Change speed of SPI - specifically prescaller !*/
_Bool spi_set_speed(SPI_HandleTypeDef *handle, SPI_SpeedTypeDef speed)
{
	switch (speed) {
	case SPEED_HIGH:
		handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
		if (HAL_SPI_Init(&hspi2)) Error_Handler();
		break;
	case SPEED_LOW:
		handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
		if (HAL_SPI_Init(&hspi2)) Error_Handler();
		break;
	}
	return 1;
}
