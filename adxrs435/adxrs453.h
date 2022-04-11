#pragma once

/***************************************************************************//**
 *   @file   adxrs453.h
 *   @brief  Header file of adxrs453 Driver.
*******************************************************************************/

#ifndef __ADXRS453_H__
#define __ADXRS453_H__

/***************************** Include Files **********************************/

#include <stdint.h>
#include "spi.h"
#include "ADXRS453_registers.h"
#include "stdbool.h"
#include "stdio.h"

/************************** ADXRS453 Definitions ******************************/

/*************************** Types Declarations *******************************/

typedef enum
{
	ADX_OK =	0x00U,
	ADX_ERR =	0x01U,
	ADX_PARITY =	0x02U
} ADXRS453_StatusTypeDef;

struct adxrs453_dev {
	/* SPI */
	spi_desc	*spi_desc;

	/* SELF */
	uint8_t error_flags;

    //for debug
    UART_HandleTypeDef *huart;
};

/************************ Functions Declarations ******************************/

/*! Takes ref to uint32_t value and sets LSB to assert parity ODD. Ref, so we can just return if already satisfied. Used only for sending commands. !*/
void adxrs453_change_parity (uint32_t *value);

/*! Copies uint16_t value, checks parity, returns TRUE/1 if ODD. Checks for both parity bits so it should be used ONLY for incoming responses. !*/
_Bool adxrs453_check_parity(uint32_t value);

/*! Initializes the ADXRS453 and checks if the device is present. !*/
ADXRS453_StatusTypeDef adxrs453_init(struct adxrs453_dev *dev);

/*! Reads and returns the value of a register. !*/
uint16_t adxrs453_get_register_value(struct adxrs453_dev *dev, uint8_t register_address);

/*! Writes data into a register. Checks if value is the same as requested. !*/
ADXRS453_StatusTypeDef adxrs453_set_register_value(struct adxrs453_dev *dev, uint8_t register_address, uint16_t register_value);

/*! Reads the sensor data with full duplex mode. Writes error_flags in dev if any read. There EF should be handled outside of this fn. !*/
float adxrs453_get_sensor_data(struct adxrs453_dev *dev);

/*! Reads the rate data and converts it to degrees/second. !*/
float adxrs453_get_rate(struct adxrs453_dev *dev);

/*! Reads the temperature sensor data and converts it to degrees Celsius. !*/
float adxrs453_get_temperature(struct adxrs453_dev *dev);

#endif // __ADXRS453_H__
