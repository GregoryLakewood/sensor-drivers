#pragma once

/***************************************************************************//**
 *   @file   as5048.h
 *   @brief  Header file of as5048 Driver.
*******************************************************************************/

#ifndef INC_AS5048_H_
#define INC_AS5048_H_

/***************************** Include Files **********************************/

#include "AS5048_registers.h"
#include "spi.h"
#include "stdbool.h"

/************************** AS5048 Definitions ********************************/

//#define GRAVITY_MSS     9.80665f
#define PWM_SCALE			0.00613f

/*************************** Types Declarations *******************************/

typedef enum
{
	AS_OK =		0x00U,
	AS_ERR =	0x01U,
	AS_WRITE_FAIL = 	0x02U,
	AS_FRAMING_ERROR =	0x03U,	//used in Clear Error Flag
	AS_COMMAND_INVALID =	0x04U,	//used in Clear Error Flag
	AS_PARITY_ERROR =	0x05U,	//used in Clear Error Flag
	AS_OCF =	0x06U,	//Offset Compensation Finished
	AS_COF =	0x07U,	//CORDIC Overflow
	AS_COMPL =	0x08U,	//strong mag field
	AS_COMPH =	0x09U	//weak mag field
} AS5048_StatusTypeDef;

struct as5048_dev {
	/* SPI */
	spi_desc	*spi_desc;

	/* SELF */
	_Bool error_flag;

    //for debug
    UART_HandleTypeDef *huart;
};

/************************ Functions Declarations ******************************/

/*! Takes ref to uint16_t value and sets MSB to assert parity EVEN. Ref, so we can just return if already satisfied. !*/
void as5048_change_parity (uint16_t *value);

/*! Copies uint16_t value, checks parity, returns TRUE/1 if ODD !*/
_Bool as5048_check_parity(uint16_t value);

/*! Reads and returns value from register. Checks parity and error flags. Return value without parity and EF bits. !*/
uint16_t as5048_read_register(struct as5048_dev *dev, uint16_t reg);

/*! Writes data to register. Dev returns data written. AS_OK if all ok, and (return data) == (data) !*/
AS5048_StatusTypeDef as5048_write_register(struct as5048_dev *dev, uint16_t reg, uint16_t data);

/*! Reads error register. Returns enum err type, err if err persists or OK if EF wasnt assested to begin with !*/
AS5048_StatusTypeDef as5048_clear_error_flag(struct as5048_dev *dev);

/*! Read and return what zero offset is programmed onto device !*/
uint16_t as5048_read_zero(struct as5048_dev *dev);

/*! Set zero offset - no programming onto device !*/
AS5048_StatusTypeDef as5048_set_zero(struct as5048_dev *dev, uint16_t zero);

/*! Just checks the diagnostic register, returns enumed error!*/
AS5048_StatusTypeDef as5048_diagnostics(struct as5048_dev *dev);

/*! Returns Automatic Gain Control value. 0 means strong mag field, 255 - weak. !*/
uint8_t as5048_read_agc(struct as5048_dev *dev);

/*! Returns magnitude in CORDIC. !*/
uint16_t as5048_read_magnitude(struct as5048_dev *dev);

/*! Returns calculated angle, offset included. Output in degrees. !*/
float as5048_read_angle(struct as5048_dev *dev);

/*! Returns angle, source PWM, offset included. Output in degrees. Depends on PWM_SCALE macro !*/
float as5048_read_angle_pwm(uint16_t read_pwm_value);

#endif /* INC_AS5048_H_ */
