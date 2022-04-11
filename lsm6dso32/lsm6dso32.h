#pragma once

/***************************************************************************//**
 *   @file   lsm6dso32.h
 *   @brief  Header file of lsm6dso32 Driver.
*******************************************************************************/

#ifndef INC_LSM6DSO32_H_
#define INC_LSM6DSO32_H_

/***************************** Include Files **********************************/

#include "ST_LSM6_registers.h"
#include "spi.h"
#include <string.h>

/************************** LSM6DSO32 Definitions *****************************/

#define GRAVITY_MSS     9.80665f

/*************************** Types Declarations *******************************/

typedef enum
{
	LSM_OK =	0x00U,
	LSM_ERR =	0x01U,
	LSM_INIT =	0x02U,	//dev failed the init
	LSM_RESET = 0x03U	//reset did not finish
} LSM6DSO32_StatusTypeDef;

typedef enum
{
	  LSM6DSO32_4g       = (uint8_t)0x00,
	  LSM6DSO32_8g       = (uint8_t)0x02,
	  LSM6DSO32_16g      = (uint8_t)0x03,
	  LSM6DSO32_32g    	 = (uint8_t)0x01
} LSM6DSO32_AccScaleDef;

typedef enum
{
	  LSM6DSO32_125dps   = (uint8_t)0x01,
	  LSM6DSO32_250dps   = (uint8_t)0x00,
	  LSM6DSO32_500dps   = (uint8_t)0x02,
	  LSM6DSO32_1000dps  = (uint8_t)0x04,
	  LSM6DSO32_2000dps  = (uint8_t)0x06
} LSM6DSO32_GyroScaleDef;

typedef enum
{
  LSM6DSO32_NOT_BATCHED		=  0,
  LSM6DSO32_12Hz5_BATCH		=  1,
  LSM6DSO32_26Hz_BATCH    	=  2,
  LSM6DSO32_52Hz_BATCH    	=  3,
  LSM6DSO32_104Hz_BATCH   	=  4,
  LSM6DSO32_208Hz_BATCH   	=  5,
  LSM6DSO32_417Hz_BATCH   	=  6,
  LSM6DSO32_833Hz_BATCH   	=  7,
  LSM6DSO32_1667Hz_BATCH  	=  8,
  LSM6DSO32_3333Hz_BATCH  	=  9,
  LSM6DSO32_6667Hz_BATCH  	= 10,
  LSM6DSO32_6Hz5_BATCH    	= 11,
  LSM6DSO32_1Hz6_BATCH    	= 11
} LSM6DSO32_BatchDef;

typedef enum
{
  LSM6DSO32_BYPASS          = 0, //fifo off
  LSM6DSO32_FIFO            = 1, //stops collecting when fifo full
  LSM6DSO32_CONT_TO_FIFO    = 3, //until trigger is deasserted
  LSM6DSO32_BYPASS_TO_CONT  = 4, //until trigger is deasserted
  LSM6DSO32_CONT            = 6, //when fifo full, overwrite oldest
  LSM6DSO32_BYPASS_TO_FIFO  = 7 //until trigger is deasserted
} LSM6DSO32_FifoModeDef;

typedef enum
{
  LSM6DSO32_ODR_OFF			= 0x00,
  LSM6DSO32_ODR_1Hz6     	= 0x0B, //only acc in power low
  LSM6DSO32_ODR_12Hz5     	= 0x01,
  LSM6DSO32_ODR_26Hz       	= 0x02,
  LSM6DSO32_ODR_52Hz        = 0x03,
  LSM6DSO32_ODR_104Hz     	= 0x04,
  LSM6DSO32_ODR_208Hz      	= 0x05,
  LSM6DSO32_ODR_417Hz     	= 0x06,
  LSM6DSO32_ODR_833Hz      	= 0x07,
  LSM6DSO32_ODR_1667Hz    	= 0x08,
  LSM6DSO32_ODR_3333Hz     	= 0x09,
  LSM6DSO32_ODR_6667Hz     	= 0x0A
} LSM6DSO32_OutputDataRateDef;

typedef enum
{
  HIGH_PERFORMANCE			= 0x00,
  LOW_PERFORMANCE			= 0x01
} LSM6DSO32_PerformancePowerDef;

typedef struct fifo_desc {
	uint8_t 		   watermark; //threshold can be uint9_t (sic) but here
					   	   	   	  //it is ignored
	LSM6DSO32_BatchDef acc_batch_rate;
	LSM6DSO32_BatchDef gyro_batch_rate;
	LSM6DSO32_FifoModeDef mode;
} fifo_desc;

typedef struct vector3d {
	float x;
	float y;
	float z;
} vector3d; //TODO place it upper in abstract

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef enum
{
  LSM6DSO32_GYRO_NC_TAG = 1,
  LSM6DSO32_XL_NC_TAG,
  LSM6DSO32_TEMPERATURE_TAG,
  LSM6DSO32_TIMESTAMP_TAG,
  LSM6DSO32_CFG_CHANGE_TAG,
  LSM6DSO32_XL_NC_T_2_TAG,
  LSM6DSO32_XL_NC_T_1_TAG,
  LSM6DSO32_XL_2XC_TAG,
  LSM6DSO32_XL_3XC_TAG,
  LSM6DSO32_GYRO_NC_T_2_TAG,
  LSM6DSO32_GYRO_NC_T_1_TAG,
  LSM6DSO32_GYRO_2XC_TAG,
  LSM6DSO32_GYRO_3XC_TAG,
  LSM6DSO32_SENSORHUB_SLAVE0_TAG,
  LSM6DSO32_SENSORHUB_SLAVE1_TAG,
  LSM6DSO32_SENSORHUB_SLAVE2_TAG,
  LSM6DSO32_SENSORHUB_SLAVE3_TAG,
  LSM6DSO32_STEP_COUNTER_TAG,
  LSM6DSO32_SENSORHUB_NACK_TAG = 0x19,
} LSM6DSO32_FifoTagDef;

struct lsm6dso32_dev {
	/* SPI */
	spi_desc			*spi_desc;
	/* SENSORS */
	vector3d *gyro;
	vector3d *acc;
	float *temp;
	axis3bit16_t *dummy;
	axis3bit16_t *gyro_raw;
	axis3bit16_t *acc_raw;
	axis1bit16_t *temp_raw;
	/* SELF */
	fifo_desc 			*fifo_desc; //fifo config
	LSM6DSO32_AccScaleDef	acc_scale;
	LSM6DSO32_GyroScaleDef  gyro_scale;
	LSM6DSO32_OutputDataRateDef acc_rate;
	LSM6DSO32_OutputDataRateDef gyro_rate;
	LSM6DSO32_PerformancePowerDef acc_performance;
	LSM6DSO32_PerformancePowerDef gyro_performance;
	uint16_t			boot_time; //ms

	UART_HandleTypeDef *uart;
};

/************************ Functions Declarations ******************************/

/*! Read device ID and check if it is correct !*/
LSM6DSO32_StatusTypeDef lsm6dso32_check_whoami(struct lsm6dso32_dev *dev);

/*! Writes value to register. !*/
HAL_StatusTypeDef lsm6dso32_write_register(struct lsm6dso32_dev *dev, uint8_t reg, uint8_t val);

/*! Reads and returns value from register. !*/
uint8_t lsm6dso32_read_register(struct lsm6dso32_dev *dev, uint8_t reg);

/*! Reads many values by reading from register. !*/
HAL_StatusTypeDef lsm6dso32_read_block(struct lsm6dso32_dev *dev, uint8_t reg, uint8_t *buf, uint8_t size);

/*! Software reset. Resets register values to default. !*/
LSM6DSO32_StatusTypeDef lsm6dso32_reset(struct lsm6dso32_dev *dev);

/*! Set scale for acc and gyro. !*/
LSM6DSO32_StatusTypeDef lsm6dso32_set_scale(struct lsm6dso32_dev *dev);

/*! Set fifo parameters. !*/
LSM6DSO32_StatusTypeDef lsm6dso32_fifo_config(struct lsm6dso32_dev *dev);

/*! Set sensor rate and performance level. !*/
LSM6DSO32_StatusTypeDef lsm6dso32_set_rate(struct lsm6dso32_dev *dev);

/*! Turns off I3C and I2c interfaces. !*/
LSM6DSO32_StatusTypeDef lsm6dso32_i2c_i3c_disable(struct lsm6dso32_dev *dev);

/*! Startup sequence. !*/
LSM6DSO32_StatusTypeDef lsm6dso32_statup(struct lsm6dso32_dev *dev);

/*! Reads data from fifo and writes gyro and acc. Flushes fifo in default. !*/
void lsm6dso32_read_fifo(struct lsm6dso32_dev *dev);

/*! Reads data from output and writes gyro, acc and temp. !*/
void lsm6dso32_read_output(struct lsm6dso32_dev *dev);

/*! Takes values from raw union. Converts them to mg and places data in acc struct !*/
LSM6DSO32_StatusTypeDef lsm6dso32_raw_to_mg(struct lsm6dso32_dev *dev);

/*! Takes values from raw union. Converts them to mdps and places data in gyro struct !*/
LSM6DSO32_StatusTypeDef lsm6dso32_raw_to_mdps(struct lsm6dso32_dev *dev);

/*! Takes values from raw union. Converts them to degC and places data in temp struct !*/
LSM6DSO32_StatusTypeDef lsm6dso32_raw_to_degC(struct lsm6dso32_dev *dev);

#endif /* INC_LSM6DSO32_H_ */
