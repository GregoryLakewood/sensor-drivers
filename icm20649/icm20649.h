#pragma once

/***************************************************************************//**
 *   @file   icm20649.h
 *   @brief  Header file of icm20649 Driver.
*******************************************************************************/

#ifndef INC_ICM20649_H_
#define INC_ICM20649_H_

/***************************** Include Files **********************************/

#include "AP_InertialSensor_Invensensev2_registers.h"
#include "spi.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/************************** ICM20649 Definitions ******************************/

#define GRAVITY_MSS     9.80665f

/*************************** Types Declarations *******************************/

typedef enum
{
	ICM_OK =	0x00U,
	ICM_ERR =	0x01U,
	ICM_INIT =	0x02U,	//dev failed the init
	ICM_FIFO =  0x03U	//error in reading fifo bytes
} ICM20649_StatusTypeDef;

typedef struct Vector3f {
	float x;
	float y;
	float z;
} Vector3f;

struct icm20649_dev {
	/* SPI */
	spi_desc	*spi_desc;
	/* SELF */

	// Last status from register user control
	uint8_t last_stat_user_ctrl;

	// Initialize as 0xFF to avoid mistake
	uint8_t current_bank;

	// do we use drdy pin?
	bool drdy_pin;

	// 29.5g is the limit, if +/- 30g range is set
	float clip_limit;

	// 1024 lsb/g
	float accel_scale;

	float fifo_accel_scale;
	float fifo_gyro_scale;

	// Buffer for fifo read
	uint8_t *fifo_buffer;

	// Raw temperature is used for fifo corruption check
	int16_t raw_temp;

    /*
      accumulators for sensor_rate sampling
      See description in _accumulate_sensor_rate_sampling()
    */
    struct {
        Vector3f accel;
        Vector3f gyro;
        float temp;
        uint8_t accel_count;
        uint8_t gyro_count;
    } accum;

    // TODO temporary vectors for printing
    Vector3f gyro_print;
    Vector3f accel_print;

    // accel filter parameters
    float accel_filter_sample_freq;
    float accel_filter_cutoff_freq;
    float accel_filter_alpha;

    // are we doing more than 1kHz sampling?
    bool fast_sampling;

    // what downsampling rate are we using from the FIFO for gyros?
    uint8_t gyro_fifo_downsample_rate;

    // what downsampling rate are we using from the FIFO for accels?
    uint8_t accel_fifo_downsample_rate;

    // what rate are we generating samples into the backend for gyros?
    uint16_t gyro_backend_rate_hz;

    // what rate are we generating samples into the backend for accels?
    uint16_t accel_backend_rate_hz;

    // this value is rate of main control loop
    // sensors out should be at leart the same
    // if loop rate is low, we can downsample
    // TODO this value source should be much much higher in absract
    uint16_t loop_rate_hz;

    //for debug
    UART_HandleTypeDef *huart;
};

/************************ Functions Declarations ******************************/

/*! Changes Bank_Select register, should not be used directly !*/
ICM20649_StatusTypeDef icm20649_change_bank(struct icm20649_dev *dev, uint8_t bank);

/*! Writes value to register. Reg first byte specifies bank number. !*/
ICM20649_StatusTypeDef icm20649_write_register(struct icm20649_dev *dev, uint16_t reg, uint8_t val);

/*! Reads and returns value from register. Reg first byte specifies bank number. !*/
uint8_t icm20649_read_register(struct icm20649_dev *dev, uint16_t reg);

/*! Calls a register and reads all the data it stores. Fnc used when recv_len > 1. !*/
ICM20649_StatusTypeDef icm20649_read_block(struct icm20649_dev *dev, uint16_t reg, uint8_t *buf, uint8_t size);

/*! Checks if DRDY_PIN is assert (if used), else checks REG_DRDY, true if data ready. !*/
bool icm20649_data_ready(struct icm20649_dev *dev);

/*! Sets the registers, checks proper functioning, returns errs, if any. !*/
ICM20649_StatusTypeDef icm20469_hardware_init(struct icm20649_dev *dev);

/*! Resets fifo config. Does not change last saved params (no params set after init) !*/
ICM20649_StatusTypeDef icm20649_fifo_reset(struct icm20649_dev *dev);

/*! Read device ID and check if it is correct !*/
ICM20649_StatusTypeDef icm20649_check_whoami(struct icm20649_dev *dev);

/*! Takes sample list, scales and one by one places it in the destination. !*/
bool icm20649_accumulate(struct icm20649_dev *dev, uint8_t *samples, uint8_t n_samples);

/*! Reads fifo buffer and saves data. !*/
void icm20649_read_fifo(struct icm20649_dev *dev);

/*! Initializes sensor settings !*/
void icm20649_start(struct icm20649_dev *dev);

/*! Set the DLPF filter frequency and gyro accel scaling. !*/
void icm20649_set_filter_and_scaling(struct icm20649_dev *dev);

/*! Checks whether raw temp is okay. If not, fifo data is corrupt !*/
bool icm20649_check_raw_temp(struct icm20649_dev *dev, int16_t t2);

#endif /* INC_ICM20649_H_ */
