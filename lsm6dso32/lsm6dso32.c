/***************************************************************************//**
 *   @file   lsm6dso32.c
 *   @brief  Driver for lsm6dso32.
*******************************************************************************/

#include "lsm6dso32.h"

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

HAL_StatusTypeDef lsm6dso32_write_register(struct lsm6dso32_dev *dev,
											uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	return spi_write_and_read(dev->spi_desc, buf, 2, NULL, 0);
}

uint8_t lsm6dso32_read_register(struct lsm6dso32_dev *dev, uint8_t reg)
{
	uint8_t buf[2] = { reg, 0 };
	buf[0] |= BIT_READ_FLAG;
	spi_write_and_read(dev->spi_desc, buf, 1, &buf[1], 1);
	return buf[1];
}

HAL_StatusTypeDef lsm6dso32_read_block(struct lsm6dso32_dev *dev,
		uint8_t reg, uint8_t *buf, uint8_t size)
{
	reg |= BIT_READ_FLAG;
	return spi_write_and_read(dev->spi_desc, &reg, 1, buf, size);
}

LSM6DSO32_StatusTypeDef lsm6dso32_check_whoami(struct lsm6dso32_dev *dev)
{

    uint8_t whoami = lsm6dso32_read_register(dev, LSM6REG_WHO_AM_I);

    if (whoami == LSM6_WHOAMI) {
    	return LSM_OK;
    }
    // not a value WHOAMI result
    return LSM_ERR;
}

LSM6DSO32_StatusTypeDef lsm6dso32_reset(struct lsm6dso32_dev *dev)
{
	uint8_t reset_status = 1;
	HAL_StatusTypeDef status = lsm6dso32_write_register(dev,
			LSM6DSO32_CTRL3_C, (uint8_t)1);
	uint8_t tries = 20;
	do {
	  reset_status = lsm6dso32_read_register(dev, LSM6DSO32_CTRL3_C);
	  reset_status &= (0x01);
	  tries--;
	  if(tries == 0) return LSM_RESET;
	} while (reset_status);
	return status;
}

LSM6DSO32_StatusTypeDef lsm6dso32_set_scale(struct lsm6dso32_dev *dev)
{
	LSM6DSO32_StatusTypeDef status = LSM_OK;
	//acc
	uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL1_XL);
	ret |= (dev->acc_scale << 2);
	status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL1_XL, ret);
	//gyro
	uint8_t ret2 = lsm6dso32_read_register(dev, LSM6DSO32_CTRL2_G);
	ret2 |= (dev->gyro_scale << 1);
	status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL2_G, ret2);
	return status;
}

LSM6DSO32_StatusTypeDef lsm6dso32_fifo_config(struct lsm6dso32_dev *dev)
{
	LSM6DSO32_StatusTypeDef status = LSM_OK;

	status = lsm6dso32_write_register(dev, LSM6DSO32_FIFO_CTRL1,
			dev->fifo_desc->watermark);

	uint8_t val = (dev->fifo_desc->gyro_batch_rate << 4);
	val |= dev->fifo_desc->acc_batch_rate;
	status = lsm6dso32_write_register(dev, LSM6DSO32_FIFO_CTRL3, val);

	uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_FIFO_CTRL4);
	ret &= 0b11110000;
	ret |= dev->fifo_desc->mode;
	status = lsm6dso32_write_register(dev, LSM6DSO32_FIFO_CTRL4, ret);

	return status;
}

LSM6DSO32_StatusTypeDef lsm6dso32_set_rate(struct lsm6dso32_dev *dev)
{
	LSM6DSO32_StatusTypeDef status = LSM_OK;
	{
		uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL6_C);
		ret = (ret & ~(1UL << 4)) | (dev->acc_performance << 4);
		status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL6_C, ret);
	} {
		uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL7_G);
		ret = (ret & ~(1UL << 7)) | (dev->gyro_performance << 7);
		status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL7_G, ret);
	} {
		uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL1_XL);
		ret = (ret & ~((uint8_t)0x15 << 4)) | (dev->acc_rate << 4);
		status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL1_XL, ret);
	} {
		uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL2_G);
		ret = (ret & ~((uint8_t)0x15 << 4)) | (dev->gyro_rate << 4);
		status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL2_G, ret);
	}
	return status;
}

LSM6DSO32_StatusTypeDef lsm6dso32_i2c_i3c_disable(struct lsm6dso32_dev *dev)
{
	LSM6DSO32_StatusTypeDef status = LSM_OK;
	/* Disable I3C interface */
	uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL9_XL);
	ret |= 0x02;
	status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL9_XL, ret);
	//uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_I3C_BUS_AVB);
	/* Disable I2C interface */
	ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL4_C);
	ret |= 0x04;
	status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL4_C, ret);
	return status;
}

LSM6DSO32_StatusTypeDef lsm6dso32_statup(struct lsm6dso32_dev *dev)
{
	LSM6DSO32_StatusTypeDef status = LSM_OK;

	/* Wait sensor boot time */
	HAL_Delay(dev->boot_time);

	/* Check device ID */
	if (lsm6dso32_check_whoami(dev)) return LSM_ERR;

	/* Restore default configuration */
	status = lsm6dso32_reset(dev);

	/* Disable other interfaces than SPI */
	status = lsm6dso32_i2c_i3c_disable(dev);

	/* Enable Block Data Update */
	//1: output registers are not updated until MSB and LSB have been read)
	{
		uint8_t ret = lsm6dso32_read_register(dev, LSM6DSO32_CTRL3_C);
		ret |= (1 << 6);
		status = lsm6dso32_write_register(dev, LSM6DSO32_CTRL3_C, ret);
	}

	/* Set full scale */
	status = lsm6dso32_set_scale(dev);

	/* Set fifo configurations */
	status = lsm6dso32_fifo_config(dev);

	/* Set Output Data Rate */
	/* Set ODR (Output Data Rate) and power mode*/
	status = lsm6dso32_set_rate(dev);

	return status;
}

/*! Reads data from fifo and writes gyro and acc. Flushes fifo in default. !*/
void lsm6dso32_read_fifo(struct lsm6dso32_dev *dev)
{
	uint16_t num = 0;
    uint8_t wmflag = 0;
    uint8_t reg_tag_raw;
    LSM6DSO32_FifoTagDef reg_tag;

    uint8_t fifo_status_2 = lsm6dso32_read_register(dev,
    		LSM6DSO32_FIFO_STATUS2);
    wmflag = (fifo_status_2 >> 7);

    uint8_t fifo_status_1 = lsm6dso32_read_register(dev,
    		LSM6DSO32_FIFO_STATUS1);
    uint8_t mask = (1 << 2) - 1;
    fifo_status_2 &= mask;
    num = ((uint16_t)fifo_status_2 << 8) | (uint16_t)fifo_status_1;

    if (wmflag > 0) {
    //TODO nie dziala jesli tutaj czytamy num
      while (--num) {

    	//print_debug(dev->uart, num);
        reg_tag_raw = lsm6dso32_read_register(dev,
        		LSM6DSO32_FIFO_DATA_OUT_TAG);
        reg_tag = reg_tag_raw >> 3;

        switch (reg_tag) {
        case LSM6DSO32_XL_NC_TAG:
            memset(dev->acc_raw->u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso32_read_block(dev, LSM6DSO32_FIFO_DATA_OUT_X_L,
            		dev->acc_raw->u8bit, 6);
            lsm6dso32_raw_to_mg(dev);
            break;

        case LSM6DSO32_GYRO_NC_TAG:
        	memset(dev->gyro_raw->u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso32_read_block(dev, LSM6DSO32_FIFO_DATA_OUT_X_L,
            		dev->gyro_raw->u8bit, 6);
            lsm6dso32_raw_to_mdps(dev);
            break;

        default:
            /* Flush unused samples */
            memset(dev->dummy->u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso32_read_block(dev, LSM6DSO32_FIFO_DATA_OUT_X_L,
            		dev->dummy->u8bit, 6);
            break;
        }
        //TODO flaga na info ze nowe dane sciagniete
      }
    }
}

/*! With out interrupt reads data from output and writes gyro, acc and temp. !*/
void lsm6dso32_read_output(struct lsm6dso32_dev *dev)
{
	uint8_t status_reg = lsm6dso32_read_register(dev, LSM6DSO32_STATUS_REG);

	if (status_reg & (0x01)) {
		memset(dev->temp_raw->u8bit, 0x00, sizeof(int16_t));
		lsm6dso32_read_block(dev, LSM6DSO32_OUT_TEMP_L,
				dev->temp_raw->u8bit, 2);
		lsm6dso32_raw_to_degC(dev);
	} if (status_reg & (0x01 << 1)) {
		memset(dev->gyro_raw->u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso32_read_block(dev, LSM6DSO32_OUTX_L_G,
				dev->gyro_raw->u8bit, 6);
		lsm6dso32_raw_to_mdps(dev);
	} if (status_reg & (0x01 << 2)) {
		memset(dev->acc_raw->u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso32_read_block(dev, LSM6DSO32_OUTX_L_A,
				dev->acc_raw->u8bit, 6);
		lsm6dso32_raw_to_mg(dev);
	}
}

/*! Takes values from raw union. Converts them to mg and places data in acc struct !*/
LSM6DSO32_StatusTypeDef lsm6dso32_raw_to_mg(struct lsm6dso32_dev *dev)
{
	float scaller;

	switch (dev->acc_scale) {
	case LSM6DSO32_4g:
		scaller = 0.122;
		break;
	case LSM6DSO32_8g:
		scaller = 0.244;
		break;
	case LSM6DSO32_16g:
		scaller = 0.488;
		break;
	case LSM6DSO32_32g:
		scaller = 0.976;
		break;
	default:
		return LSM_ERR;
	}

	dev->acc->x = dev->acc_raw->i16bit[0] * scaller;
	dev->acc->y = dev->acc_raw->i16bit[1] * scaller;
	dev->acc->z = dev->acc_raw->i16bit[2] * scaller;

	return LSM_OK;
}

/*! Takes values from raw union. Converts them to mdps and places data in gyro struct !*/
LSM6DSO32_StatusTypeDef lsm6dso32_raw_to_mdps(struct lsm6dso32_dev *dev)
{
	float scaller;

	switch (dev->gyro_scale) {
	case LSM6DSO32_125dps:
		scaller = 4.375;
		break;
	case LSM6DSO32_250dps:
		scaller = 8.75;
		break;
	case LSM6DSO32_500dps:
		scaller = 17.50;
		break;
	case LSM6DSO32_1000dps:
		scaller = 35.00;
		break;
	case LSM6DSO32_2000dps:
		scaller = 70.00;
		break;
	default:
		return LSM_ERR;
	}

	dev->gyro->x = dev->gyro_raw->i16bit[0] * scaller;
	dev->gyro->y = dev->gyro_raw->i16bit[1] * scaller;
	dev->gyro->z = dev->gyro_raw->i16bit[2] * scaller;

	return LSM_OK;
}

/*! Takes values from raw union. Converts them to degC and places data in temp struct !*/
LSM6DSO32_StatusTypeDef lsm6dso32_raw_to_degC(struct lsm6dso32_dev *dev) {
	*(dev->temp) = ((float)(dev->temp_raw->i16bit / 256.0f)) + 25.0f;
	return LSM_OK;
}
