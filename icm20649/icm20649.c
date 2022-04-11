/*
 * icm20649.c
 *
 *  Created on: Feb 15, 2022
 */

#include "icm20649.h"

/*
 *  DS-000189-ICM-20948-v1.3.pdf, page 11, section 3.1 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = (0.0174532f / 16.4f);

#define INV2_SAMPLE_SIZE 14
#define INV2_FIFO_BUFFER_LEN 8

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

float temp_sensitivity = 1.0f/333.87f; // degC/LSB
float temp_zero = 21; // degC

/*! Changes Bank_Select register, should not be used directly !*/
ICM20649_StatusTypeDef icm20649_change_bank(struct icm20649_dev *dev,
		uint8_t bank)
{
	ICM20649_StatusTypeDef status = ICM_OK;
	if (bank != dev->current_bank) {
		uint8_t buf[2] = { INV2REG_BANK_SEL, bank << 4 };
		status = spi_write_and_read(dev->spi_desc, buf, 2, NULL, 0);
		if (status == ICM_OK){
			dev->current_bank = bank;
		}
	}
	return status;
}

/*! Writes value to register. Reg first byte specifies bank number. !*/
ICM20649_StatusTypeDef icm20649_write_register(struct icm20649_dev *dev,
		uint16_t reg, uint8_t val)
{
	if (dev->current_bank != GET_BANK(reg)) {
		if (icm20649_change_bank(dev, GET_BANK(reg)) != ICM_OK){
			return ICM_ERR;
		}
	}
	uint8_t buf[2] = { GET_REG(reg), val };
	return spi_write_and_read(dev->spi_desc, buf, 2, NULL, 0);
}

/*! Reads and returns value from register. Reg first byte specifies bank number. !*/
uint8_t icm20649_read_register(struct icm20649_dev *dev, uint16_t reg)
{
	if (dev->current_bank != GET_BANK(reg)) {
		if (icm20649_change_bank(dev, GET_BANK(reg)) != ICM_OK){
			return ICM_ERR;
		}
	}
	uint8_t buf[2] = { GET_REG(reg), 0 };
	buf[0] |= BIT_READ_FLAG;
	spi_write_and_read(dev->spi_desc, buf, 1, &buf[1], 1);
	return buf[1];
}

/*! Calls a register and reads all the data it stores. Fnc used when recv_len > 1. !*/
ICM20649_StatusTypeDef icm20649_read_block(struct icm20649_dev *dev,
		uint16_t reg, uint8_t *buf, uint8_t size)
{
	if (dev->current_bank != GET_BANK(reg)) {
		if (icm20649_change_bank(dev, GET_BANK(reg))){
			return ICM_ERR;
		}
	}
	uint8_t send = GET_REG(reg);
	send |= BIT_READ_FLAG;
	return spi_write_and_read(dev->spi_desc, &send, 1, buf, size);
}

/*! Checks if DRDY_PIN is assert (if used), else checks REG_DRDY, true if data ready. !*/
bool icm20649_data_ready(struct icm20649_dev *dev)
{
    if (dev->drdy_pin == true) {
    	return HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin);
    } else {
    	return icm20649_read_register(dev, INV2REG_INT_STATUS_1);
    }
}

/*! Sets the registers, checks proper functioning, returns errs, if any. !*/
ICM20649_StatusTypeDef icm20469_hardware_init(struct icm20649_dev *dev)
{
	spi_set_speed(dev->spi_desc->handle, SPEED_LOW);

    if (icm20649_check_whoami(dev)) {
        return ICM_ERR;
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        dev->last_stat_user_ctrl = icm20649_read_register(dev, INV2REG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (dev->last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
        	dev->last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            icm20649_write_register(dev, INV2REG_USER_CTRL,
            		dev->last_stat_user_ctrl);
            HAL_Delay(10);
        }

        /* reset device */
        icm20649_write_register(dev, INV2REG_PWR_MGMT_1,
        		BIT_PWR_MGMT_1_DEVICE_RESET);
        HAL_Delay(100);

        /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
         * done just after the device is reset) */
        dev->last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
        icm20649_write_register(dev, INV2REG_USER_CTRL, dev->last_stat_user_ctrl);

        // Wake up device and select Auto clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        icm20649_write_register(dev, INV2REG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_AUTO);
        HAL_Delay(5);

        // check it has woken up
        if (icm20649_read_register(dev,
        		INV2REG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_AUTO) {
            break;
        }

        HAL_Delay(10);
        if (icm20649_data_ready(dev)) {
            break;
        }
    }

    spi_set_speed(dev->spi_desc->handle, SPEED_HIGH);

    if (tries == 5) {
        //hal.console->printf("Failed to boot Invensense 5 times\n");
        return ICM_INIT;
    }

    // Specific for ICM20649)
    dev->clip_limit = 29.5f * GRAVITY_MSS;
    dev->accel_scale = GRAVITY_MSS / 1024.0f;

    return ICM_OK;
}

/*! Resets fifo config. Does not change last saved params (no params set after init) !*/
ICM20649_StatusTypeDef icm20649_fifo_reset(struct icm20649_dev *dev)
{
    uint8_t user_ctrl = dev->last_stat_user_ctrl;
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_EN);

    spi_set_speed(dev->spi_desc->handle, SPEED_LOW);
    icm20649_write_register(dev, INV2REG_FIFO_EN_2, 0);
    icm20649_write_register(dev, INV2REG_USER_CTRL, user_ctrl);
    icm20649_write_register(dev, INV2REG_FIFO_RST, 0x0F);
    icm20649_write_register(dev, INV2REG_FIFO_RST, 0x00);
    icm20649_write_register(dev, INV2REG_USER_CTRL,
    		user_ctrl | BIT_USER_CTRL_FIFO_EN);
    icm20649_write_register(dev, INV2REG_FIFO_EN_2,
    		BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
            BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN);
    HAL_Delay(1);
    spi_set_speed(dev->spi_desc->handle, SPEED_HIGH);
    dev->last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;

    //notify_accel_fifo_reset(_accel_instance);
    //notify_gyro_fifo_reset(_gyro_instance);

    return ICM_OK; //TODO maybe err handling?
}

/*! Read device ID and check if it is correct !*/
ICM20649_StatusTypeDef icm20649_check_whoami(struct icm20649_dev *dev)
{

    uint8_t whoami = icm20649_read_register(dev, INV2REG_WHO_AM_I);

    icm20649_read_block(dev, INV2REG_WHO_AM_I, &whoami, 1);

    switch (whoami) {
    case INV2_WHOAMI_ICM20648:
        return ICM_OK;
    case INV2_WHOAMI_ICM20948:
        return ICM_OK;
    case INV2_WHOAMI_ICM20649:
        return ICM_OK;
    }
    // not a value WHOAMI result
    return ICM_ERR;
}

/*! Takes sample list, scales and one by one places it in the destination. !*/
bool icm20649_accumulate(struct icm20649_dev *dev, uint8_t *samples, uint8_t n_samples)
{
	for (uint8_t i = 0; i < n_samples; i++) {
		const uint8_t *data = samples + INV2_SAMPLE_SIZE * i;
		Vector3f accel, gyro;

		accel.x = int16_val(data, 1) * dev->accel_scale;
		accel.y = int16_val(data, 0) * dev->accel_scale;
		accel.z = -int16_val(data, 2) * dev->accel_scale;

		int16_t t2 = int16_val(data, 6);
		if (!icm20649_check_raw_temp(dev, t2)) {
			print_error(dev->huart, "temp reset IMU");
			icm20649_fifo_reset(dev);
			return false;
		}
		float temp = t2 * temp_sensitivity + temp_zero;

		gyro.x = int16_val(data, 4) * GYRO_SCALE;
		gyro.y = int16_val(data, 3) * GYRO_SCALE;
		gyro.z = -int16_val(data, 5) * GYRO_SCALE;

		dev->accum.accel.x = accel.x;
		dev->accum.accel.y = accel.y;
		dev->accum.accel.z = accel.z;
		dev->accum.gyro.x = gyro.x;
		dev->accum.gyro.y = gyro.y;
		dev->accum.gyro.z = gyro.z;

		dev->accum.temp = temp;
	}
	return true;
}

bool icm20649_accumulate_sensor_rate_sampling(struct icm20649_dev *dev, uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    //int32_t unscaled_clip_limit = dev->clip_limit / dev->accel_scale;
    //bool clipped = false;
    bool ret = true;

    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + INV2_SAMPLE_SIZE * i;

        // use temperature to detect FIFO corruption
        int16_t t2 = int16_val(data, 6);
        if (!icm20649_check_raw_temp(dev, t2)) {
			print_error(dev->huart, "temp reset IMU");
			icm20649_fifo_reset(dev);
            ret = false;
            break;
        }
        tsum += t2;
        if (dev->accum.gyro_count % 2 == 0) {
            // accel data is at 4kHz or 1kHz
            Vector3f a;
            a.x = int16_val(data, 1);
            a.y = int16_val(data, 0);
            a.z = -int16_val(data, 2);

            //_accum.accel += _accum.accel_filter.apply(a);
    		dev->accum.accel.x += a.x;
    		dev->accum.accel.y += a.y;
    		dev->accum.accel.z += a.z;

            Vector3f a2;
            a2.x = a.x * dev->accel_scale;
            a2.y = a.y * dev->accel_scale;
            a2.z = a.z * dev->accel_scale;
            //_notify_new_accel_sensor_rate_sample(_accel_instance, a2);

            dev->accum.accel_count++;

            if (dev->accum.accel_count % dev->accel_fifo_downsample_rate == 0) {
            	dev->accum.accel.x *= dev->fifo_accel_scale;
            	dev->accum.accel.y *= dev->fifo_accel_scale;
            	dev->accum.accel.z *= dev->fifo_accel_scale;
                //_rotate_and_correct_accel(_accel_instance, _accum.accel);
                //_notify_new_accel_raw_sample(_accel_instance, _accum.accel, 0, false);

                //TODO temp for debug printing
                dev->accel_print.x = dev->accum.accel.x;
                dev->accel_print.y = dev->accum.accel.y;
                dev->accel_print.z = dev->accum.accel.z;

            	dev->accum.accel.x = 0.0;
				dev->accum.accel.y = 0.0;
				dev->accum.accel.z = 0.0;
                dev->accum.accel_count = 0;
                // we assume that the gyro rate is always >= and a multiple of the accel rate
                dev->accum.gyro_count = 0;
            }
        }

        dev->accum.gyro_count++;

        Vector3f g;
        g.x = int16_val(data, 4);
        g.y = int16_val(data, 3);
        g.z = -int16_val(data, 5);

        Vector3f g2;
        g2.x = g.x * GYRO_SCALE;
        g2.y = g.y * GYRO_SCALE;
        g2.z = g.z * GYRO_SCALE;
        //_notify_new_gyro_sensor_rate_sample(_gyro_instance, g2);

		dev->accum.gyro.x += g.x;
		dev->accum.gyro.y += g.y;
		dev->accum.gyro.z += g.z;

        if (dev->accum.gyro_count % dev->gyro_fifo_downsample_rate == 0) {
            dev->accum.gyro.x *= dev->fifo_gyro_scale;
            dev->accum.gyro.y *= dev->fifo_gyro_scale;
            dev->accum.gyro.z *= dev->fifo_gyro_scale;
            //_rotate_and_correct_gyro(_gyro_instance, _accum.gyro);
            //_notify_new_gyro_raw_sample(_gyro_instance, _accum.gyro);

            //TODO temp for debug printing
            dev->gyro_print.x = dev->accum.gyro.x;
            dev->gyro_print.y = dev->accum.gyro.y;
            dev->gyro_print.z = dev->accum.gyro.z;

            dev->accum.gyro.x = 0.0;
			dev->accum.gyro.y = 0.0;
			dev->accum.gyro.z = 0.0;
        }
    }

    if (ret) {
    	float tsumcast = tsum;
        float temp = (tsumcast/n_samples) * temp_sensitivity + temp_zero;
        //_temp_filtered = _temp_filter.apply(temp);
        dev->accum.temp = temp;
    }
    return ret;
}

/*! Reads fifo buffer and saves data. !*/
void icm20649_read_fifo(struct icm20649_dev *dev)
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t *rx = dev->fifo_buffer;
    bool need_reset = false;

    if (icm20649_read_block(dev, INV2REG_FIFO_COUNTH, rx, 2)) {
        //goto check_registers;
    	print_error(dev->huart, "problem check registers");
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / INV2_SAMPLE_SIZE;

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        //goto check_registers;
    	print_error(dev->huart, "problem check registers");
    }

    /*
      testing has shown that if we have more than 32 samples in the
      FIFO then some of those samples will be corrupt. It always is
      the ones at the end of the FIFO, so clear those with a reset
      once we've read the first 24. Reading 24 gives us the normal
      number of samples for fast sampling at 400Hz
     */

    if (n_samples > 32) {
        need_reset = true;
        n_samples = 24;
    }
    while (n_samples > 0) {
    	uint8_t n = 0;
        if (n_samples < INV2_FIFO_BUFFER_LEN) {
        	n = n_samples;
        } else {
        	n = INV2_FIFO_BUFFER_LEN;
        }

        //I changed the code so it probably doesnt "keep things nicely setup for DMA"
        //whatever that means
        //TODO err handlin
        memset(rx, 0, n * INV2_SAMPLE_SIZE);
        if (icm20649_read_block(dev, INV2REG_FIFO_R_W, rx, n * INV2_SAMPLE_SIZE))
        	print_error(dev->huart, "Problem reading fifo");

        if (dev->fast_sampling) {
        	if (!icm20649_accumulate_sensor_rate_sampling(dev, rx, n)) {
        		print_error(dev->huart, "accumulate (fast) error");
        		break;
        	}
        } else {
        	if (!icm20649_accumulate(dev, rx, n)) {
        		print_error(dev->huart, "accumulate (slow) error");
        		break;
        	}
        }
        n_samples -= n;
    }

    if (need_reset) {
        print_error(dev->huart, "fifo reset");
        icm20649_fifo_reset(dev);
    }

//check_registers: //TODO
	//print_error(dev->huart, "problem check registers");
}

/*! Initializes sensor settings !*/
void icm20649_start(struct icm20649_dev *dev)
{
	spi_set_speed(dev->spi_desc->handle, SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    icm20649_write_register(dev, INV2REG_PWR_MGMT_2, 0x00);
    HAL_Delay(1);

    // always use FIFO
    icm20649_fifo_reset(dev);

    // setup on-sensor filtering and scaling
    // TODO for now lets stick to the slow sampling
    icm20649_set_filter_and_scaling(dev);

    // indicate what multiplier is appropriate for the sensors'
    // readings to fit them into an int16_t:
    //_set_raw_sample_accel_multiplier(_accel_instance, multiplier_accel);

    // set sample rate to 1.125KHz
    icm20649_write_register(dev, INV2REG_GYRO_SMPLRT_DIV, 0);
    HAL_Delay(1);

    // configure interrupt to fire when new data arrives
    //_register_write(INV2REG_INT_ENABLE_1, 0x01);
    //hal.scheduler->delay(1);

    // now that we have initialised, we set the bus speed to high
    spi_set_speed(dev->spi_desc->handle, SPEED_HIGH);

    // setup sensor rotations from probe()
    //set_gyro_orientation(_gyro_instance, _rotation);
    //set_accel_orientation(_accel_instance, _rotation);

    // setup scale factors for fifo data after downsampling
    dev->fifo_accel_scale = dev->accel_scale / dev->accel_fifo_downsample_rate;
    dev->fifo_gyro_scale = GYRO_SCALE / dev->gyro_fifo_downsample_rate;

    // allocate fifo buffer
    dev->fifo_buffer = (uint8_t *)malloc(INV2_FIFO_BUFFER_LEN * INV2_SAMPLE_SIZE);
    if (dev->fifo_buffer == NULL) {
        print_error(dev->huart, "malloc fail, aborting");
        while (1);
    }

    // set accel filter parameters
    dev->accel_filter_sample_freq = 4500;
    dev->accel_filter_cutoff_freq = 188;
    float dt = 1.0/dev->accel_filter_sample_freq;
    float rc = 1.0f/(M_TWOPI*dev->accel_filter_cutoff_freq);
    float alpha = dt / (dt + rc);
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    dev->accel_filter_alpha = alpha;

    // start the timer process to read samples
    //_dev->register_periodic_callback(1265625UL / _gyro_backend_rate_hz,
    //FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensensev2::_poll_data, void));
}

void icm20649_set_filter_and_scaling(struct icm20649_dev *dev)
{
    uint8_t gyro_config = BITS_GYRO_FS_2000DPS_20649;
    uint8_t accel_config = BITS_ACCEL_FS_30G_20649;

    // assume 1.125kHz sampling to start
    dev->gyro_fifo_downsample_rate = 1;
    dev->accel_fifo_downsample_rate = 1;
    dev->gyro_backend_rate_hz = 1125;
    dev->accel_backend_rate_hz =  1125;
    uint8_t fast_sampling_rate = 1;

    if (dev->fast_sampling) {
        // constrain the gyro rate to be at least the loop rate
    	// I dont implement possibility to explicidly set gyro rate,
    	// just give what main loop requires
        if (dev->loop_rate_hz > 1125) {
        	fast_sampling_rate = 2;
        }
        if (dev->loop_rate_hz > 2250) {
        	fast_sampling_rate = 4;
        }
        // max for sensor is 8 samples in 1/1125 sec, so 9khz, no lpf nor downsampling
        fast_sampling_rate = 8;

        // calculate rate we will be giving gyro samples to the backend
        dev->gyro_fifo_downsample_rate = 8 / fast_sampling_rate;
        dev->gyro_backend_rate_hz *= fast_sampling_rate;

        // calculate rate we will be giving accel samples to the backend
        dev->accel_fifo_downsample_rate = (4 / fast_sampling_rate > 1) ?
        		(4 / fast_sampling_rate) : 1;
        dev->accel_backend_rate_hz *= (fast_sampling_rate < 4) ?
        		fast_sampling_rate : 4;

        /* set divider for internal sample rate to 0x1F when fast
         sampling enabled. This reduces the impact of the slave
         sensor on the sample rate.
         */
        icm20649_write_register(dev, INV2REG_I2C_SLV4_CTRL, 0x1F);
    }

    if (dev->fast_sampling) {
        // this gives us 9kHz sampling on gyros
        gyro_config |= BIT_GYRO_NODLPF_9KHZ;
        accel_config |= BIT_ACCEL_NODLPF_4_5KHZ;
    } else {
        // limit to 1.125kHz (must, if not on SPI)
        gyro_config |= BIT_GYRO_DLPF_ENABLE | (GYRO_DLPF_CFG_188HZ << GYRO_DLPF_CFG_SHIFT);
        accel_config |= BIT_ACCEL_DLPF_ENABLE | (ACCEL_DLPF_CFG_265HZ << ACCEL_DLPF_CFG_SHIFT);
    }

    icm20649_write_register(dev, INV2REG_GYRO_CONFIG_1, gyro_config);
    icm20649_write_register(dev, INV2REG_ACCEL_CONFIG, accel_config);
    icm20649_write_register(dev, INV2REG_FIFO_MODE, 0xF);
}

/*! Checks whether raw temp is okay. If not, fifo data is corrupt !*/
bool icm20649_check_raw_temp(struct icm20649_dev *dev, int16_t t2)
{
    if (abs(t2 - dev->raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (!icm20649_read_block(dev, INV2REG_TEMP_OUT_H, trx, 2)) {
    	dev->raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - dev->raw_temp) < 400);
}
