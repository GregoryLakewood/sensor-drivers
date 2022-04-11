/******************************************************************************
 *   @file   adxrs4535048.c
 *   @brief  Source file of adxrs453 Driver.
******************************************************************************/

#include "adxrs453.h"

/*! Takes ref to uint32_t value and sets LSB to assert parity ODD.
 * Ref, so we can just return if already satisfied.
 * Used only for sending commands. !*/
void adxrs453_change_parity(uint32_t *value)
{
	//TODO wgle czy nie powinno tutaj byc tak ze nie zmieniamy
	//na poczatku parity na 0?
	//przeciez to dodatkowa operacja lulz
	// *value &= ~(uint32_t)(0x01U); //assert parity bit 0
	uint32_t x = *value;
	x ^= x >> 16;
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	if (x & 1) { //1 if parity is odd
		return;
	} *value = *value | 0x01U;
}

/*! Copies uint16_t value, checks parity, returns TRUE/1 if ODD.
 * Checks for both parity bits so it should be used
 * ONLY for incoming responses. !*/
_Bool adxrs453_check_parity(uint32_t value)
{
	//check p0
	uint16_t first_half = (uint16_t)(value >> 16);
	first_half ^= first_half >> 8;
	first_half ^= first_half >> 4;
	first_half ^= first_half >> 2;
	first_half ^= first_half >> 1;
	if ((first_half & 1) == 0) return false;
	//check p1
	value ^= value >> 16;
	value ^= value >> 8;
	value ^= value >> 4;
	value ^= value >> 2;
	value ^= value >> 1;
	return value & 1;
}

/*! Initializes the ADXRS453 and checks if the device is present. !*/
ADXRS453_StatusTypeDef adxrs453_init(struct adxrs453_dev *dev)
{
	ADXRS453_StatusTypeDef status = ADX_OK;

	uint8_t send_buffer[4] = {0, 0, 0, 0};
	uint8_t recv_buffer[4] = {0, 0, 0, 0};

	send_buffer[0] = 0x20; //rq sensor data
	send_buffer[3] = 0x3; //CHK sq init

	HAL_Delay(100); //boot delay

	//request self-test
	status = spi_write(dev->spi_desc, send_buffer, 4);

	HAL_Delay(50);
	send_buffer[3] = 0x0; //end CHK

	//this reply can be discarded
	status = spi_write(dev->spi_desc, send_buffer, 4);

	HAL_Delay(50);

	//self-test result (2 same responses)
	status = spi_write_and_read_duplex(dev->spi_desc, send_buffer, 4,
			recv_buffer, 4);
	status = spi_write_and_read_duplex(dev->spi_desc, send_buffer, 4,
			recv_buffer, 4);
	//dev should res with ...FF or ...FE

	//now self-test end, res should be ...00 or ...01
	status = spi_write_and_read_duplex(dev->spi_desc, send_buffer, 4,
			recv_buffer, 4);

	//TODO assert no err flags

	// read the value of the ADXRS453 ID register
	uint16_t adxrs453_id = adxrs453_get_register_value(dev, ADXRS453_REG_PID);
	if((adxrs453_id >> 8) != 0x52)
		status = -1;

	return status;
}


/*! Reads and returns the value of a register. !*/
uint16_t adxrs453_get_register_value(struct adxrs453_dev *dev,
		uint8_t register_address)
{
	uint8_t send_buffer[4] = {0, 0, 0, 0};
	uint8_t recv_buffer[4] = {0, 0, 0, 0};

	send_buffer[0] = ADXRS453_READ | (register_address >> 7);
	send_buffer[1] = (register_address << 1);

	uint32_t command = ((uint32_t)send_buffer[0] << 24) |
		  ((uint32_t)send_buffer[1] << 16) |
		  ((uint16_t)send_buffer[2] << 8) |
		  send_buffer[3];
	adxrs453_change_parity(&command);
	send_buffer[3] = (uint8_t)command;

	spi_write(dev->spi_desc, send_buffer, 4);
	spi_read(dev->spi_desc, recv_buffer, 4);

	uint32_t response = ((uint32_t)recv_buffer[0] << 24) |
		  ((uint32_t)recv_buffer[1] << 16) |
		  ((uint16_t)recv_buffer[2] << 8) |
		  recv_buffer[3];
	if (!adxrs453_check_parity(response)) return ADX_PARITY; //TODO

	uint16_t register_value = ((uint16_t)recv_buffer[1] << 11) |
			 ((uint16_t)recv_buffer[2] << 3) |
			 (recv_buffer[3] >> 5);

	return register_value;
}

/*! Writes data into a register. Checks if value is the same as requested. !*/
ADXRS453_StatusTypeDef adxrs453_set_register_value(struct adxrs453_dev *dev,
				 uint8_t register_address,
				 uint16_t register_value)
{
	uint8_t send_buffer[4] = {0, 0, 0, 0};
	uint8_t recv_buffer[4] = {0, 0, 0, 0};

	send_buffer[0] = ADXRS453_WRITE | (register_address >> 7);
	send_buffer[1] = (register_address << 1) |
			 (register_value >> 15);
	send_buffer[2] = (register_value >> 7);
	send_buffer[3] = (register_value << 1);

	uint32_t command = ((uint32_t)send_buffer[0] << 24) |
		  ((uint32_t)send_buffer[1] << 16) |
		  ((uint16_t)send_buffer[2] << 8) |
		  send_buffer[3];
	adxrs453_change_parity(&command);
	send_buffer[3] = (uint8_t)command;

	spi_write(dev->spi_desc, send_buffer, 4);
	spi_read(dev->spi_desc, recv_buffer, 4);

	uint32_t response = ((uint32_t)recv_buffer[0] << 24) |
		  ((uint32_t)recv_buffer[1] << 16) |
		  ((uint16_t)recv_buffer[2] << 8) |
		  recv_buffer[3];
	if (!adxrs453_check_parity(response)) return ADX_PARITY;

	uint16_t response_data = ((uint16_t)recv_buffer[1] << 11) |
			 ((uint16_t)recv_buffer[2] << 3) |
			 (recv_buffer[3] >> 5);

	if (response_data != register_value) return ADX_ERR;
	return ADX_OK;
}

/*! Reads the sensor data with full duplex mode.
 * Writes error_flags in dev if any read.
 * There EF should be handled outside of this fn. !*/
float adxrs453_get_sensor_data(struct adxrs453_dev *dev)
{
	uint8_t send_buffer[4] = {0, 0, 0, 0};
	uint8_t recv_buffer[4] = {0, 0, 0, 0};
	uint16_t data_value = 0;

	send_buffer[0] = ADXRS453_SENSOR_DATA;
	//assuming sq = 000
	//CHK = 0

	spi_write_and_read_duplex(dev->spi_desc, send_buffer, 4, recv_buffer, 4);

	uint8_t error_flags = recv_buffer[3] & (~0x01);
	dev->error_flags = error_flags;

	// for debug
	for (int i = 1; i < 8; ++i) {
		if ((error_flags >> i) & 1) {
			printf("Error Flag Set: %d\n", i);
		}
	}

	data_value = ((uint16_t)recv_buffer[0] << 14) |
			 ((uint16_t)recv_buffer[1] << 6) |
			 (recv_buffer[2] >> 2);

	if(data_value < 0x8000)
		return ((float)data_value / 80.0);
	else
		return (-1) * ((float)(0xFFFF - data_value + 1) / 80.0);
}

/*! Reads the rate data and converts it to degrees/second. !*/
float adxrs453_get_rate(struct adxrs453_dev *dev)
{
	uint16_t register_value = 0;
	float rate = 0.0;

	register_value = adxrs453_get_register_value(dev, ADXRS453_REG_RATE);

	/*!< If data received is in positive degree range */
	if(register_value < 0x8000)
		rate = ((float)register_value / 80);
	/*!< If data received is in negative degree range */
	else
		rate = (-1) * ((float)(0xFFFF - register_value + 1) / 80.0);

	return rate;
}

/*! Reads the temperature sensor data and converts it to degrees Celsius. !*/
float adxrs453_get_temperature(struct adxrs453_dev *dev)
{
	uint32_t register_value = 0;
	float temperature = 0;

	register_value = adxrs453_get_register_value(dev, ADXRS453_REG_TEM);
	register_value = (register_value >> 6) - 0x31F;
	temperature = (float) register_value / 5;

	return temperature;
}
