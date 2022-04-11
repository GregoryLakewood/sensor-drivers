/******************************************************************************
 *   @file   as5048.c
 *   @brief  Source file of as5048 Driver.
******************************************************************************/

#include "as5048.h"

/*! Takes ref to uint16_t value and sets MSB to assert parity EVEN.
 * Ref, so we can just return if already satisfied. !*/
void as5048_change_parity(uint16_t *value)
{
	*value &= ~((uint16_t)(0x01U) << 15); //assert parity bit 0
	uint16_t x = *value;
	//x &= ~((uint16_t)(0x01U) << 15);
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	if (x & 1) { //1 if parity is odd
		*value = (*value | ((uint16_t)(0x01U) << 15));
	} return;
}

/*! Copies uint16_t value, checks parity, returns TRUE/1 if ODD !*/
_Bool as5048_check_parity(uint16_t value)
{
	value ^= value >> 8;
	value ^= value >> 4;
	value ^= value >> 2;
	value ^= value >> 1;
	return value & 1;
}

/*! Reads and returns value from register.
 * Checks parity and error flags.
 * Return value without parity and EF bits. !*/
uint16_t as5048_read_register(struct as5048_dev *dev, uint16_t reg)
{
	uint8_t buf[4] = { 0, 0, 0, 0 };
	reg |= BIT_READ_FLAG;
	as5048_change_parity(&reg);
	buf[0] = (reg >> 8);
	buf[1] = reg;
	spi_write(dev->spi_desc, buf, 2);
	spi_read(dev->spi_desc, &buf[2], 2);
	//MSB-1 bit indicates error flag
	if (buf[2] & (0x01 << 6)) {
		print_error(dev->huart, "Error Flag asserted in Read Response");
		dev->error_flag = true;
		return AS_ERR;
	}
	uint16_t value = ((uint16_t)(buf[2]) << 8) | buf[3];
	if (as5048_check_parity(value)) {
		print_error(dev->huart, "Parity Error in Read Response");
		dev->error_flag = true;
		return AS_PARITY_ERROR;
	}
	//EF & PARITY bits are not needed in return
	value &= ~((uint16_t)0x03 << 14);
	return value;
}

/*! Writes data to register. Dev returns data written.
 * AS_OK if all ok, and (return data) == (data) !*/
AS5048_StatusTypeDef as5048_write_register(struct as5048_dev *dev,
		uint16_t reg, uint16_t data)
{
	uint8_t buf[6] = { 0, 0, 0, 0, 0, 0 };
	reg &= BIT_WRITE_FLAG;
	as5048_change_parity(&reg);
	buf[0] = (reg >> 8);
	buf[1] = reg;
	data &= BIT_WRITE_FLAG;
	as5048_change_parity(&data);
	buf[2] = (data >> 8);
	buf[3] = data;
	spi_write(dev->spi_desc, buf, 2);
	spi_write(dev->spi_desc, &buf[2], 2);
	spi_read(dev->spi_desc, &buf[4], 2);
	uint16_t check = ((uint16_t)(buf[4]) << 8) | buf[5];
	//MSB-1 bit indicates error flag
	if (buf[4] & (0x01 << 6)) {
		print_error(dev->huart, "Error Flag asserted in Write Response");
		dev->error_flag = true;
		return AS_ERR;
	}
	if (as5048_check_parity(check)) {
		print_error(dev->huart, "Parity Error in Write Response");
		dev->error_flag = true;
		return AS_PARITY_ERROR;
	}
	if ((buf[2] != buf[4]) || (buf[3] != buf[5])) {
		print_error(dev->huart, "Data and check not the same");
		return AS_WRITE_FAIL;
	}
	return AS_OK;
}

/*! Reads error register. Returns enum err type, err if err persists
 * or OK if EF wasnt assested to begin with !*/
AS5048_StatusTypeDef as5048_clear_error_flag(struct as5048_dev *dev)
{
	AS5048_StatusTypeDef status;
	if (dev->error_flag != false) {
		print_error(dev->huart,
				"Error Flag not asserted, yet Clear Flag called");
		status = AS_OK;
		dev->error_flag = false;
	}
	uint8_t buf[4] = { 0x40, 0x01, 0, 0 }; //msg is hardcoded
	spi_write(dev->spi_desc, buf, 2);
	spi_read(dev->spi_desc, &buf[2], 2);
	if (!(buf[2] &= 0x40)) {
		print_error(dev->huart,
				"No EF in response, yet Clear Flag called");
		dev->error_flag = false;
		return AS_OK; //EF wasnt asserted to begin with
	}
	switch (buf[3] &= 0x07) {
		case 0x04:
			print_error(dev->huart, "Parity Error");
			status = AS_PARITY_ERROR;
			break;
		case 0x02:
			print_error(dev->huart, "Command Invalid");
			status = AS_COMMAND_INVALID;
			break;
		case 0x01:
			print_error(dev->huart, "Framing Error");
			status = AS_FRAMING_ERROR;
			break;
		default:
			status = AS_ERR; //unrecognised error occured
	}
	spi_read(dev->spi_desc, &buf[2], 2);
	if (buf[2] &= 0x40) {
		return AS_ERR; //EF persists
	} else {
		dev->error_flag = false;
		return status;
	}
}

/*! Read and return what zero offset is programmed onto device !*/
uint16_t as5048_read_zero(struct as5048_dev *dev)
{
	uint8_t hi_byte = as5048_read_register(dev, AS5048_ZERO_H);
	uint8_t lo_byte = as5048_read_register(dev, AS5048_ZERO_L);
	uint16_t zero = ((uint16_t)hi_byte << 6) | lo_byte;
	return zero;
}

/*! Set zero offset - no programming onto device !*/
AS5048_StatusTypeDef as5048_set_zero(struct as5048_dev *dev, uint16_t zero)
{
	AS5048_StatusTypeDef status = AS_OK;
	uint16_t hi_byte = zero >> 6;
	uint16_t lo_byte = zero & (uint16_t)0x3F;
	status = as5048_write_register(dev, AS5048_ZERO_H, hi_byte);
	status = as5048_write_register(dev, AS5048_ZERO_L, lo_byte);
	uint16_t check = as5048_read_zero(dev);
	if (check != zero) return AS_ERR;
	return status;
}

/*! Just checks the diagnostic register, returns enumed error!*/
AS5048_StatusTypeDef as5048_diagnostics(struct as5048_dev *dev)
{
	AS5048_StatusTypeDef status = AS_OK;
	uint16_t value = as5048_read_register(dev, AS5048_DIAG_AGC);
	if (dev->error_flag) return AS_ERR;
	if (!((value >> 8) & 0x01)) return AS_OCF; //Offset Compensation Fail
	switch (value >> 8) {
		case 0x09U:
			status = AS_COMPH;
			print_error(dev->huart, "weak mag field");
			break;
		case 0x05U:
			status = AS_COMPL;
			print_error(dev->huart, "strong mag field");
			break;
		case 0x03U:
			status = AS_COF;
			print_error(dev->huart, "cordic overflow");
			break;
		case 0x01U:
			status = AS_OK;
			break;
		default:
			status = AS_ERR;
			print_error(dev->huart, "many errors occured");
	}
	return status;
}

/*! Returns Automatic Gain Control value.
 * 0 means strong mag field, 255 - weak. !*/
uint8_t as5048_read_agc(struct as5048_dev *dev)
{
	uint16_t value = as5048_read_register(dev, AS5048_DIAG_AGC);
	if (dev->error_flag) return AS_ERR;
	return (uint8_t)value;
}

/*! Returns magnitude in CORDIC. !*/
uint16_t as5048_read_magnitude(struct as5048_dev *dev)
{
	uint16_t value = as5048_read_register(dev, AS5048_MAG);
	if (dev->error_flag) return AS_ERR;
	return value;
}

/*! Returns calculated angle, offset included. Output in degrees !*/
float as5048_read_angle(struct as5048_dev *dev)
{
	uint16_t value = as5048_read_register(dev, AS5048_ANG);
	if (dev->error_flag) return AS_ERR;
	return (float)(value * 0.02197f);
}

/*! Returns angle, source PWM, offset included.
 * Output in degrees. Depends on PWM_SCALE macro !*/
float as5048_read_angle_pwm(uint16_t read_pwm_value) {
	/* this fnc is not written well, it depends on max count time, system
	 * clock, and many other params that are not taken into account
	 * angle calc is based on empirical measurements
	 * 0 deg = 230 cycles
	 * err = 172 cycles -> 1 bit = 14.33 cycles = 0.087890625 deg
	 * from up -> 0.00613333042568 deg = 1 cycle
	 * 0.006061626536 deg = 1 cycle
	 * 1 bit = +/- 14/15 cycles
	 * 360 deg = 59620 cycles
	 * period = 59720, maybe
	 */
	//TODO here should be pwm minus init

	return (float)read_pwm_value * PWM_SCALE;
}
