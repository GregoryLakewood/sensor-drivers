
#pragma once

#define OP_REG(r)				((((uint16_t)0x00U) << 8) | (r))
#define OUT_REG(r)				((((uint16_t)0x3FU) << 8) | (r))

#define BIT_READ_FLAG			(uint16_t)0x80 << 7
#define BIT_WRITE_FLAG			~((uint16_t)0x80 << 7)

#define AS5048_NOP				OP_REG(0x00U)
#define AS5048_ERR_FLAG			OP_REG(0x01U)
#define AS5048_PROG_CONTROL		OP_REG(0x03U)
#define AS5048_ZERO_H			OP_REG(0x16U)
#define AS5048_ZERO_L			OP_REG(0x17U)

#define AS5048_DIAG_AGC			OUT_REG(0xFDU)
#define AS5048_MAG				OUT_REG(0xFEU)
#define AS5048_ANG				OUT_REG(0xFFU)
