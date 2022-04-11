typedef enum Speed {
    SPEED_HIGH,
    SPEED_LOW,
} SPI_SpeedTypeDef;

typedef struct spi_desc {
	SPI_HandleTypeDef	*handle;	// SPI handle pointer
	uint16_t	pin;				// SPI CS pin
	GPIO_TypeDef	*port;			// SPI port
} spi_desc;

/*! Transmit only basic func for spi !*/
HAL_StatusTypeDef spi_write(struct spi_desc *desc, uint8_t *send, uint8_t send_len);

/*! Receive only basic func for spi !*/
HAL_StatusTypeDef spi_read(struct spi_desc *desc, uint8_t *recv, uint8_t recv_len);

/*! Function for full-duplex, simultaneous, spi transfer !*/
HAL_StatusTypeDef spi_write_and_read_duplex(struct spi_desc *desc, uint8_t *send, uint8_t send_len, uint8_t *recv, uint8_t recv_len);

/*! Basic function for semi-duplex spi transfer !*/
HAL_StatusTypeDef spi_write_and_read(struct spi_desc *desc, uint8_t *send, uint8_t send_len, uint8_t *recv, uint8_t recv_len);

/*! Simple function to print message on UART !*/
void print_error(UART_HandleTypeDef *huart, char *msg);

/*! Change speed of SPI - specifically prescaller !*/
_Bool spi_set_speed(SPI_HandleTypeDef *handle, SPI_SpeedTypeDef speed);
