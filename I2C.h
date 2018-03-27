


void I2C_Init(void);

int8_t I2C_Write_String(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t byteCount);

int8_t I2C_Read_String(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t byteCount);

