#include "bme280.h"

uint8_t bme280_init()
{
	uint8_t err = bme_id;
	HAL_Delay(2);
	HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, &err, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, BME_ADDRESS, &err, 1, 10);
	if (err == 0x60)
	{
		// Config oversampling hygro
		uint8_t data[3] = { bme_ctrlHum, 0x01, 0 };
		HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, data, 2, 10);
		// Config oversampling temp & press
		data[0] = bme_ctrlMeas;
		data[1] = 0b001 << 5 | 0b001 << 2 | 0b11;
		HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, data, 2, 10);
		// Config 20ms, filter off
		data[0] = bme_config;
		data[1] = 0b111 << 5 | 0b000 << 2;
		HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, data, 2, 10);
		err = 0;
	}
	return err;
}

void bme280_read(bme_structTypedef *data)
{
	uint8_t rawData[8];
	uint8_t addrRd = bme_press; //0xF7
	HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, &addrRd, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, BME_ADDRESS, rawData, 8, 10);
	// pression msb, lsb, xlsb, temp msb, lsb, xlsb, hum msb, lsb
	data->press = (uint32_t)rawData[0]<<12 | (uint32_t)rawData[1]<<4 | rawData[2]>>4;
	data->temp = (uint32_t)rawData[3]<<12 | (uint32_t)rawData[4]<<4 | rawData[5]>>4;
	data->hum = (uint16_t)rawData[6]<<8 | rawData[7];

	addrRd = 0x88;
	uint8_t compensationData[19];
	HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, &addrRd, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, BME_ADDRESS, compensationData, 6, 10);

	int32_t t_fine;
	int64_t var1, var2;
	uint16_t dig[9];
	dig[0] = (uint16_t)compensationData[1]<<8 | compensationData[0];
	dig[1] = (uint16_t)compensationData[3]<<8 | compensationData[2];
	dig[2] = (uint16_t)compensationData[5]<<8 | compensationData[4];

	var1 = ((((data->temp>>3) - ((int32_t)dig[0]<<1))) * ((int32_t)dig[1])) >> 11;
	var2 = (((((data->temp>>4) - ((int32_t)dig[0])) * ((data->temp>>4) - ((int32_t)dig[0]))) >> 12) *	((int32_t)dig[2])) >> 14;
	t_fine = var1 + var2;
	data->temp = (t_fine * 5 + 128) >> 8;

	addrRd = 0x8E;
	HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, &addrRd, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, BME_ADDRESS, compensationData, 19, 10);
	dig[0] = compensationData[0] | (uint16_t)compensationData[1]<<8;
	dig[1] = compensationData[2] | (uint16_t)compensationData[3]<<8;
	dig[2] = compensationData[4] | (uint16_t)compensationData[5]<<8;
	dig[3] = compensationData[6] | (uint16_t)compensationData[7]<<8;
	dig[4] = compensationData[8] | (uint16_t)compensationData[9]<<8;
	dig[5] = compensationData[10] | (uint16_t)compensationData[11]<<8;
	dig[6] = compensationData[12] | (uint16_t)compensationData[13]<<8;
	dig[7] = compensationData[14] | (uint16_t)compensationData[15]<<8;
	dig[8] = compensationData[16] | (uint16_t)compensationData[17]<<8;
	int64_t p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig[5];
	var2 = var2 + ((var1*(int64_t)dig[4])<<17);
	var2 = var2 + (((int64_t)dig[3])<<35);
	var1 = ((var1 * var1 * (int64_t)dig[2])>>8) + ((var1 * (int64_t)dig[1])<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig[0])>>33;
	if (var1 == 0)
	{
		var1=1; // avoid exception caused by division by zero
	}
	p = 1048576-data->press;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig[8]) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig[7]) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig[6])<<4);
	data->press = p>>8;

	addrRd = 0xE1;
	HAL_I2C_Master_Transmit(&hi2c1, BME_ADDRESS, &addrRd, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, BME_ADDRESS, compensationData, 8, 10);
	dig[0] = compensationData[0] | (uint16_t)compensationData[1]<<8;
	dig[1] = compensationData[3] | (uint16_t)compensationData[4]<<8;
	dig[2] = compensationData[5] | (uint16_t)compensationData[6]<<8;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((data->hum << 14) - (((int32_t)dig[1]) << 20) - (((int32_t)dig[2]) * var1)) + ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)compensationData[7])) >> 10) * (((var1 *
	((int32_t)compensationData[2])) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig[0]) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)compensationData[18])) >> 4));
//	var1 = (var1 < 0 ? 0 : var1);
//	var1 = (var1 > 419430400 ? 419430400 : var1);
	data->hum = var1>>12;
}
