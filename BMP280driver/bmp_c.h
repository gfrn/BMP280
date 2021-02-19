#ifndef BMP_C_H
#define BMP_C_H

#define USE_SPI 1
#define USE_I2C 0

#include "bmp280.h"

int bmp_init(struct bmp280_dev *bmp, struct bmp280_config *conf, struct bmp280_uncomp_data *ucomp_data, uint8_t use_spi);
void close_all(struct bmp280_dev bmp);
int i2c_init(struct bmp280_dev *bmp);
int spi_init(struct bmp280_dev *bmp);
int get_values(double *temp, double *pres, struct bmp280_dev *bmp, struct bmp280_uncomp_data *ucomp_data);

#endif