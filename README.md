# BMP280 SPI communication 

See "bmp.c" for utilization details.

To switch between I²C and SPI, comment out `spi_init()`/`i2c_init()`

Datasheet: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf

Utilizes [Bosch Sensortec's](https://github.com/BoschSensortec/BMP280_driver) drivers for coefficient calculations

## Performance details

- Utilizes a constant amount of memory (capped at <1MB)
- CPU utilization scales non-linearly (approx. to log with base 1.5) for I2C
- CPU utilization scales non-linearly (approx. to log with base 1.6) for SPI, until higher (>200 Hz) polling rates are used.
- No memory leaks detected

