# BMP280 SPI communication 

See "example_bmp.c" for utilization details.

To switch between I²C and SPI, set the last parameter of `bmp_init` to either USE_SPI or USE_I2C

Datasheet: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf

Utilizes [Bosch Sensortec's](https://github.com/BoschSensortec/BMP280_driver) drivers for coefficient calculations

## Performance details

- Utilizes a constant amount of memory (capped at <1MB)
- CPU utilization scales non-linearly (approx. to log with base 1.5) for I2C
- CPU utilization scales non-linearly (approx. to log with base 1.6) for SPI, until higher (>200 Hz) polling rates are used.
- No memory leaks detected

