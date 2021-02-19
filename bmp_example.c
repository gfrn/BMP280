///@file bmp.c

#include <stdio.h>

#include "BMP280driver/bmp_c.h"

int main(int argc, char *argv[])
{
    int8_t rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    double pres, temp;

    // IIR filter
    conf.filter = BMP280_FILTER_OFF;
    // Temperature over sampling
    conf.os_temp = BMP280_OS_2X;
    // Pressure over sampling
    conf.os_pres = BMP280_OS_16X;
    // Setting the output data period as 500us
    conf.odr = BMP280_ODR_0_5_MS;

    bmp_init(&bmp, &conf, &ucomp_data, USE_SPI);

    while (1)
    {
        // Getting the compensated temperature as floating point value
        rslt |= get_values(&temp, &pres, &bmp, &ucomp_data);

        printf("Temperature: %.2f C \r\nPressure: %.2f hPa \r\n", temp, pres / 100);

        bmp.delay_ms(1000);
    }

    // Unreachable code, here in case I decide to change it in the future
    close_all(bmp);
    return rslt;
}
