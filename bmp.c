///@file bmp.c

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>
#include <time.h>

#include <iobb.h>
#include "BMP280driver/bmp280.h"

#define AUTOCS 1

typedef struct
{
    int header;
    int pin;
} Pin;

const Pin cs_pin = {8, 11};

static const char *spi_device = "/dev/spidev0.0";
static const char *i2c_device = "/dev/i2c-2";
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;
static int fd;

/*!
 *  @brief Internal SPI transfer helper function.
 *
 *  @param[in] tx       : Pointer to the tx data buffer.
 *  @param[in] rx       : Pointer to the rx data buffer.
 *  @param[in] len      : No of bytes to write.
 *  @retval 0 -> Success
 *  @retval 1 -> Failure
 */
int8_t transfer(uint8_t const *tx, uint8_t const *rx, size_t len)
{
    int ret;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    if (!AUTOCS)
        pin_low(cs_pin.header, cs_pin.pin);

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    if (!AUTOCS)
        pin_high(cs_pin.header, cs_pin.pin);

    return ret < 0;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void bdelay_ms(uint32_t period_ms)
{
    struct timespec sleep;
    sleep.tv_sec = period_ms / 1000;
    sleep.tv_nsec = (period_ms % 1000) * 1000000L;
    while (clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep, &sleep))
        ;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] len          : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval 1 -> Failure 
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    uint8_t dt[2] = {reg_addr, reg_data[0]};
    return transfer(dt, dt, len + 1);
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in]  cs        : Chip select to enable the sensor.
 *  @param[in]  reg_addr  : Register address.
 *  @param[out] reg_data  : Pointer to the data buffer to store the read data.
 *  @param[in]  len    : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval 1 -> Failure
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int ret;
    if (reg_addr == BMP280_PRES_MSB_ADDR)
    {
        // Enable forced mode
        spi_reg_write(0, 0x74, "\x55", 2);
        uint8_t status[2] = {0};
        spi_reg_read(0, BMP280_STATUS_ADDR, status, 1);

        // Check if measurement operation is active
        while (status[0] & 0x08)
        {
            spi_reg_read(0, BMP280_STATUS_ADDR, status, 1);
            usleep(20);
        }
    }
    uint8_t t_len = len + 1;

    uint8_t dt[t_len];
    dt[0] = reg_addr;

    ret = transfer(dt, reg_data, t_len);

    // Shifts every byte left to compensate for the register addr in RX
    for (int i = 0; i < t_len; i++)
        reg_data[i] = reg_data[i + 1];

    return ret;
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval -1 -> Failure
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    uint8_t dt[2] = {reg_addr, reg_data[0]};
    return write(fd, dt, 2) < 0 ? -1 : 0;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval -1 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    if (reg_addr == BMP280_PRES_MSB_ADDR)
    {
        i2c_reg_write(0, BMP280_CTRL_MEAS_ADDR, "\x55", 1);

        uint8_t status[2] = {0, 0};
        i2c_reg_read(0, BMP280_STATUS_ADDR, status, 1);

        while (status[0] & 0x08)
        {
            i2c_reg_read(0, BMP280_STATUS_ADDR, status, 1);
            usleep(20);
        }
    }

    uint8_t dt[2] = {reg_addr, 0};
    write(fd, dt, 1);

    return read(fd, reg_data, length) < 0 ? -1 : 0;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 */
void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
            printf("Error [%d] : Null pointer error\r\n", rslt);
        else if (rslt == BMP280_E_COMM_FAIL)
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
            printf("Error [%d] : Device not found\r\n", rslt);
        else
            // For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
    }
}

/*!
 *  @brief Initializes SPI operations.
 *
 *  @param[in] bmp : BMP object.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval !0 -> Failure Info
 */
int spi_init(struct bmp280_dev *bmp)
{
    int ret = 0;
    fd = open(spi_device, O_RDWR);

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ret |= ioctl(fd, SPI_IOC_RD_MODE, &mode);

    ret |= ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ret |= ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);

    ret |= ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    ret |= ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);

    if (!AUTOCS)
    {
        iolib_init();
        iolib_setdir(cs_pin.header, cs_pin.pin, DigitalOut);
    }

    bmp->dev_id = 0;
    bmp->read = spi_reg_read;
    bmp->write = spi_reg_write;
    bmp->intf = BMP280_SPI_INTF;

    return ret;
}

/*!
 *  @brief Initializes I2C operations.
 *
 *  @param[in] bmp : BMP object.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval -1 -> Failure 
 */
int i2c_init(struct bmp280_dev *bmp)
{
    fd = open(i2c_device, O_RDWR);

    ioctl(fd, I2C_SLAVE, BMP280_I2C_ADDR_PRIM);

    bmp->dev_id = BMP280_I2C_ADDR_PRIM;
    bmp->read = i2c_reg_read;
    bmp->write = i2c_reg_write;
    bmp->intf = BMP280_I2C_INTF;

    return fd < 0 ? -1 : 0;
}

/*!
 *  @brief Finalizes all BMP280 operations
 *
 *  @param[in] bmp : BMP object.
 *
 *  @return void
 */
void close_all(struct bmp280_dev bmp)
{
    close(fd);
    iolib_free();
}

/*!
 *  @brief Initializes BMP280 operations.
 *
 *  @param[in] bmp : BMP object.
 *  @param[in] conf : BMP config object.
 *  @param[in] ucomp_data : uncompensated data
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval -1 -> Failure 
 */
int bmp_init(struct bmp280_dev *bmp, struct bmp280_config *conf, struct bmp280_uncomp_data *ucomp_data)
{
    int rslt;
    bmp->delay_ms = bdelay_ms;

    rslt = bmp280_init(bmp);
    print_rslt(" bmp280_init status", rslt);

    rslt = bmp280_get_config(conf, bmp);
    print_rslt(" bmp280_get_config status", rslt);

    rslt = bmp280_set_config(conf, bmp);
    print_rslt(" bmp280_set_config status", rslt);

    // Always set the power mode after setting the configuration
    rslt = bmp280_set_power_mode(BMP280_SLEEP_MODE, bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);

    // Throw away first measurement
    rslt |= bmp280_get_uncomp_data(ucomp_data, bmp);

    bmp->delay_ms(100);
}

int main(int argc, char *argv[])
{
    int8_t rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    double pres, temp;
    // Pick from either SPI or I2C below, no other code has to be changed:

    rslt = spi_init(&bmp);
    //rslt = i2c_init(&bmp);

    // IIR filter
    conf.filter = BMP280_FILTER_OFF;
    // Temperature over sampling
    conf.os_temp = BMP280_OS_2X;
    // Pressure over sampling
    conf.os_pres = BMP280_OS_16X;
    // Setting the output data period as 500us
    conf.odr = BMP280_ODR_0_5_MS;

    bmp_init(&bmp, &conf, &ucomp_data);

    while (1)
    {
        // Reading the raw data from sensor
        rslt |= bmp280_get_uncomp_data(&ucomp_data, &bmp);

        // Getting the compensated temperature as floating point value
        rslt |= bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
        rslt |= bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);

        printf("Temperature: %.2f C \r\nPressure: %.2f hPa \r\n", temp, pres / 100);

        // Sleep time between measurements mins out at BMP280_ODR
        bmp.delay_ms(10);
    }

    // Unreachable code, here in case I decide to change it in the future
    close_all(bmp);
    return rslt;
}
