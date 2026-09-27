/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ---------------------------------------------------------------------
 * Fixed version of the bcm2835-based MLX90641 I2C driver:
 *  - buf/cmd use unsigned char to avoid sign-extension corrupting the
 *    high byte of every 16-bit word whose low byte is >= 0x80
 *  - init (bcm2835_init/i2c_begin) is done once, up front, not lazily
 *    inside Read only
 *  - slave address is (re)set in both Read and Write
 *  - MLX90641_I2CGeneralReset added (check your .hpp: remove this if
 *    it isn't declared there)
 * ---------------------------------------------------------------------
 */

#include <iostream>
#include <MLX90641_I2C_Driver.hpp>
#include <unistd.h>
#include <stdint.h>
#include <bcm2835.h>

static uint8_t g_slaveAddr = 0x33; /* default MLX90641 address */
static int g_init = 0;

static void ensure_init(void)
{
    if (!g_init) {
        if (!bcm2835_init()) {
            std::cerr << "bcm2835_init failed - are you running as root (sudo)?" << std::endl;
        }
        bcm2835_i2c_begin();
        bcm2835_i2c_set_baudrate(400000); /* 400 kHz default */
        g_init = 1;
    }
}

void MLX90641_I2CInit()
{
    ensure_init();
}

void MLX90641_I2CFreqSet(int freq)
{
    ensure_init();
    bcm2835_i2c_set_baudrate((uint32_t)freq);
}

int MLX90641_I2CGeneralReset(void)
{
    ensure_init();
    unsigned char reset_cmd = 0x06;
    bcm2835_i2c_setSlaveAddress(0x00);
    uint8_t result = bcm2835_i2c_write((const char *)&reset_cmd, 1);
    return (result == BCM2835_I2C_REASON_OK) ? 0 : -1;
}

int MLX90641_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    ensure_init();

    unsigned char cmd[2] = {
        (unsigned char)(startAddress >> 8),
        (unsigned char)(startAddress & 0xFF)
    };

    bcm2835_i2c_setSlaveAddress(slaveAddr);

    uint16_t bytesToRead = nMemAddressRead * 2;
    if (bytesToRead > 1664) {
        std::cerr << "MLX90641_I2CRead: requested read (" << bytesToRead
                  << " bytes) exceeds buffer size" << std::endl;
        return -1;
    }

    unsigned char buf[1664];
    uint8_t result = bcm2835_i2c_write_read_rs((char *)cmd, 2, (char *)buf, bytesToRead);
    if (result != BCM2835_I2C_REASON_OK) {
        std::cerr << "MLX90641_I2CRead: bcm2835 i2c error " << (int)result << std::endl;
        return -1;
    }

    uint16_t *p = data;
    for (int count = 0; count < nMemAddressRead; count++) {
        int i = count << 1;
        /* cast through uint8_t so no sign-extension corrupts the high byte */
        *p++ = ((uint16_t)buf[i] << 8) | (uint16_t)buf[i + 1];
    }
    return 0;
}

int MLX90641_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    ensure_init();

    unsigned char cmd[4] = {
        (unsigned char)(writeAddress >> 8),
        (unsigned char)(writeAddress & 0x00FF),
        (unsigned char)(data >> 8),
        (unsigned char)(data & 0x00FF)
    };

    bcm2835_i2c_setSlaveAddress(slaveAddr);

    uint8_t result = bcm2835_i2c_write((const char *)cmd, 4);
    if (result != BCM2835_I2C_REASON_OK) {
        std::cerr << "MLX90641_I2CWrite: bcm2835 i2c error " << (int)result << std::endl;
        return -1;
    }
    return 0;
}