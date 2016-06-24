/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Dronesmith Technologies. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file luci_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>
#include <nuttx/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32.h>
#include "board_config.h"
#include <systemlib/err.h>

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI4
	px4_arch_configgpio(GPIO_SPI_CS_LSM9DS0_GYRO);
	px4_arch_configgpio(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG);
	px4_arch_configgpio(GPIO_SPI_CS_MS5611);
	px4_arch_configgpio(GPIO_SPI_CS_MPU9250);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_GYRO, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);

	px4_arch_configgpio(GPIO_DRDY_LSM9DS0_GYRO);
	px4_arch_configgpio(GPIO_DRDY_LSM9DS0_XM2);
	px4_arch_configgpio(GPIO_DRDY_LSM9DS0_XM1);
	px4_arch_configgpio(GPIO_DRDY_MPU9250);
#endif

#ifdef CONFIG_STM32_SPI2
	px4_arch_configgpio(GPIO_SPI_CS_FRAM);
	px4_arch_gpiowrite(GPIO_SPI_CS_FRAM, 1);
#endif

#ifdef CONFIG_STM32_SPI1
	px4_arch_configgpio(GPIO_SPI_CS_EXT0);
	px4_arch_configgpio(GPIO_SPI_CS_EXT1);
	px4_arch_gpiowrite(GPIO_SPI_CS_EXT0, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_EXT1, 1);
#endif
}

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_GYRO:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_GYRO, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		break;

	case PX4_SPIDEV_ACCEL_MAG:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_GYRO, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_GYRO, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		break;

	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_GYRO, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, !selected);
		break;

	default:
		break;
	}
}

__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}


#ifdef CONFIG_STM32_SPI2
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* there can only be one device on this bus, so always select it */
	px4_arch_gpiowrite(GPIO_SPI_CS_FRAM, !selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}
#endif

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_EXT0:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_EXT0, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_EXT1, 1);
		break;

	case PX4_SPIDEV_EXT1:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_EXT0, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_EXT1, !selected);
		break;

	default:
		break;

	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	px4_arch_configgpio(GPIO_SPI_CS_OFF_LSM9DS0_GYRO);
	px4_arch_configgpio(GPIO_SPI_CS_OFF_LSM9DS0_ACCEL_MAG);
	px4_arch_configgpio(GPIO_SPI_CS_OFF_MS5611);
	px4_arch_configgpio(GPIO_SPI_CS_OFF_MPU9250);

	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_LSM9DS0_GYRO, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_LSM9DS0_ACCEL_MAG, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_MS5611, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_MPU9250, 0);

	px4_arch_configgpio(GPIO_SPI1_SCK_OFF);
	px4_arch_configgpio(GPIO_SPI1_MISO_OFF);
	px4_arch_configgpio(GPIO_SPI1_MOSI_OFF);

	px4_arch_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);

	px4_arch_configgpio(GPIO_DRDY_OFF_LSM9DS0_GYRO);
	px4_arch_configgpio(GPIO_DRDY_OFF_LSM9DS0_XM2);
	px4_arch_configgpio(GPIO_DRDY_OFF_LSM9DS0_XM1);
	px4_arch_configgpio(GPIO_DRDY_OFF_MPU9250);

	px4_arch_gpiowrite(GPIO_DRDY_OFF_LSM9DS0_GYRO, 0);
	px4_arch_gpiowrite(GPIO_DRDY_OFF_LSM9DS0_XM2, 0);
	px4_arch_gpiowrite(GPIO_DRDY_OFF_LSM9DS0_XM1, 0);
	px4_arch_gpiowrite(GPIO_DRDY_OFF_MPU9250, 0);

	/* set the sensor rail off */
	px4_arch_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
#ifdef CONFIG_STM32_SPI4
	px4_arch_configgpio(GPIO_SPI_CS_LSM9DS0_GYRO);
	px4_arch_configgpio(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG);
	px4_arch_configgpio(GPIO_SPI_CS_MS5611);
	px4_arch_configgpio(GPIO_SPI_CS_MPU9250);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_GYRO, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);

	px4_arch_configgpio(GPIO_SPI4_SCK);
	px4_arch_configgpio(GPIO_SPI4_MISO);
	px4_arch_configgpio(GPIO_SPI4_MOSI);


#endif
}
