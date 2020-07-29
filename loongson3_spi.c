/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2020 Jiaxun Yang <jiaxun.yang@flygoat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#if CONFIG_LOONGSON3_SPI == 1

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include "flash.h"
#include "chipdrivers.h"
#include "programmer.h"
#include "spi.h"

#define LOONGSON64C_SPI_BASE	0x1fe00220
#define LOONGSON64G_SPI_BASE	0x1fe001f0
#define LOONGSON3_SPI_REG_SIZE	0x10

#define SPICTRL_SPCR	0x0
#define SPCR_MSTR	(1 << 4)
#define SPCR_SPE	(1 << 6)

#define SPICTRL_SPSR	0x1
#define SPSR_RFEMPTY	(1 << 0)
#define SPSR_RFFULL		(1 << 1)
#define SPSR_WFEMPTY	(1 << 2)
#define SPSR_WFFULL		(1 << 3)
#define SPSR_WCOL		(1 << 6)

#define SPICTRL_FIFO	0x2

#define SPICTRL_SFCP	0x4
#define SFCP_MEMEN		(1 << 0)

#define SPICTRL_SOFTCS	0x5
/* Firmware flash is always connected to CS0 */
#define SOFTCS_ASSERT	((0 << 4) | (1 << 0))
#define SOFTCS_DESSERT	((1 << 4) | (1 << 0))

static uint8_t *spictrl_base;

static int loongson3_spi_shutdown(void *data);
static int loongson3_spi_send_command(const struct flashctx *flash, unsigned int writecnt,
				  unsigned int readcnt,
				   const uint8_t *writearr,
				   uint8_t *readarr);

static const struct spi_master spi_master_loongson3 = {
	.max_data_read	= MAX_DATA_READ_UNLIMITED,
	.max_data_write	= MAX_DATA_WRITE_UNLIMITED,
	.command	= loongson3_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= default_spi_read,
	.write_256	= default_spi_write_256,
	.write_aai	= default_spi_write_aai,
};

static int cpu_is_loongson64c(char *cpu)
{
	if (strcmp(cpu, "3b1500") == 0)
		return 1;

	if (strcmp(cpu, "3a2000") == 0)
		return 1;

	if (strcmp(cpu, "3b2000") == 0)
		return 1;

	if (strcmp(cpu, "3a3000") == 0)
		return 1;

	if (strcmp(cpu, "3b3000") == 0)
		return 1;

	return 0;
}

static int cpu_is_loongson64g(char *cpu)
{
	if (strcmp(cpu, "3a4000") == 0)
		return 1;

	if (strcmp(cpu, "3b4000") == 0)
		return 1;

	return 0;
}

int loongson3_spi_init(void)
{
	uint8_t reg;
	char *cpu;

	/* Use -cpu parameter as different kernels have different cpuinfo
	 * and it is almost impossible for us to determine all of them.
	 */
	cpu = extract_programmer_param("cpu");

	if (!cpu) {
		free(cpu);
		msg_perr("No -cpu specified\n");
		return 1;
	}

	if (cpu_is_loongson64c(cpu)) {
		spictrl_base = rphysmap("Loongson64C SPICTRL", LOONGSON64C_SPI_BASE,
			 LOONGSON3_SPI_REG_SIZE);
			msg_pwarn("64c\n");
	} else if (cpu_is_loongson64g(cpu)) {
		spictrl_base = rphysmap("Loongson64G SPICTRL", LOONGSON64G_SPI_BASE,
			 LOONGSON3_SPI_REG_SIZE);
			msg_pwarn("64g\n");
	} else {
		free(cpu);
		msg_perr("Invalid -cpu specified\n");
		return 1;
	}

	free(cpu);

	if (!spictrl_base) {
		msg_perr("Failed to map base\n");
		return 1;
	}

	reg = mmio_readb(spictrl_base + SPICTRL_SFCP);
	if (!(reg & SFCP_MEMEN))
		msg_pwarn("Read engine is not enabled, SPI is not system firmware?\n");

	if (register_shutdown(loongson3_spi_shutdown, NULL))
		return 1;

	/* Dessert CS */
	mmio_writeb(SOFTCS_DESSERT, spictrl_base + SPICTRL_SOFTCS);

	/* Enable SPI Controller */
	reg = mmio_readb(spictrl_base + SPICTRL_SPCR);
	reg |= SPCR_MSTR | SPCR_SPE;
	mmio_writeb(reg, spictrl_base + SPICTRL_SPCR);

	/* Disable read engine for software control */
	reg = mmio_readb(spictrl_base + SPICTRL_SFCP);
	reg &= ~SFCP_MEMEN;
	mmio_writeb(reg, spictrl_base + SPICTRL_SFCP);


	/* Sometimes Read FIFO is not empty at boot time */
	while (!(mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_RFEMPTY))
		mmio_readb(spictrl_base + SPICTRL_FIFO);

	register_spi_master(&spi_master_loongson3);
	return 0;
}

static int loongson3_spi_shutdown(void *data)
{
	if (!spictrl_base) {
		uint8_t reg;

		/* Disable soft CS */
		mmio_writeb(0x0, spictrl_base + SPICTRL_SOFTCS);

		/* Enable read engine again */
		reg = mmio_readb(spictrl_base + SPICTRL_SFCP);
		reg |= SFCP_MEMEN;
		mmio_writeb(reg, spictrl_base + SPICTRL_SFCP);
	}

	return 0;
}

static int loongson3_spi_send_command(const struct flashctx *flash, unsigned int writecnt,
				  unsigned int readcnt,
				   const uint8_t *writearr,
				   uint8_t *readarr)
{
	unsigned int i;

	msg_pwarn("writecnt: %d, readcnt: %d\n", writecnt, readcnt);

	mmio_writeb(SOFTCS_ASSERT, spictrl_base + SPICTRL_SOFTCS);

	for (i = 0; i < writecnt; i++) {
		mmio_writeb(writearr[i], spictrl_base + SPICTRL_FIFO);

		/* Wait until Read FIFO not empty */
		while (mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_RFEMPTY);

		mmio_readb(spictrl_base + SPICTRL_FIFO);
	}

	for (i = 0; i < readcnt; i++) {
		mmio_writeb(writearr[i], spictrl_base + SPICTRL_FIFO);

		/* Wait until Read FIFO not empty */
		while (mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_RFEMPTY);

		readarr[i] = mmio_readb(spictrl_base + SPICTRL_FIFO);
	}

	mmio_writeb(SOFTCS_DESSERT, spictrl_base + SPICTRL_SOFTCS);

	return 0;
}

#if 0
#define FIFO_DETPTH	4

static int loongson3_spi_send_command(const struct flashctx *flash, unsigned int writecnt,
				  unsigned int readcnt,
				   const uint8_t *writearr,
				   uint8_t *readarr)
{
	unsigned int i, j, cur_depth;

	msg_pwarn("writecnt: %d, readcnt: %d\n", writecnt, readcnt);

	mmio_writeb(SOFTCS_ASSERT, spictrl_base + SPICTRL_SOFTCS);

	cur_depth = 0;
	for (i = 0; i < writecnt; i++) {
		mmio_writeb(writearr[i], spictrl_base + SPICTRL_FIFO);

		if ((mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_WFFULL) ||
			(writecnt - i == 1)) {

			msg_pwarn("CMD W FULL %d\n", cur_depth);
			/* Wait until WF empty */
//			while (!(mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_WFEMPTY));
			/* Wait until RF is not empty */

			for (j = 0; j < cur_depth + 1; j++) {
				while (mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_RFEMPTY);

				mmio_readb(spictrl_base + SPICTRL_FIFO);
			}
			cur_depth = 0;
		}
		cur_depth++;
	}

	cur_depth = 0;
	for (i = 0; i < readcnt; i++) {
		mmio_writeb(0x0, spictrl_base + SPICTRL_FIFO);

		if ((mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_WFFULL) ||
			(readcnt - i == 1)) {
			msg_pwarn("CMD R FULL %d\n", cur_depth);
			/* Wait until WF empty */
//			while (!(mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_WFEMPTY));
			/* Wait until RF is not empty */

			for (j = 0; j < cur_depth + 1; j++) {
				while (mmio_readb(spictrl_base + SPICTRL_SPSR) & SPSR_RFEMPTY);

				readarr[i - cur_depth + j] = mmio_readb(spictrl_base + SPICTRL_FIFO);
			}
			cur_depth = 0;
		}
		cur_depth++;
	}

	mmio_writeb(SOFTCS_DESSERT, spictrl_base + SPICTRL_SOFTCS);

	return 0;
}
#endif

#endif // CONFIG_LOONGSON3_SPI == 1
