/*
 * FB driver for the S1D15G00 LCD display controller
 *
 * Copyright (C) 2015 Stefan Wahren
 * Based on n6100fb.c by Zsolt Hajdu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "fbtft.h"

#define DRVNAME		"fb_s1d15g00"
#define WIDTH		132
#define HEIGHT		132

/* supported commands */
#define EPSON_DISON       0xaf
#define EPSON_DISOFF      0xae
#define EPSON_DISNOR      0xa6
#define EPSON_DISINV      0xa7
#define EPSON_COMSCN      0xbb
#define EPSON_DISCTL      0xca
#define EPSON_SLPIN       0x95
#define EPSON_SLPOUT      0x94
#define EPSON_PASET       0x75
#define EPSON_CASET       0x15
#define EPSON_DATCTL      0xbc
#define EPSON_RGBSET8     0xce
#define EPSON_RAMWR       0x5c
#define EPSON_RAMRD       0x5d
#define EPSON_PTLIN       0xa8
#define EPSON_PTLOUT      0xa9
#define EPSON_RMWIN       0xe0
#define EPSON_RMWOUT      0xee
#define EPSON_ASCSET      0xaa
#define EPSON_SCSTART     0xab
#define EPSON_OSCON       0xd1
#define EPSON_OSCOFF      0xd2
#define EPSON_PWRCTR      0x20
#define EPSON_VOLCTR      0x81
#define EPSON_VOLUP       0xd6
#define EPSON_VOLDOWN     0xd7
#define EPSON_TMPGRD      0x82
#define EPSON_EPCTIN      0xcd
#define EPSON_EPCOUT      0xcc
#define EPSON_EPMWR       0xfc
#define EPSON_EPMRD       0xfd
#define EPSON_EPSRRD1     0x7c
#define EPSON_EPSRRD2     0x7d
#define EPSON_NOP         0x25

/* 16 bit to 12 bit conversion helper */
#define RGB565_R4(c) (((c & 0xF800) >> 11) * 15 / 31)
#define RGB565_G4(c) (((c & 0x07E0) >> 5) * 15 / 63)
#define RGB565_B4(c) ((c & 0x001F) * 15 / 31)

static short alpha = 32;
module_param(alpha, short, 0);
MODULE_PARM_DESC(alpha, "Voltage regulator volume value alpha: 0-63 (default: 32)");

/* 12 bit pixel over 9-bit SPI bus: dc + high byte, dc + low byte */
int write_vmem16_bus9(struct fbtft_par *par, size_t offset, size_t len)
{
	u16 *vmem16 = (u16 *)(par->info->screen_base + offset);
	u16 *txbuf16 = par->txbuf.buf;
	size_t buf_len;
	size_t tx_array_size;
	int i, j;
	int ret = 0;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s(offset=%zu, len=%zu)\n",
		      __func__, offset, len);

	if (!par->txbuf.buf) {
		dev_err(par->info->device, "%s: txbuf.buf is NULL\n", __func__);
		return -1;
	}

	j = 0;

	/* make sure buffer is multiple of 3 and 8 */
	buf_len = par->txbuf.len - (par->txbuf.len % 24);
	tx_array_size = buf_len / 2;

	while (j < len / 2) {
		i = 0;

		while (i < buf_len / 2) {
			/* convert odd pixel to 12 bit */
			txbuf16[i]  = 0x0100;
			txbuf16[i] |= RGB565_R4(vmem16[j]) << 4;
			txbuf16[i] |= RGB565_G4(vmem16[j]);
			i++;

			txbuf16[i]  = 0x0100;
			txbuf16[i] |= RGB565_B4(vmem16[j]) << 4;
			j++;

			if (j >= len / 2) {
				i++;
				break;
			}

			/* convert even pixel to 12 bit */
			txbuf16[i] |= RGB565_R4(vmem16[j]);
			i++;

			txbuf16[i]  = 0x0100;
			txbuf16[i] |= RGB565_G4(vmem16[j]) << 4;
			txbuf16[i] |= RGB565_B4(vmem16[j]);
			i++;
			j++;

			if (j >= len / 2)
				break;
		}

		/* fill up with NOP */
		if (j >= len / 2) {
			while (i % 4) {
				txbuf16[i] = 0x0000;
				i++;
			}
		}

		dev_dbg(par->info->device, "%s: i=%d, j=%d\n", __func__, i, j);

		ret = par->fbtftops.write(par, par->txbuf.buf, i * 2);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int init_display(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	par->fbtftops.reset(par);

	write_reg(par, EPSON_DISCTL, 0x00, 0x20, 0x0a); /* display reset */
	write_reg(par, EPSON_COMSCN, 0x01); /* COM scan */
	write_reg(par, EPSON_OSCON); /* internal oscillators on */

	write_reg(par, EPSON_SLPOUT); /* sleep out */

	write_reg(par, EPSON_VOLCTR, alpha & 0x3F, 0x03); /* voltage control */

	write_reg(par, EPSON_PWRCTR, 0x0f); /* voltage regulators on */

	write_reg(par, EPSON_DISINV); /* inverse display */

	/* no flip, RGB, 12 bit */
	write_reg(par, EPSON_DATCTL, 0x00, 0x00, 0x02);

	mdelay(100); /* allow power supply to stabilize */
	write_reg(par, EPSON_DISON); /* display on */
	mdelay(10);

	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par, "%s(xs=%d, ys=%d, xe=%d, ye=%d)\n",
		      __func__, xs, ys, xe, ye);

	/* Column address set */
	write_reg(par, EPSON_CASET, xs & 0xff, xe & 0xff);

	/* Row address set */
	write_reg(par, EPSON_PASET, ys & 0xff, ye & 0xff);

	/* Write memory */
	write_reg(par, EPSON_RAMWR);
}

static int set_var(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	write_reg(par, EPSON_DATCTL, 0x00, par->bgr, 0x02);

	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.fbtftops = {
		.write_vmem = write_vmem16_bus9,
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.set_var = set_var,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "epson,s1d15g00", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:s1d15g00");
MODULE_ALIAS("platform:s1d15g00");

MODULE_DESCRIPTION("FB driver for the S1D15G00 LCD display controller");
MODULE_AUTHOR("Stefan Wahren");
MODULE_LICENSE("GPL v2");
