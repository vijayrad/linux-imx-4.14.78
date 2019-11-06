/*
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

#define MIPI_DSI_MAX_RET_PACK_SIZE		(0x4)

#define MIPI_DSI_CMD_WRT_DISP_BRIGHT		(0x51)
#define MIPI_DSI_MAX_BRIGHT		(255)
#define MIPI_DSI_DEF_BRIGHT		(255)

#define MIPI_DSI_MAX_DPHY_CLK		(500)
#define MIPI_DSI_ONE_DATA_LANE		(0x1)
#define MIPI_DSI_TWO_DATA_LANE		(0x2)

#ifdef RECT // Rectangular Display
#define MIPI_DSI_REFRESH	(64)
#define MIPI_DSI_XRES		(320)
#define MIPI_DSI_YRES		(432)
#define MIPI_DSI_PIXCLOCK		(91895)
#define MIPI_DSI_LEFT_MARGIN		(20)
#define MIPI_DSI_RIGHT_MARGIN		(5)
#define MIPI_DSI_UPPER_MARGIN		(30)
#define MIPI_DSI_LOWER_MARGIN		(26)
#define MIPI_DSI_HSYNC_LEN		(2)
#define MIPI_DSI_VSYNC_LEN		(2)
#else // Round Display
#define MIPI_DSI_REFRESH	(60)
#define MIPI_DSI_XRES		(400)
#define MIPI_DSI_YRES		(400)
#define MIPI_DSI_PIXCLOCK		(79290)
#define MIPI_DSI_LEFT_MARGIN		(40)
#define MIPI_DSI_RIGHT_MARGIN		(40)
#define MIPI_DSI_UPPER_MARGIN		(18)
#define MIPI_DSI_LOWER_MARGIN		(18)
#define MIPI_DSI_HSYNC_LEN		(1)
#define MIPI_DSI_VSYNC_LEN		(1)
#endif

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static void parse_variadic(int n, u8 *buf, ...)
{
	int i = 0;
	va_list args;

	if (unlikely(!n)) return;

	va_start(args, buf);

	for (i = 0; i < n; i++)
		buf[i + 1] = (u8)va_arg(args, int);

	va_end(args);
}

#define MIPI_DSI_DCS_write_1A_nP(n, addr, ...) {		\
	int err;						\
								\
	buf[0] = addr;						\
	parse_variadic(n, buf, ##__VA_ARGS__);			\
								\
	if (n >= 2)						\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,		\
			MIPI_DSI_DCS_LONG_WRITE, (u32*)buf, n + 1);	\
	else if (n == 1)					\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,	\
			MIPI_DSI_DCS_SHORT_WRITE_PARAM, (u32*)buf, 0);	\
	else if (n == 0)					\
	{							\
		buf[1] = 0;					\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,	\
			MIPI_DSI_DCS_SHORT_WRITE, (u32*)buf, 0);	\
	}							\
	CHECK_RETCODE(err);					\
}

#define MIPI_DSI_DCS_write_1A_0P(addr)		\
	MIPI_DSI_DCS_write_1A_nP(0, addr)

#define MIPI_DSI_DCS_write_1A_1P(addr, ...)	\
	MIPI_DSI_DCS_write_1A_nP(1, addr, __VA_ARGS__)

#define MIPI_DSI_DCS_write_1A_2P(addr, ...)	\
	MIPI_DSI_DCS_write_1A_nP(2, addr, __VA_ARGS__)

#define MIPI_DSI_DCS_write_1A_3P(addr, ...)	\
	MIPI_DSI_DCS_write_1A_nP(3, addr, __VA_ARGS__)

#define MIPI_DSI_DCS_write_1A_4P(addr, ...)	\
	MIPI_DSI_DCS_write_1A_nP(4, addr, __VA_ARGS__)

#define MIPI_DSI_DCS_write_1A_15P(addr, ...)	\
	MIPI_DSI_DCS_write_1A_nP(15, addr, __VA_ARGS__)

static int hx8369bl_brightness;
static int mipid_init_backlight(struct mipi_dsi_info *mipi_dsi);

static struct fb_videomode truly_lcd_modedb[] = {
	{
		"TRULY-WVGA",								/* name										 */
		MIPI_DSI_REFRESH,						/* refresh / frame rate		 */
		MIPI_DSI_XRES,							/* resolution							 */
		MIPI_DSI_YRES,							/* resolution							 */
		MIPI_DSI_PIXCLOCK,					/* pixel clock							*/
		MIPI_DSI_LEFT_MARGIN,				/* l/r margin							 */
		MIPI_DSI_RIGHT_MARGIN,			/* l/r margin							 */
		MIPI_DSI_UPPER_MARGIN,			/* u/l margin							 */
		MIPI_DSI_LOWER_MARGIN,			/* u/l margin							 */
		MIPI_DSI_HSYNC_LEN,					/* hsync/vsync length			 */
		MIPI_DSI_VSYNC_LEN,					/* hsync/vsync length			 */
		0,													/* sync										 */
		FB_VMODE_NONINTERLACED,			/* vmode										*/
		0,													/* flag FB_MODE_IS_DETAILED */
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num	= MIPI_DSI_ONE_DATA_LANE,
	.max_phy_clk		= MIPI_DSI_MAX_DPHY_CLK,
	.dpi_fmt	= MIPI_RGB888,
};

void mipid_hx8369_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}

int mipid_hx8369_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u8 buf[DSI_CMD_BUF_MAXSIZE];
	int err;

	/* preapre mipi dsi display setup */
#ifdef RECT
	dev_info(&mipi_dsi->pdev->dev, "C0 MIPI DSI LCD setup\n");
	MIPI_DSI_DCS_write_1A_2P(0xC0,0xB0,0xB0);
	MIPI_DSI_DCS_write_1A_1P(0xC1,0x45);
	MIPI_DSI_DCS_write_1A_3P(0xC5,0x00,0x28,0x80);
	MIPI_DSI_DCS_write_1A_1P(0x36,0x48);
	MIPI_DSI_DCS_write_1A_1P(0x3A,0x66);
	MIPI_DSI_DCS_write_1A_0P(0x21);
	MIPI_DSI_DCS_write_1A_1P(0xB1,0xA0);
	MIPI_DSI_DCS_write_1A_1P(0xB4,0x02);
	MIPI_DSI_DCS_write_1A_4P(0xB5,0x02,0x02,0x03,0x03);
	MIPI_DSI_DCS_write_1A_3P(0xB6,0x00,0x02,0x3B);
	MIPI_DSI_DCS_write_1A_1P(0xE9,0x00);
	MIPI_DSI_DCS_write_1A_4P(0xF7,0xA9,0x51,0x2C,0x02);
	MIPI_DSI_DCS_write_1A_15P(0xE0,0x00,0x07,0x0A,0x06,0x11,0x07,0x26,0xBD,0x3C,0x09,0x17,0x0A,0x26,0x22,0x0F);
	MIPI_DSI_DCS_write_1A_15P(0xE1,0x00,0x22,0x31,0x0C,0x1C,0x0C,0x4C,0x76,0x62,0x0B,0x16,0x0F,0x39,0x39,0x0F);
	MIPI_DSI_DCS_write_1A_0P(0x11);
	msleep(120);
	MIPI_DSI_DCS_write_1A_0P(0x29);
	msleep(10);

#else
	// Round MIPI DSI
	msleep(10);
	MIPI_DSI_DCS_write_1A_1P(0xFE,0x00);
	MIPI_DSI_DCS_write_1A_1P(0x11,0x00);
	MIPI_DSI_DCS_write_1A_1P(0x35,0x01);
	MIPI_DSI_DCS_write_1A_1P(0x51,0xFF);
	msleep(300);
	MIPI_DSI_DCS_write_1A_1P(0x29,0x00);
	dev_info(&mipi_dsi->pdev->dev, "Microtip Round Display Setup\n");
#endif

	return err;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	int err;
	u32 buf;
	int brightness = bl->props.brightness;
	struct mipi_dsi_info *mipi_dsi = bl_get_data(bl);

	if (bl->props.power != FB_BLANK_UNBLANK ||
			bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	buf = MIPI_DSI_CMD_WRT_DISP_BRIGHT |
			((brightness & MIPI_DSI_MAX_BRIGHT) << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
		&buf, 0);
	CHECK_RETCODE(err);

	hx8369bl_brightness = brightness & MIPI_DSI_MAX_BRIGHT;

	dev_dbg(&bl->dev, "mipid backlight bringtness:%d.\n", brightness);
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return hx8369bl_brightness;
}

static int mipi_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	return 0;
}

static const struct backlight_ops mipid_lcd_bl_ops = {
	.update_status = mipid_bl_update_status,
	.get_brightness = mipid_bl_get_brightness,
	.check_fb = mipi_bl_check_fb,
};

static int mipid_init_backlight(struct mipi_dsi_info *mipi_dsi)
{
	struct backlight_properties props;
	struct backlight_device	*bl;

	if (mipi_dsi->bl) {
		pr_debug("mipid backlight already init!\n");
		return 0;
	}
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = MIPI_DSI_MAX_BRIGHT;
	props.type = BACKLIGHT_RAW;
	bl = backlight_device_register("mipid-bl", &mipi_dsi->pdev->dev,
		mipi_dsi, &mipid_lcd_bl_ops, &props);
	if (IS_ERR(bl)) {
		pr_err("error %ld on backlight register\n", PTR_ERR(bl));
		return PTR_ERR(bl);
	}
	mipi_dsi->bl = bl;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.brightness = MIPI_DSI_DEF_BRIGHT;

	mipid_bl_update_status(bl);
	return 0;
}
