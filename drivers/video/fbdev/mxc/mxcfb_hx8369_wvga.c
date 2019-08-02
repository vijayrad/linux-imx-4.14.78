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

#define HX8369BL_MAX_BRIGHT		(255)
#define HX8369BL_DEF_BRIGHT		(255)

#define HX8369_MAX_DPHY_CLK		(800)
#define HX8369_ONE_DATA_LANE		(0x1)
#define HX8369_TWO_DATA_LANE		(0x2)

#define HX8369_CMD_SETEXTC		(0xB9)
#define HX8369_CMD_SETEXTC_LEN		(0x4)
#define HX8369_CMD_SETEXTC_PARAM_1		(0x6983ff)

#define HX8369_CMD_GETHXID		(0xF4)
#define HX8369_CMD_GETHXID_LEN		(0x4)
#define HX8369_ID		(0x69)
#define HX8369_ID_MASK		(0xFF)

/* C0 Display register sequence */
#define HX8369_CMD_PWR_CTRL1		(0xC0)
#define HX8369_CMD_PWR_CTRL1_SIZE		(3)
#define HX8369_CMD_PWR_CTRL1_DATA1		(0xB0B0)

#define HX8369_CMD_PWR_CTRL2		(0xC1)
#define HX8369_CMD_PWR_CTRL2_SIZE		(2)
#define HX8369_CMD_PWR_CTRL2_DATA1		(0x45)

#define HX8369_CMD_VCOM_CTRL		(0xC5)
#define HX8369_CMD_VCOM_CTRL_SIZE		(4)
#define HX8369_CMD_VCOM_CTRL_DATA1		(0x802800)

#define HX8369_CMD_MEM_CTRL		(0x36)
#define HX8369_CMD_MEM_CTRL_SIZE		(2)
#define HX8369_CMD_MEM_CTRL_DATA1		(0x48)

#define HX8369_CMD_SETPIXEL_FMT		(0x3A)
#define HX8369_CMD_SETPIXEL_FMT_SIZE		(2)
#define HX8369_CMD_SETPIXEL_FMT_24BPP		(0x77)
#define HX8369_CMD_SETPIXEL_FMT_18BPP		(0x66)
#define HX8369_CMD_SETPIXEL_FMT_16BPP		(0x55)

#define HX8369_CMD_DISP_INV_ON_CTRL		(0x21)

#define HX8369_CMD_FRAME_RATE_CTRL		(0xB1)
#define HX8369_CMD_FRAME_RATE_CTRL_SIZE		(2)
#define HX8369_CMD_FRAME_RATE_CTRL_DATA1		(0xA0)

#define HX8369_CMD_DISP_INV_CTRL		(0xB4)
#define HX8369_CMD_DISP_INV_CTRL_SIZE		(2)
#define HX8369_CMD_DISP_INV_CTRL_DATA1		(0x02)

#define HX8369_CMD_PORCH_CONTROL		(0xB5)
#define HX8369_CMD_PORCH_CONTROL_LEN		(5)
#define HX8369_CMD_PORCH_CONTROL_PARAM_1		(0x030202)
#define HX8369_CMD_PORCH_CONTROL_PARAM_2		(0x03)

#define HX8369_CMD_DISP_FUNCT_CTRL		(0xB6)
#define HX8369_CMD_DISP_FUNCT_CTRL_SIZE		(4)
#define HX8369_CMD_DISP_FUNCT_CTRL_DATA1		(0x3B0200)

#define HX8369_CMD_SET_IMG_CTRL		(0xE9)
#define HX8369_CMD_SET_IMG_CTRL_SIZE		(2)
#define HX8369_CMD_SET_IMG_CTRL_DATA1		(0x00)

#define HX8369_CMD_ADJUST_CTRL		(0xF7)
#define HX8369_CMD_ADJUST_CTRL_SIZE		(5)
#define HX8369_CMD_ADJUST_CTRL_DATA1		(0x2C51A9)
#define HX8369_CMD_ADJUST_CTRL_DATA2		(0x02)

#define HX8369_CMD_SET_PASSIVE_GAMMA		(0xE0)
#define HX8369_CMD_SET_PASSIVE_GAMMA_LEN		(16)
#define HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_1		(0x0A0700)
#define HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_2		(0x26071106)
#define HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_3		(0x17093CBD)
#define HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_4		(0x0F22260A)

#define HX8369_CMD_SET_NEGATIVE_GAMMA		(0xE1)
#define HX8369_CMD_SET_NEGATIVE_GAMMA_LEN		(16)
#define HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_1		(0x312200)
#define HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_2		(0x4C0C1C0C)
#define HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_3		(0x160B6276)
#define HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_4		(0x0F39390F)

#define HX8369_CMD_SLEEP_OUT_CTRL		(0x11)

#define HX8369_CMD_DISP_ON_CTRL		(0x29)

#define HX8369_CMD_MEM_WRITE_CTRL		(0x2C)

#define HX8369_CMD_WRT_DISP_BRIGHT				(0x51)
#define HX8369_CMD_WRT_DISP_BRIGHT_PARAM_1			(0xFF)

#define HX8359_REFRESH		(64)
#define HX8359_XRES		(320)
#define HX8359_YRES		(432)
#define HX8359_PIXCLOCK		(91895)
#define HX8359_LEFT_MARGIN		(20)
#define HX8359_RIGHT_MARGIN		(5)
#define HX8359_UPPER_MARGIN		(30)
#define HX8359_LOWER_MARGIN		(26)
#define HX8359_HSYNC_LEN		(2)
#define HX8359_VSYNC_LEN		(2)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static int hx8369bl_brightness;
static int mipid_init_backlight(struct mipi_dsi_info *mipi_dsi);

static struct fb_videomode truly_lcd_modedb[] = {
	{
		"TRULY-WVGA",							 /* name										 */
		HX8359_REFRESH,						 /* refresh / frame rate		 */
		HX8359_XRES,								/* resolution							 */
		HX8359_YRES,								/* resolution							 */
		HX8359_PIXCLOCK,						/* pixel clock							*/
		HX8359_LEFT_MARGIN,				 /* l/r margin							 */
		HX8359_RIGHT_MARGIN,				/* l/r margin							 */
		HX8359_UPPER_MARGIN,				/* u/l margin							 */
		HX8359_LOWER_MARGIN,				/* u/l margin							 */
		HX8359_HSYNC_LEN,					 /* hsync/vsync length			 */
		HX8359_VSYNC_LEN,					 /* hsync/vsync length			 */
		FB_SYNC_OE_LOW_ACT,				 /* sync										 */
		FB_VMODE_NONINTERLACED,		 /* vmode										*/
		0,													/* flag FB_MODE_IS_DETAILED */
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num	= HX8369_ONE_DATA_LANE,
	.max_phy_clk		= HX8369_MAX_DPHY_CLK,
	.dpi_fmt	= MIPI_RGB666_PACKED,
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
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	int err;

	/* preapre mipi dsi display setup */
	dev_info(&mipi_dsi->pdev->dev, "C0 MIPI DSI LCD setup\n");

	/* Update the C0 display update seqence from here */
	/* Power control - 0xC0 Power control 1 */
	buf[0] = HX8369_CMD_PWR_CTRL1 | (HX8369_CMD_PWR_CTRL1_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_PWR_CTRL1_SIZE);
	CHECK_RETCODE(err);

	/* Power control - 0xC1 Power control 2 */
	buf[0] = HX8369_CMD_PWR_CTRL2 | (HX8369_CMD_PWR_CTRL2_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_PWR_CTRL2_SIZE);
	CHECK_RETCODE(err);

	/*	Set VCOM control - 0xC5 VCOM control */
	buf[0] = HX8369_CMD_VCOM_CTRL | (HX8369_CMD_VCOM_CTRL_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_VCOM_CTRL_SIZE);
	CHECK_RETCODE(err);

	/* 0x36 Memory access control */
	buf[0] = HX8369_CMD_MEM_CTRL | (HX8369_CMD_MEM_CTRL_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_MEM_CTRL_SIZE);
	CHECK_RETCODE(err);

	/* Set pixel format:24bpp - 0x3A Pixel format */
	buf[0] = HX8369_CMD_SETPIXEL_FMT;
	switch (lcd_config.dpi_fmt) {
	case MIPI_RGB565_PACKED:
	case MIPI_RGB565_LOOSELY:
	case MIPI_RGB565_CONFIG3:
		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_16BPP << 8);
		break;

	case MIPI_RGB666_LOOSELY:
	case MIPI_RGB666_PACKED:
		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_18BPP << 8);
		break;

	case MIPI_RGB888:
		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_24BPP << 8);
		break;

	default:
		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_24BPP << 8);
		break;
	}
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM, buf, 0);
	CHECK_RETCODE(err);

	/* Enter the invert mode - 0x21 Display inversion ON */
	buf[0] = MIPI_DCS_ENTER_INVERT_MODE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM, buf, 0);
	CHECK_RETCODE(err);

	/* 0xB1 Frame Rate Control */
	buf[0] = HX8369_CMD_FRAME_RATE_CTRL | (HX8369_CMD_FRAME_RATE_CTRL_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_FRAME_RATE_CTRL_SIZE);
	CHECK_RETCODE(err);

	/* 0xB4 Display Inversion Control */
	buf[0] = HX8369_CMD_DISP_INV_CTRL | (HX8369_CMD_DISP_INV_CTRL_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_DISP_INV_CTRL_SIZE);
	CHECK_RETCODE(err);

	/* 0xB5 Display porch Control */
	buf[0] = HX8369_CMD_PORCH_CONTROL | (HX8369_CMD_PORCH_CONTROL_PARAM_1 << 8);
	buf[1] = HX8369_CMD_PORCH_CONTROL_PARAM_2;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_PORCH_CONTROL_LEN);

	/* 0xB6 Display Function Control */
	buf[0] = HX8369_CMD_DISP_FUNCT_CTRL | (HX8369_CMD_DISP_FUNCT_CTRL_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_DISP_FUNCT_CTRL_SIZE);
	CHECK_RETCODE(err);

	/* 0xE9 Set Image */
	buf[0] = HX8369_CMD_SET_IMG_CTRL | (HX8369_CMD_SET_IMG_CTRL_DATA1 << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_SET_IMG_CTRL_SIZE);
	CHECK_RETCODE(err);

	/* 0xF7 Adjust Control */
	buf[0] = HX8369_CMD_ADJUST_CTRL | (HX8369_CMD_ADJUST_CTRL_DATA1 << 8);
	buf[1] = HX8369_CMD_ADJUST_CTRL_DATA2;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_ADJUST_CTRL_SIZE);
	CHECK_RETCODE(err);

	/* Set gamma curve related setting - 0xE0 Passive Gamma Control */
	buf[0] = HX8369_CMD_SET_PASSIVE_GAMMA |
			(HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_1 << 8);
	buf[1] = HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_2;
	buf[2] = HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_3;
	buf[3] = HX8369_CMD_SET_PASSIVE_GAMMA_PARAM_4;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_SET_PASSIVE_GAMMA_LEN);
	CHECK_RETCODE(err);

	/* Set gamma curve related setting - 0xE1 Negative Gamma Control */
	buf[0] = HX8369_CMD_SET_NEGATIVE_GAMMA |
			(HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_1 << 8);
	buf[1] = HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_2;
	buf[2] = HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_3;
	buf[3] = HX8369_CMD_SET_NEGATIVE_GAMMA_PARAM_4;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
		HX8369_CMD_SET_NEGATIVE_GAMMA_LEN);
	CHECK_RETCODE(err);

	/* Exit the sleep mode - 0x11 Sleep Out control */
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM, buf, 0);
	CHECK_RETCODE(err);

	// /* To allow 120 mSec time for the supply voltages
	//	* and clock circuits to stabilize.
	//	*/
	msleep(120);

	/* turn on the disply - 0x29 Display ON */
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM, buf, 0);
	CHECK_RETCODE(err);

	// /* To allow 10 mSec time after display on
	//	*/
	msleep(10);

	/* configure the memory - 0x2C Memory Write */
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
		MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM, buf, 0);
	CHECK_RETCODE(err);

	/* power on the backlight */
	// err = mipid_init_backlight(mipi_dsi);
	dev_info(&mipi_dsi->pdev->dev, "C0 MIPI DSI LCD setup Completed\n");
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

	buf = HX8369_CMD_WRT_DISP_BRIGHT |
			((brightness & HX8369BL_MAX_BRIGHT) << 8);
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
		&buf, 0);
	CHECK_RETCODE(err);

	hx8369bl_brightness = brightness & HX8369BL_MAX_BRIGHT;

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
	props.max_brightness = HX8369BL_MAX_BRIGHT;
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
	bl->props.brightness = HX8369BL_DEF_BRIGHT;

	mipid_bl_update_status(bl);
	return 0;
}
