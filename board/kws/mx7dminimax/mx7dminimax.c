/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*   Group      SOC            Hybrid       TSI
 *    I2C-1     PMIC, EEPROM
 *    I2C-2                     used
 *    I2C-3                     used
 *    I2C-4     RTC
 * 
 */
 
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <asm/string.h>
#include <stdlib.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../common/pfuze.h"
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#include <asm/arch/crm_regs.h>

#ifdef CONFIG_VIDEO_MXS
#include <linux/fb.h>
#include <mxsfb.h>
#endif

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/


DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | \
	PAD_CTL_PUS_PU100KOHM | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define ENET_PAD_CTRL	  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM) 
#define ENET_RX_PAD_CTRL  (PAD_CTL_PUS_PU100KOHM | PAD_CTL_DSE_3P3V_49OHM)
#define ENET_PAD_CTRL_MII (PAD_CTL_DSE_3P3V_32OHM)
#define ENET_PAD_CTRL_MII_PU (PAD_CTL_PUS_PU5KOHM | PAD_CTL_DSE_3P3V_49OHM) 

#define I2C_PAD_CTRL    (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
	PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU100KOHM)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
	PAD_CTL_DSE_3P3V_49OHM)

#define QSPI_PAD_CTRL	\
	(PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define SPI_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define BUTTON_PAD_CTRL    (PAD_CTL_PUS_PU5KOHM | PAD_CTL_DSE_3P3V_98OHM)

#define NAND_PAD_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_SRE_SLOW | PAD_CTL_HYS)

#define NAND_PAD_READY0_CTRL (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU5KOHM)

#define PIO_PAD_CTRL 	NO_PAD_CTRL
#define EPDC_PAD_CTRL	0x0


#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1 for PMIC, EEPROM */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C1_SCL__GPIO4_IO8 | PC,
		.gp = IMX_GPIO_NR(4, 8),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C1_SDA__GPIO4_IO9 | PC,
		.gp = IMX_GPIO_NR(4, 9),
	},
};

/* I2C2 for baseboard */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C2_SCL__I2C2_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C2_SCL__GPIO4_IO10 | PC,
		.gp = IMX_GPIO_NR(4, 10),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C2_SDA__I2C2_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C2_SDA__GPIO4_IO11 | PC,
		.gp = IMX_GPIO_NR(4, 11),
	},
};

/* I2C3 for baseboard */
struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C3_SCL__I2C3_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C3_SCL__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C3_SDA__I2C3_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C3_SDA__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13),
	},
};

/* I2C4  RTC at 0x68 */
struct i2c_pads_info i2c_pad_info4 = {
	.scl = {
		.i2c_mode = MX7D_PAD_SAI1_RX_SYNC__I2C4_SCL | PC,
		.gpio_mode = MX7D_PAD_SAI1_RX_SYNC__GPIO6_IO16 | PC,
		.gp = IMX_GPIO_NR(6, 16),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_ENET1_RGMII_TD3__I2C4_SDA | PC,
		.gpio_mode = MX7D_PAD_ENET1_RGMII_TD3__GPIO7_IO9 | PC,
		.gp = IMX_GPIO_NR(7, 18),
	},
};
#endif



/* yyyy  muss stark überarbeitet werden... */
/*48 Byte manufacturer data block in EEPROM */
struct manufacturer_data {
	char 	tag[6];
	char    charge; 			/* manufacturer date (e.g. A) */
	char    version;			/* hardware version (e.g. "1" */
	unsigned short	hardware; 	/* hardware version (e.g. V1) */
	char	serial_number[20]; 	/* serial number (0...99999) */
	char 	macaddr[20]; 		/* MAC adress */
	char 	macaddr1[20]; 		/* MAC1 adress */
	char	name[20];			/* device name (in CHIP.INI??) */
} t_manufacturer_data;

extern int cli_readline(const char *const prompt);

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const wdog_pads[] = {
	MX7D_PAD_GPIO1_IO00__WDOG1_WDOG_B | MUX_PAD_CTRL(NO_PAD_CTRL),
//	MX7D_PAD_GPIO1_IO00__GPIO1_IO0 | MUX_PAD_CTRL(NO_PAD_CTRL),	//test only
};

static iomux_v3_cfg_t const uart1_pads[] = {
	MX7D_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD2__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const support_pads[] = {
	MX7D_PAD_EPDC_GDSP__GPIO2_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL),	// MM EEPROM WP (1=WP)
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX7D_PAD_SD1_CLK__SD1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_CMD__SD1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA0__SD1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA1__SD1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA2__SD1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD1_DATA3__SD1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_EPDC_DATA08__GPIO2_IO8 | MUX_PAD_CTRL(USDHC_PAD_CTRL),	// CardDetect
};

static iomux_v3_cfg_t const usdhc3_emmc_pads[] = {
	MX7D_PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_STROBE__SD3_STROBE	 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_RESET_B__GPIO6_IO11 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


#define BOARD_REV_C  0x300
#define BOARD_REV_B  0x200
#define BOARD_REV_A  0x100

static int mx7minimax_rev(void)
{
	int ret;
	ret = BOARD_REV_A;	//todo
	return ret;
}

u32 get_board_rev(void)
{
	int rev = mx7minimax_rev();

	return (get_cpu_rev() & ~(0xF << 8)) | rev;
}


#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	//MX7D_PAD_EPDC_DATA12__GPIO2_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), // MM: UART7-RXD

	MX7D_PAD_LCD_CLK__LCD_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_ENABLE__LCD_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_HSYNC__LCD_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_EPDC_DATA02__LCD_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA00__LCD_DATA0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA01__LCD_DATA1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA02__LCD_DATA2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA03__LCD_DATA3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA04__LCD_DATA4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA05__LCD_DATA5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA06__LCD_DATA6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA07__LCD_DATA7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA08__LCD_DATA8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA09__LCD_DATA9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA10__LCD_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA11__LCD_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA12__LCD_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA13__LCD_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA14__LCD_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA15__LCD_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA16__LCD_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA17__LCD_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA18__LCD_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA19__LCD_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA20__LCD_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA21__LCD_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA22__LCD_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA23__LCD_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	MX7D_PAD_EPDC_DATA15__GPIO2_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),	//MM: PWR-enable  check mode!
	MX7D_PAD_LCD_RESET__GPIO3_IO4	| MUX_PAD_CTRL(NO_PAD_CTRL),	//MM: LCD_RESET n.c. on TP MA64
};

static iomux_v3_cfg_t const pwm_pads[] = {
	/* Use GPIO for Brightness adjustment, duty cycle = period */
	MX7D_PAD_GPIO1_IO01__GPIO1_IO1 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

struct lcd_panel_info_t {
	unsigned int lcdif_base_addr;
	int depth;
	void	(*enable)(struct lcd_panel_info_t const *dev);
	struct fb_videomode mode;
};

void do_enable_parallel_lcd(struct lcd_panel_info_t const *dev)
{
	//printf("++MM: do_enable_parallel_lcd() -entry\n");
	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));
	imx_iomux_v3_setup_multiple_pads(pwm_pads, ARRAY_SIZE(pwm_pads));

#if 0
	/* Reset LCD  (but is n.c. */
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 1);
#endif

	/* MM: Set LCD_PWR on */
	gpio_direction_output(IMX_GPIO_NR(2, 15) , 0);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);
}

static struct lcd_panel_info_t const displays[] = {{
	.lcdif_base_addr = ELCDIF1_IPS_BASE_ADDR,
	.depth = 24,
	.enable	= do_enable_parallel_lcd,
	.mode	= {
		.name			= "TFT800x480",
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30066,
		.left_margin    = 8,
		.right_margin   = 4,
		.upper_margin   = 2,
		.lower_margin   = 4,
		.hsync_len      = 41,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");

	if (!panel) {
		panel = displays[0].mode.name;
		printf("No panel detected: default to %s\n", panel);
		i = 0;
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = mxs_lcd_panel_setup(displays[i].mode, displays[i].depth,
				    displays[i].lcdif_base_addr);
		if (!ret) {
			if (displays[i].enable)
				displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}
#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_support(void)
{
    imx_iomux_v3_setup_multiple_pads(support_pads, ARRAY_SIZE(support_pads));
	gpio_direction_output(IMX_GPIO_NR(2, 27) , 0);  /* 0 = EEPROM allow writing */
}


#ifdef CONFIG_FSL_ESDHC

#define USDHC1_CD_GPIO	IMX_GPIO_NR(2, 8)
//#define USDHC1_PWR_GPIO	IMX_GPIO_NR(5, 2)  //Eval: SD1 Power on/off
#define USDHC3_PWR_GPIO IMX_GPIO_NR(6, 11)

static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR},
};

int mmc_get_env_devno(void)
{
	struct bootrom_sw_info **p =
		(struct bootrom_sw_info **)ROM_SW_INFO_ADDR;

	u8 boot_type = (*p)->boot_dev_type;
	u8 dev_no = (*p)->boot_dev_instance;

	/* If not boot from sd/mmc, use default value */
	if ((boot_type != BOOT_TYPE_SD) && (boot_type != BOOT_TYPE_MMC))
		return CONFIG_SYS_MMC_ENV_DEV;

	if (2 == dev_no)
		dev_no--;

	return dev_no;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	if (1 == dev_no)
		dev_no++;

	return dev_no;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;
return (1); /* yyyy */
	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = 1; /* Assume uSDHC3 emmc is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc2                    USDHC3 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_request(USDHC1_CD_GPIO, "usdhc1_cd");
			gpio_direction_input(USDHC1_CD_GPIO);
			//gpio_request(USDHC1_PWR_GPIO, "usdhc1_pwr");
			//gpio_direction_output(USDHC1_PWR_GPIO, 0);
			//udelay(500);
			//gpio_direction_output(USDHC1_PWR_GPIO, 1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_emmc_pads, ARRAY_SIZE(usdhc3_emmc_pads));
			gpio_request(USDHC3_PWR_GPIO, "usdhc3_pwr");
			gpio_direction_output(USDHC3_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC3_PWR_GPIO, 1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return 0;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int check_mmc_autodetect(void)
{
	char *autodetect_str = getenv("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

void board_late_mmc_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_devno();

	if (!check_mmc_autodetect())
		return;

	setenv_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	setenv("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

#endif


#ifdef CONFIG_FEC_MXC

static iomux_v3_cfg_t const fec1_pads[] = {

/*   MM Module default: PHY delivers clock to SOC for enet0 
 *    - set TX_CLK to ENET_REF_CLK1 and
 *    - set IOMUX_GPR_GPR1 "disable clock output", "enable ext. clock")
 *   See: setup_fec()
 */
	MX7D_PAD_ENET1_TX_CLK__CCM_ENET_REF_CLK1 | MUX_PAD_CTRL(ENET_PAD_CTRL)  | MUX_MODE_SION,
		
	MX7D_PAD_ENET1_RGMII_TD0__ENET1_RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_TD1__ENET1_RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD0__ENET1_RGMII_RD0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RD1__ENET1_RGMII_RD1 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),

	MX7D_PAD_ENET1_RGMII_TX_CTL__ENET1_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RX_CTL__ENET1_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_ENET1_RGMII_RXC__ENET1_RX_ER | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),

 /*  There is no external pullup on MDIO-Line. So activate internal 5K Pullup */ 
	MX7D_PAD_UART1_RX_DATA__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL_MII_PU),
	MX7D_PAD_GPIO1_IO11__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),

	MX7D_PAD_ENET1_RGMII_RD3__GPIO7_IO3 | MUX_PAD_CTRL(ENET_PAD_CTRL),			//MM ETH_INT
	MX7D_PAD_ENET1_RGMII_TXC__GPIO7_IO11 | MUX_PAD_CTRL(ENET_PAD_CTRL),			//MM ETH_SIGDET 
};

static iomux_v3_cfg_t const fec2_pads[] = {
  //MX7D_PAD_ENET1_COL__CCM_EXT_CLK4	| MUX_PAD_CTRL(ENET_PAD_CTRL),			//MM ALT6 Input
    MX7D_PAD_ENET1_COL__GPIO7_IO15 | MUX_PAD_CTRL(PIO_PAD_CTRL),				// set to PIO
    MX7D_PAD_EPDC_BDR0__CCM_ENET_REF_CLK2 | MUX_PAD_CTRL(ENET_PAD_CTRL) | MUX_MODE_SION, //MM ALT3 Output
	
	MX7D_PAD_EPDC_GDRL__ENET2_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),		//ok (="TX_EN")
	MX7D_PAD_EPDC_SDCE2__ENET2_RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX7D_PAD_EPDC_SDCE3__ENET2_RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX7D_PAD_EPDC_SDCE0__ENET2_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDCLK__ENET2_RGMII_RD0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDLE__ENET2_RGMII_RD1	 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX7D_PAD_EPDC_SDCE1__GPIO2_IO21		 | MUX_PAD_CTRL(PIO_PAD_CTRL),			//MM (="RX_ER not used")

	MX7D_PAD_GPIO1_IO14__ENET2_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL_MII),
	MX7D_PAD_SD2_WP__ENET2_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL_MII),

	MX7D_PAD_EPDC_DATA10__GPIO2_IO10 | MUX_PAD_CTRL(PIO_PAD_CTRL), //check pad!!! //ETH_INT 
};
	
static void setup_iomux_fec(void)
{
	if (CONFIG_FEC_ENET_DEV == 0) {
		imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
	} else {
		imx_iomux_v3_setup_multiple_pads(fec2_pads, ARRAY_SIZE(fec2_pads));
		/*
		 *  Switch KSZ8863 doesn't use the "rx_er" signal.
		 *  But it generates ca. 0.1% framing errors if this signal is left floating.
		 *  (Error in new datasheet? See older datasheet!)
		 *  So set it to output and low level.
		 */
		gpio_direction_output(IMX_GPIO_NR(2, 21) , 0);  /* set rx_er = 0 */
	}
}

#ifdef failer
static int do_ssnum(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{	
	struct manufacturer_data *manu = &t_manufacturer_data;
	
	int res;
	char c;
	char serial[32];
	char *prefix = "MMX-AP1-";
    char version, charge, num;

	/*MMX-AP1-1A000001*/
	res = cli_readline("Enter serial number (e.g. 1A0002): ");
	console_buffer[res] = 0; /*extern, console I/O buffer size CONFIG_SYS_CBSIZE + 1 */

    version = console_buffer[0];
    charge = console_buffer[1];
	num = simple_strtoul(&console_buffer[2], NULL, 10);
	sprintf (serial, "%s%c%c%05d", prefix, version, charge, num);
	printf("Complete serial: %s\n", serial);
	strcpy(manu->serial_number, serial); 

	res = cli_readline("Write to EEPROM (y/n)? ");
	c = console_buffer[0];

	if (c != 'y' && c != 'Y' && c != 'z' && c != 'Z'){
		printf ("Nothing written \n");
		return 0;
	} else {
		i2c_write(0x50,0,2,(uint8_t *)manu->serial_number,20);
		printf("New Serial number written to EEPROM! \n");
		return 0;
	}
}

#endif

int board_eth_init(bd_t *bis)
{
	int ret;

	printf("++MM: board_eth_init()\n");
	setup_iomux_fec();

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);

	if (ret)
		printf("FEC1 MXC: %s:failed\n", __func__);

	return ret;
}

static int setup_fec(int fec_id)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;
	int ret;

 	if (fec_id == 0) {
		if (0) {     
			/* Use 50M anatop REF_CLK1 for ENET1, clear gpr1[13], set gpr1[17]
			and output it on the pin */
			//printf ("++MM: set refclk for enet1 to output\n");
			clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
				IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK,
				IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK);
		} else {
			/* Clock is input from PHY */
			/* Use 50M external CLK for ENET1, set gpr1[13], clear gpr1[17] */
			/* See Manual: IOMUXC_GPR_GPR1 register ! */
			//printf ("++MM: set refclk for enet1 to input\n");
			clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
				IOMUXC_GPR_GPR1_GPR_ENET1_CLK_DIR_MASK,
				IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK);
		}
    }
    else
    {
		if (1) {     
			/* Clock output to PHY */
			/* Use 50MHz anatop REF_CLK1 for ENET1, clear gpr1[14], set gpr1[18]
		 	* and output it on the pin */
			printf ("++MM: set refclk for enet2 to output\n");
			clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
				IOMUXC_GPR_GPR1_GPR_ENET2_TX_CLK_SEL_MASK,
				IOMUXC_GPR_GPR1_GPR_ENET2_CLK_DIR_MASK);
		} else {
			/* Clock is input from PHY */
			/* Use 50M external CLK for ENET1, set gpr1[14], clear gpr1[18] */
			clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],
				IOMUXC_GPR_GPR1_GPR_ENET2_CLK_DIR_MASK,
				IOMUXC_GPR_GPR1_GPR_ENET2_TX_CLK_SEL_MASK);
		}
	}
	
	ret = set_clk_enet(ENET_50MHz);
	if (ret)
		return ret;

	return 0;
}


int board_phy_config(struct phy_device *phydev)
{
	//printf ("++MM: board_phy_config()\n");	

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	
	return 0;
}
#endif

#ifdef CONFIG_MXC_EPDC
   error: "do not enable epdc"
#endif


int board_early_init_f(void)
{
	setup_iomux_uart();
	setup_iomux_support();
	
#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2); 
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3); 
//	setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info4);  //-> yyyy super crash
#endif

	return 0;
}

/* yyyy  muss stark überarbeitet werden... */
static int get_sernum_from_string(struct manufacturer_data *manu)
{
   int slen = strlen(manu->serial_number);
   char *p = &manu->serial_number[slen-1];
   char *ptr; 
   int  snum;
   
   if (slen < 4)
      goto err;
      
   while (slen-- > 0)
   {   
      if (*p == '-') {
         if (strlen(p) < 3)
			goto err;
         p++;
         manu->version = *p++;
         manu->charge  = *p++;
         snum = simple_strtoul(p, &ptr, 10);
	//	 if (snum > 0 && strlen(ptr) == 0)
			return (snum);
      }   
      p--;   
   }
 err:
   printf ("XXX erroneous serial number XXX\n");
   manu->version = '\0';
   manu->charge  = '@';
   return 0;
}

int board_init(void)
{
	struct manufacturer_data *manu = &t_manufacturer_data;
	char buf[16];
    uint sernum;
    char *prefix = "00:12:91:6D";
	int ret;
	uint addr = 0;

	ret = i2c_read(0x50, addr, 2, (uint8_t*)buf, 16);
	if (ret) {
		printf("Can't read EEPROM\n");
	}
	
	strcpy(manu->serial_number, buf);

	sernum = get_sernum_from_string (manu);
	sernum *= 2;
	sprintf(manu->macaddr,"%s:%02X:%02X", prefix, sernum/256, sernum&0xff);
	sernum++;
	sprintf(manu->macaddr1,"%s:%02X:%02X", prefix,  sernum/256, sernum&0xff);

	printf("Serial: %s\n", manu->serial_number);
	printf("MAC:   %s\n", manu->macaddr);
	printf("MAC1:  %s\n", manu->macaddr1);
	
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif
	return 0;
}

#ifdef CONFIG_CMD_BMODE
/* remember:
#define MAKE_CFGVAL(cfg1, cfg2, cfg3, cfg4) \
	((cfg4) << 24) | ((cfg3) << 16) | ((cfg2) << 8) | (cfg1)
 see: soc.c
 but: is not supported for iMX7!
*/
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x10, 0x10, 0x00, 0x00)},	//-> 0x00001010
	{"emmc", MAKE_CFGVAL(0x10, 0x2a, 0x00, 0x00)},	//-> 0x00002a10
	{NULL,   0},
};
#endif

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg, rev_id;

	ret = power_pfuze3000_init(I2C_PMIC);
	if (ret)
		return ret;

	p = pmic_get("PFUZE3000");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE3000_DEVICEID, &reg);
	pmic_reg_read(p, PFUZE3000_REVID, &rev_id);
	printf("PMIC:  PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", reg, rev_id);

	/* disable Low Power Mode during standby mode */
	pmic_reg_read(p, PFUZE3000_LDOGCTL, &reg);
	reg |= 0x1;
	pmic_reg_write(p, PFUZE3000_LDOGCTL, reg);

	/* SW1A/1B mode set to APS/APS */
	reg = 0x8;
	pmic_reg_write(p, PFUZE3000_SW1AMODE, reg);
	pmic_reg_write(p, PFUZE3000_SW1BMODE, reg);

	/* SW1A/1B standby voltage set to 0.975V */
	reg = 0xb;
	pmic_reg_write(p, PFUZE3000_SW1ASTBY, reg);
	pmic_reg_write(p, PFUZE3000_SW1BSTBY, reg);

	/* set SW1B normal voltage to 0.975V */
	pmic_reg_read(p, PFUZE3000_SW1BVOLT, &reg);
	reg &= ~0x1f;
	reg |= PFUZE3000_SW1AB_SETP(975);
	pmic_reg_write(p, PFUZE3000_SW1BVOLT, reg);

	return 0;
}
#endif


int board_late_init(void)
{
	struct manufacturer_data *manu = &t_manufacturer_data;
	char *bootdev;

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_init();
#endif

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	setenv("sernum", manu->serial_number);
	setenv("ethaddr", manu->macaddr);
	setenv("eth1addr", manu->macaddr1);

	switch (get_boot_device()) {
		case SD1_BOOT:
		case MMC1_BOOT:
			bootdev = "SD/MMC";
			break;
		case SD3_BOOT:
		case MMC3_BOOT:
			bootdev = "eMMC";
			break;
		default:
			bootdev = "unknown device";
			break;
	}
    printf ("Booted from %s\n", bootdev);

	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	return 0;
}

int checkboard(void)
{
//	int rev = mx7minimax_rev();
	char *revname = "A";

	printf("Board: KWS i.MX7D MiniMax Rev%s\n", revname);

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX7

/* Minimax: Only USB2-power is not switchable */

iomux_v3_cfg_t const usb_otg2_pads[] = {
	MX7D_PAD_GPIO1_IO07__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:	
		break;
	case 1:	
		imx_iomux_v3_setup_multiple_pads(usb_otg2_pads,
					 ARRAY_SIZE(usb_otg2_pads));
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}
#endif

#ifdef CONFIG_FSL_FASTBOOT	// not defined for SabreSD...
  
void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}
}

#ifdef CONFIG_ANDROID_RECOVERY  // not defined for SabreSD...

/* Use S3 button for recovery key */
#define GPIO_VOL_DN_KEY IMX_GPIO_NR(5, 10)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX7D_PAD_SD2_WP__GPIO5_IO10 | MUX_PAD_CTRL(BUTTON_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
	int button_pressed = 0;
	int recovery_mode = 0;

	recovery_mode = recovery_check_and_clean_flag();

	/* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
		ARRAY_SIZE(recovery_key_pads));

	gpio_direction_input(GPIO_VOL_DN_KEY);

	if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
	}

	return recovery_mode || button_pressed;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc1 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}
#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef failer
U_BOOT_CMD(
	__ssnum, CONFIG_SYS_MAXARGS, 1, do_ssnum,
	NULL,
	NULL
);
#endif
