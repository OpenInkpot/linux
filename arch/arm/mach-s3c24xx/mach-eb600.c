/* linux/arch/arm/mach-s3c2440/mach-eb600.c
 *
 * Copyright (c) 2009
 * 	Alexandr Tsidaev <a.tsidaev@gmail.com>
 *
 * Copyright (c) 2015-2018
 * 	Sergiy Kibrik <sakib@darkstar.site>
 *
 *  Based on linux/arch/arm/mach-s3c2440/mach-eb600.c
 *    by Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/gpio_keys.h>
#include <linux/serial_s3c.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/host.h>
#include <linux/pda_power.h>
#include <linux/s3c_adc_battery.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/regs-clock.h>
#include <mach/fb.h>

#include <linux/eink_apollofb.h>
#include <linux/delay.h>

#include <mach/gpio-samsung.h>
#include <linux/platform_data/mtd-nand-s3c2410.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/leds-s3c24xx.h>
#include <linux/platform_data/mmc-s3cmci.h>
#include <linux/platform_data/usb-s3c2410_udc.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/samsung-time.h>

#include <sound/s3c24xx_uda134x.h>
#include <linux/clk.h>

#include "common.h"

#define EB600_GREEN_LED_PIN S3C2410_GPB(8)

static struct map_desc eb600_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

/* UART configuration */

static struct s3c2410_uartcfg eb600_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	},
};

/* LEDS */

static struct s3c24xx_led_platdata eb600_pdata_led_green = {
	.gpio		= EB600_GREEN_LED_PIN,
	.flags		= S3C24XX_LEDF_ACTLOW,
	.name		= "green",
	.def_trigger	= "nand-disk",
};

static struct platform_device eb600_led_green = {
	.name		= "s3c24xx_led",
	.id		= 1,
	.dev		= {
		.platform_data = &eb600_pdata_led_green,
	},
};

/* NAND driver info */

static struct mtd_partition eb600_nand_part[] = {
	[0] = {
		.name	= "uboot",
		.offset = 0x00000000,
		.size	= 0x00040000,
	},
	[1] = {
		.name	= "kernel",
		.offset = 0x00040000,
		.size	= 0x00200000 - 0x00040000,
	},
	[2] = {
		.name	= "rootfs",
		.offset = 0x00200000,
		.size	= 0x01200000 - 0x00200000,
	},
	[3] = {
		.name	= "app",
		.offset	= 0x01200000,
		.size	= 0x02200000 - 0x01200000,
	},
	[4] = {
		.name	= "update",
		.offset	= 0x02200000,
		.size	= 0x03400000 - 0x02200000,
	},
	[5] = {
		.name	= "other",
		.offset	= 0x03400000,
		.size	= 0x1ff00000 - 0x03400000,
	},
	[6] = {
		.name	= "startbmp",
		.offset	= 0x1ff00000,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct s3c2410_nand_set eb600_nand_sets[] = {
	[0] = {
		.name		= "NAND",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(eb600_nand_part),
		.partitions	= eb600_nand_part
	},
};

static struct s3c2410_platform_nand eb600_nand_info = {
	.tacls		= 15,
	.twrph0		= 45,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(eb600_nand_sets),
	.sets		= eb600_nand_sets,
};

/* Keyboard */

static struct gpio_keys_button eb600_buttons[] = {
	{
		.gpio		= S3C2410_GPG(2),
		.code		= KEY_LEFT,
		.desc		= "Left button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPG(3),
		.code		= KEY_RIGHT,
		.desc		= "Right button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPG(0),
		.code		= KEY_UP,
		.desc		= "Up button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPG(1),
		.code		= KEY_DOWN,
		.desc		= "Down button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF(7),
		.code		= KEY_ENTER,
		.desc		= "Enter button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF(6),
		.code		= KEY_DIRECTION,
		.desc		= "Direction button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF(5),
		.code		= KEY_ESC,
		.desc		= "Esc button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF(4),
		.code		= KEY_PLAYPAUSE,
		.desc		= "Play_Pause button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF(3),
		.code		= KEY_MENU,
		.desc		= "Menu button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPG(4),
		.code		= KEY_KPPLUS,
		.desc		= "Plus button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPG(5),
		.code		= KEY_KPMINUS,
		.desc		= "Minus button",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF(0),
		.code		= KEY_POWER,
		.desc		= "Power button",
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data eb600_keys_data = {
	.name		= "eb600-keys",
	.buttons	= eb600_buttons,
	.nbuttons	= ARRAY_SIZE(eb600_buttons),
};

static struct platform_device eb600_keys = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data = &eb600_keys_data,
	},
};

/* Power Supply */

static int power_supply_init(struct device *dev)
{
	return gpio_request(S3C2410_GPF(2), "USB plugged");
}

static int eb600_is_usb_online(void)
{
	return !gpio_get_value(S3C2410_GPF(2));
}

static void power_supply_exit(struct device *dev)
{
	gpio_free(S3C2410_GPF(2));
}

static char *eb600_supplicants[] = {
	"main-battery",
};

static struct pda_power_pdata power_supply_info = {
	.init			= power_supply_init,
	.is_usb_online		= eb600_is_usb_online,
	.exit			= power_supply_exit,
	.supplied_to		= eb600_supplicants,
	.num_supplicants	= ARRAY_SIZE(eb600_supplicants),
};

static struct resource power_supply_resources[] = {
	[0] = DEFINE_RES_NAMED(IRQ_EINT2, 1, "usb", IORESOURCE_IRQ \
			| IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE),
};

static struct platform_device power_supply = {
	.name		= "pda-power",
	.id		= -1,
	.dev		= {
				.platform_data =
					&power_supply_info,
	},
	.resource	= power_supply_resources,
	.num_resources	= ARRAY_SIZE(power_supply_resources),
};

/* Battery */

/*NB: discharge table, measured samples, may differ accross batteries */
static const struct s3c_adc_bat_thresh bat_lut_noac[] = {
	{ .volt = 4137, .cur = 0, .level = 100 },
	{ .volt = 4087, .cur = 0, .level = 95 },
	{ .volt = 4055, .cur = 0, .level = 90 },
	{ .volt = 4023, .cur = 0, .level = 85 },
	{ .volt = 3960, .cur = 0, .level = 80 },
	{ .volt = 3922, .cur = 0, .level = 75 },
	{ .volt = 3903, .cur = 0, .level = 70 },
	{ .volt = 3872, .cur = 0, .level = 65 },
	{ .volt = 3853, .cur = 0, .level = 60 },
	{ .volt = 3828, .cur = 0, .level = 55 },
	{ .volt = 3809, .cur = 0, .level = 50 },
	{ .volt = 3796, .cur = 0, .level = 45 },
	{ .volt = 3771, .cur = 0, .level = 40 },
	{ .volt = 3752, .cur = 0, .level = 35 },
	{ .volt = 3733, .cur = 0, .level = 30 },
	{ .volt = 3720, .cur = 0, .level = 25 },
	{ .volt = 3708, .cur = 0, .level = 20 },
	{ .volt = 3682, .cur = 0, .level = 15 },
	{ .volt = 3663, .cur = 0, .level = 10 },
	{ .volt = 3651, .cur = 0, .level =  5 },
	{ .volt = 3588, .cur = 0, .level =  0 },
};

/*NB: charge table, measured samples, may differ accross batteries */
static const struct s3c_adc_bat_thresh bat_lut_acin[] = {
	{ .volt = 4257, .cur = 0, .level = 100 },
	{ .volt = 4245, .cur = 0, .level = 90 },
	{ .volt = 4232, .cur = 0, .level = 80 },
	{ .volt = 4200, .cur = 0, .level = 70 },
	{ .volt = 4169, .cur = 0, .level = 60 },
	{ .volt = 4137, .cur = 0, .level = 50 },
	{ .volt = 4099, .cur = 0, .level = 40 },
	{ .volt = 4074, .cur = 0, .level = 30 },
	{ .volt = 4042, .cur = 0, .level = 20 },
	{ .volt = 3998, .cur = 0, .level = 10 },
	{ .volt = 3619, .cur = 0, .level =  0 },
};

static struct s3c_adc_bat_pdata eb600_battery_cfg = {
	.gpio_charge_finished = -1,
	.lut_noac = bat_lut_noac,
	.lut_noac_cnt = ARRAY_SIZE(bat_lut_noac),
	.lut_acin = bat_lut_acin,
	.lut_acin_cnt = ARRAY_SIZE(bat_lut_acin),
	.volt_channel = 0,
	.volt_mult = 6317,
	.current_channel = 1,	/* no current ch connected */
	.current_mult = 0,	/* so try to work-around driver logic */
	.internal_impedance = 200,
};

static struct platform_device eb600_battery = {
	.name		= "s3c-adc-battery",
	.id		= -1,
	.dev = {
		.parent = &s3c_device_adc.dev,
		.platform_data = &eb600_battery_cfg,
	},
};

/* Apollo eInk driver info */

const unsigned int apollo_pins[] = {
	[H_CD]	 S3C2410_GPC(10),
	[H_RW]	 S3C2410_GPC(11),
	[H_DS]	 S3C2410_GPC(9),
	[H_ACK]	 S3C2410_GPC(12),
	[H_WUP]	 S3C2410_GPC(8),
	[H_NRST] S3C2410_GPC(13),
	[H_PWR]	 S3C2410_GPC(14),
	[H_D0]	 S3C2410_GPC(0),
	[H_D1]	 S3C2410_GPC(1),
	[H_D2]	 S3C2410_GPC(2),
	[H_D3]	 S3C2410_GPC(3),
	[H_D4]	 S3C2410_GPC(4),
	[H_D5]	 S3C2410_GPC(5),
	[H_D6]	 S3C2410_GPC(6),
	[H_D7]	 S3C2410_GPC(7),
};

static int apollo_get_ctl_pin(unsigned int pin)
{
	return gpio_get_value(apollo_pins[pin]) ? 1 : 0;
}

static void apollo_set_ctl_pin(unsigned int pin, unsigned char val)
{
	gpio_set_value(apollo_pins[pin], val);
}

static void apollo_set_data_pins_as_output(void)
{
	int i;
	for(i = 0; i <= 7; i++)
		gpio_direction_output(S3C2410_GPC(i), 0);
}

static void apollo_set_data_pins_as_input(void)
{
	int i;
	for(i = 0; i <= 7; i++)
		gpio_direction_input(S3C2410_GPC(i));
}

static void apollo_write_value(unsigned char val)
{
	unsigned long int gpc;
	unsigned long flags;

	local_irq_save(flags);

	gpc = __raw_readl(S3C2410_GPCDAT);
	gpc &= ~0xff;
	gpc |= val;
	__raw_writel(gpc, S3C2410_GPCDAT);

	local_irq_restore(flags);
}

static unsigned char apollo_read_value(void)
{
	unsigned char res = 0;
	unsigned long flags;

	apollo_set_data_pins_as_input();

	local_irq_save(flags);
	res = __raw_readl(S3C2410_GPCDAT);
	local_irq_restore(flags);

	apollo_set_data_pins_as_output();

	return res;
}

int apollo_send_data_fast(void *foo, unsigned char data)
{
	int res = 0;
	unsigned long int gpc;
	unsigned long flags;
	unsigned long timeout = jiffies + 2 * HZ;

	local_irq_save(flags);

	gpc = __raw_readl(S3C2410_GPCDAT);
	gpc &= ~0xff;
	gpc |= data;
	__raw_writel(gpc, S3C2410_GPCDAT);

	gpc &= ~0x0200;
	__raw_writel(gpc, S3C2410_GPCDAT);

	while (__raw_readl(S3C2410_GPCDAT) & 0x1000)
		if (time_before(jiffies, timeout)) {
		} else {
			printk(KERN_ERR "%s: Wait for H_ACK == %u, timeout\n",
					__func__, 0);
			res = 1;
		}

	gpc = __raw_readl(S3C2410_GPCDAT);
	gpc |= 0x0200;
	__raw_writel(gpc, S3C2410_GPCDAT);

	local_irq_restore(flags);

	return res;
}

static int apollo_setuphw(void)
{
	unsigned int i;
	for(i = 0; i < sizeof(apollo_pins) / sizeof(unsigned int); i++)
		gpio_request(apollo_pins[i], "apollo pin");

	apollo_set_data_pins_as_output();
	gpio_direction_input(apollo_pins[H_ACK]);
	gpio_direction_output(apollo_pins[H_PWR], 1);
	udelay(30);
	gpio_direction_output(apollo_pins[H_CD], 0);
	gpio_direction_output(apollo_pins[H_DS], 1);
	gpio_direction_output(apollo_pins[H_RW], 0);
	gpio_direction_output(apollo_pins[H_WUP], 0);
	gpio_direction_output(apollo_pins[H_NRST], 1);

	udelay(20);
	apollo_set_ctl_pin(H_NRST, 0);
	udelay(20);
	apollo_set_ctl_pin(H_NRST, 1);
	udelay(20);

	apollo_set_ctl_pin(H_CD, 0);
	return 0;
}

static void apollo_initialize(void)
{
	apollo_set_ctl_pin(H_RW, 0);
}

static int apollo_init(void)
{
	apollo_setuphw();
	apollo_initialize();

	return 0;
}

static struct eink_apollofb_platdata eb600_apollofb_platdata = {
	.ops = {
		.set_ctl_pin	= apollo_set_ctl_pin,
		.get_ctl_pin	= apollo_get_ctl_pin,
		.read_value	= apollo_read_value,
		.write_value	= apollo_write_value,
		.send_data_fast = apollo_send_data_fast,
		.initialize = apollo_init,
	},
	.defio_delay = HZ / 20,
};

static struct platform_device eb600_apollo = {
	.name		= "eink-apollo",
	.id		= -1,
	.dev		= {
		.platform_data = &eb600_apollofb_platdata,
	},
};

static void set_mmc_power(unsigned char power_mode, unsigned short vdd)
{
	switch (power_mode) {
		case MMC_POWER_OFF:
			gpio_set_value(S3C2410_GPB(6), 0);
			break;
		case MMC_POWER_UP:
		case MMC_POWER_ON:
			gpio_set_value(S3C2410_GPB(6), 1);
			break;
		default:
			break;
	}
}

/* MMC driver info */

static struct s3c24xx_mci_pdata eb600_mmc_cfg = {
	.gpio_detect    = S3C2410_GPG(7),
	.set_power      = set_mmc_power,
	.ocr_avail      = MMC_VDD_32_33,
};

/* Audio */

static struct s3c24xx_uda134x_platform_data eb600_audio_pins = {
	.l3_clk = S3C2410_GPB(4),
	.l3_mode = S3C2410_GPB(2),
	.l3_data = S3C2410_GPB(3),
	.model = UDA134X_UDA1345
};

static struct platform_device eb600_audio = {
	.name           = "s3c24xx_uda134x",
	.id             = 0,
	.dev            = {
		.platform_data  = &eb600_audio_pins,
	},
};

/* UDC */

static struct s3c2410_udc_mach_info eb600_udc_platform_data = {
	.pullup_pin		= S3C2410_GPG(6),
};

static struct platform_device *eb600_devices[] __initdata = {
	&s3c_device_nand,
	&s3c_device_usbgadget,
	&s3c_device_wdt,
	&s3c_device_sdi,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&s3c_device_rtc,
	&s3c_device_adc,
	&eb600_led_green,
	&eb600_apollo,
	&eb600_keys,
	&eb600_battery,
	&power_supply,
	&eb600_audio,
};

static void eb600_power_off(void)
{
	int pin_state = 1;
	while (pin_state)
	{
		s3c_gpio_cfgpin(S3C2410_GPB(5), S3C2410_GPIO_OUTPUT);
		gpio_set_value(S3C2410_GPB(5), 0);
		pin_state = gpio_get_value(S3C2410_GPB(5));
	}
}

static long eb600_panic_blink(int state)
{
	gpio_set_value(EB600_GREEN_LED_PIN, 1);
	mdelay(200);
	gpio_set_value(EB600_GREEN_LED_PIN, 0);
	mdelay(200);

	return 400;
}

static void __init eb600_map_io(void)
{
	s3c24xx_init_io(eb600_iodesc, ARRAY_SIZE(eb600_iodesc));
	s3c24xx_init_uarts(eb600_uartcfgs, ARRAY_SIZE(eb600_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM3, SAMSUNG_PWM4);
	s3c_pm_init();
}

static void __init eb600_init_time(void)
{
	s3c2440_init_clocks(12000000);
	samsung_timer_init();
}

static void __init eb600_init_gpio(void)
{
	int i;

	// Green led
	s3c_gpio_cfgpin(eb600_pdata_led_green.gpio, S3C2410_GPIO_OUTPUT);
	gpio_request_one(eb600_pdata_led_green.gpio,
			GPIOF_OUT_INIT_HIGH, NULL);
	gpio_free(eb600_pdata_led_green.gpio);

	/* USB */
	/* Switching FSUSB20L to s3c onboard usb */
	s3c_gpio_cfgpin(S3C2410_GPH(8), S3C2410_GPIO_OUTPUT);
	gpio_request_one(S3C2410_GPH(8), GPIOF_OUT_INIT_HIGH, "USBSW");

	__raw_writel(0x155555, S3C2410_GPBCON);
	__raw_writel(0x1a0, S3C2410_GPBDAT);

	__raw_writel(0xEFFF, S3C2410_GPCUP);

	__raw_writel(0xFFFF, S3C2410_GPDUP);
	__raw_writel(0xD5555555, S3C2410_GPDCON);
	__raw_writel(0x0, S3C2410_GPDDAT);

	for(i = 0; i < 13; i++) {
		s3c_gpio_cfgpin(S3C2410_GPJ(i), S3C2410_GPIO_OUTPUT);
		s3c_gpio_setpull(S3C2410_GPJ(i), S3C_GPIO_PULL_NONE);
	}
}

static void __init eb600_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);

	/* Turn off suspend on both USB ports, and switch the
	 * selectable USB port to USB device mode. */

	s3c2410_modify_misccr(S3C2410_MISCCR_USBHOST |
			      S3C2410_MISCCR_USBSUSPND0 |
			      S3C2410_MISCCR_USBSUSPND1, 0x0);

	s3c_nand_set_platdata(&eb600_nand_info);
	s3c24xx_mci_set_platdata(&eb600_mmc_cfg);
	s3c24xx_udc_set_platdata(&eb600_udc_platform_data);

	eb600_init_gpio();

	platform_add_devices(eb600_devices, ARRAY_SIZE(eb600_devices));

	pm_power_off = &eb600_power_off;
	panic_blink = eb600_panic_blink;
}

MACHINE_START(EB600, "Netronix EB-600")
	.atag_offset	= 0x100,
	.map_io		= eb600_map_io,
	.init_machine	= eb600_machine_init,
	.init_irq	= s3c2440_init_irq,
	.init_time	= eb600_init_time,
MACHINE_END
