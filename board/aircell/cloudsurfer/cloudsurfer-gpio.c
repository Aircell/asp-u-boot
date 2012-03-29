/*
 * (C) Copyright 2008
 * Logic Produc Development, <www.logicpd.com>
 * Peter Barada <peter.barada@logicpd.com>
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

#include <common.h>
#include <asm/arch/cpu.h>
#include <asm/io.h>
// #include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
// #include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <i2c.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#include "cloudsurfer-gpio.h"

#define NUM_OF_BITS_IN_REG                 32
#define MAX_GPIO_PINS                      192
#define GPIO_PIN                           1
#define GPIO_MAX_MODULES			       6

/* GPIO base address */
#define	GPIO1_MODULE_BA                    0x48310000
#define	GPIO2_MODULE_BA                    0x49050000
#define GPIO3_MODULE_BA                    0x49052000
#define GPIO4_MODULE_BA                    0x49054000
#define GPIO5_MODULE_BA                    0x49056000
#define GPIO6_MODULE_BA                    0x49058000

#define GPIO_DATAIN			((uint32_t)0x038)
#define GPIO_DATAOUT			((uint32_t)0x03C)
#define GPIO_OE				((uint32_t)0x034)


/* int and long both fit to 32 bits */
typedef volatile uint32_t*  PREG_U32;
typedef volatile int32_t*  PREG_S32;

#define in_regl(offSet)			(*(PREG_U32)(offSet))
#define out_regl(offSet, value)       (*(PREG_U32)(offSet) = (uint32_t)(value))

static uint32_t g_gpio_module_base_address[GPIO_MAX_MODULES]
				= {GPIO1_MODULE_BA, GPIO2_MODULE_BA, GPIO3_MODULE_BA,
					 GPIO4_MODULE_BA, GPIO5_MODULE_BA, GPIO6_MODULE_BA};

uint32_t check_gpio_pin_num(uint32_t pin_num)
{
  return (pin_num > MAX_GPIO_PINS);
}

uint32_t get_module_pin_mask(uint32_t pin_num, uint32_t *module_num, uint32_t *offset, uint32_t *pinmask)
{
  uint32_t snum, ret_val;

  *module_num = pin_num / NUM_OF_BITS_IN_REG + 1;
  snum = (*module_num-1)*NUM_OF_BITS_IN_REG;
  *offset = pin_num - snum;
  ret_val = check_gpio_pin_num(pin_num);
  if (ret_val)
    return ret_val;

  *pinmask = GPIO_PIN<<*offset;
  return 0;
}


void gpio_write_output_pin(int module_num, uint32_t pin_mask, uint32_t data)
{
  uint32_t temp, gpio_data_out_reg;

  gpio_data_out_reg = (g_gpio_module_base_address[module_num-1]+GPIO_DATAOUT);

  temp = in_regl(gpio_data_out_reg);
  temp = temp & ~pin_mask;

  out_regl(gpio_data_out_reg, (temp | (data & pin_mask)));
}

void gpio_read_input_pin(uint32_t module_num, uint32_t pin_mask, uint32_t *data)
{
  uint32_t gpio_data_in_reg, temp;

  gpio_data_in_reg = (g_gpio_module_base_address[module_num-1]+GPIO_DATAIN);

  temp = in_regl(gpio_data_in_reg);
  *data = temp & pin_mask;
}

uint32_t pin_get_gpio_input(uint32_t pin_num)
{
  uint32_t module_num, pinmask, offset, data;

  get_module_pin_mask(pin_num, &module_num, &offset, &pinmask);

  gpio_read_input_pin(module_num, (1<<offset), &data);
  data >>= offset;

  // printf("%s:%d pin %d data %d\n", __FUNCTION__, __LINE__, pin_num, data);

  return data;
}


uint32_t pin_set_gpio_dataout(uint32_t pin_num, uint32_t set)
{
  uint32_t module_num, pinmask, offset, ret_val;

  // printf("%s:%d pin %d set %d\n", __FUNCTION__, __LINE__, pin_num, set);

  ret_val = get_module_pin_mask(pin_num, &module_num, &offset, &pinmask);

  if (set)
    gpio_write_output_pin(module_num, (1<<offset), (1<<offset));
  else
    gpio_write_output_pin(module_num, (1<<offset), (0<<offset));

  return ret_val;
}

uint32_t set_gpio_in_out(uint32_t module_num, uint32_t pin_mask, uint32_t io_mask)
{
  uint32_t temp_oe, gpio_pin_output_en_reg;

  gpio_pin_output_en_reg = (g_gpio_module_base_address[module_num-1]+GPIO_OE);

  temp_oe = in_regl(gpio_pin_output_en_reg);
  temp_oe &= ~pin_mask;
  temp_oe |= io_mask;

  out_regl(gpio_pin_output_en_reg, temp_oe);

  return 0;
}

uint32_t pin_init_gpio(uint32_t pin_num, uint32_t in_out)
{
	uint32_t module_num, pinmask, offset, ret_val;

	ret_val = get_module_pin_mask(pin_num, &module_num, &offset, &pinmask);

	set_gpio_in_out(module_num, pinmask, in_out<<offset);

	return ret_val;
}

/* Restor GPIO_OE to reset state */
void gpio_oe_force_all_input(void)
{
	int module;
	for (module=0; module<ARRAY_SIZE(g_gpio_module_base_address); ++module) {
		set_gpio_in_out(module+1, 0, 0xffffffff);
	}
}

void cloudsurfer_gpios(void) 
{

	/* Setup some of the GPIO pins..... */
	/* Software reset */
	if (!omap_request_gpio(AIRCELL_SOFTWARE_RESET)) {
		omap_set_gpio_dataout(AIRCELL_SOFTWARE_RESET, 1);
		omap_set_gpio_direction(AIRCELL_SOFTWARE_RESET, 0);
	}

	/* Enable 5V Analog - OFF */
	if (!omap_request_gpio(AIRCELL_5VA_ENABLE)) {
		omap_set_gpio_dataout(AIRCELL_5VA_ENABLE, 0);
		omap_set_gpio_direction(AIRCELL_5VA_ENABLE, 0);
	}

	/* Enable 5V Digital - OFF */
	if (!omap_request_gpio(AIRCELL_5VD_ENABLE)) {
		omap_set_gpio_dataout(AIRCELL_5VD_ENABLE, 0);
		omap_set_gpio_direction(AIRCELL_5VD_ENABLE, 0);
	}

	/* Battery Powered detect */
	if (!omap_request_gpio(AIRCELL_BATTERY_POWERED)) {
		omap_set_gpio_direction(AIRCELL_BATTERY_POWERED, 1);
	}

	/* Enable 1.8V - ON */
	if (!omap_request_gpio(AIRCELL_18V_ENABLE)) {
		omap_set_gpio_direction(AIRCELL_18V_ENABLE, 0);
		omap_set_gpio_dataout(AIRCELL_18V_ENABLE, 1);
	}

	/* LCD reset - Active high */
	if (!omap_request_gpio(AIRCELL_LCD_RESET)) {
		omap_set_gpio_direction(AIRCELL_LCD_RESET, 0);
		omap_set_gpio_dataout(AIRCELL_LCD_RESET, 1);
	}

	/* Crade and power detect */
	if (!omap_request_gpio(AIRCELL_POWER_APPLIED_DETECT)) {
		omap_set_gpio_direction(AIRCELL_POWER_APPLIED_DETECT, 1);
	}

	/* LED driver IC - OFF */
	if (!omap_request_gpio(AIRCELL_LED_ENABLE)) {
		omap_set_gpio_direction(AIRCELL_LED_ENABLE, 0);
		omap_set_gpio_dataout(AIRCELL_LED_ENABLE, 0);
	}

	/* earpiece enable - OFF */
	if (!omap_request_gpio(AIRCELL_EARPIECE_ENABLE)) {
		omap_set_gpio_direction(AIRCELL_EARPIECE_ENABLE, 0);
		omap_set_gpio_dataout(AIRCELL_EARPIECE_ENABLE, 0);
	}

	/* ringer audio enable - OFF  */
	if (!omap_request_gpio(AIRCELL_RINGER_ENABLE)) {
		omap_set_gpio_direction(AIRCELL_RINGER_ENABLE, 0);
		omap_set_gpio_dataout(AIRCELL_RINGER_ENABLE, 0);
	}

	/* volume up switch */
	if (!omap_request_gpio(AIRCELL_VOLUME_UP_DETECT)) {
		omap_set_gpio_direction(AIRCELL_VOLUME_UP_DETECT, 1);
	}

	/* headset detection */
	if (!omap_request_gpio(AIRCELL_HEADSET_DETECT)) {
		omap_set_gpio_direction(AIRCELL_HEADSET_DETECT, 1);
	}

	/* volume down switch */
	if (!omap_request_gpio(AIRCELL_VOLUME_DOWN_DETECT)) {
		omap_set_gpio_direction(AIRCELL_VOLUME_DOWN_DETECT, 1);
	}

	/* touch panel reset active low - In reset */
	if (!omap_request_gpio(AIRCELL_TOUCH_RESET)) {
		omap_set_gpio_direction(AIRCELL_TOUCH_RESET, 0);
		omap_set_gpio_dataout(AIRCELL_TOUCH_RESET, 0);
	}

	/* Touch screen interrupt */
	if (!omap_request_gpio(AIRCELL_TOUCH_INTERRUPT)) {
		omap_set_gpio_direction(AIRCELL_TOUCH_INTERRUPT, 1);
	}

	/* Proximity interrupt */
	if (!omap_request_gpio(AIRCELL_PROX_INTERRUPT)) {
		omap_set_gpio_direction(AIRCELL_PROX_INTERRUPT, 1);
	}

	/* Accelerometer interrupt */
	if (!omap_request_gpio(AIRCELL_ACCEL_INTERRUPT)) {
		omap_set_gpio_direction(AIRCELL_ACCEL_INTERRUPT, 1);
	}

	return;
}

