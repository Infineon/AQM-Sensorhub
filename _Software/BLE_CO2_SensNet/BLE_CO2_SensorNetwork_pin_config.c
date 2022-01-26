/*
 * This file is specific to this BLE app only, the default pin configuration
 * for this platform can be found in the platforms/CYW920719Q40EVB_01/wiced_platform_pin_config.c
 * Please note that its mandatory to name this file in the format "BLE_pin_config.c"
 */

#include "wiced_platform.h"

wiced_platform_gpio_t platform_gpio_pins[]=
	{
		[PLATFORM_GPIO_0]	= {WICED_P00, WICED_GPIO},
		[PLATFORM_GPIO_1]	= {WICED_P01, WICED_GPIO},
		[PLATFORM_GPIO_2]	= {WICED_P02, WICED_GPIO},
		[PLATFORM_GPIO_3]	= {WICED_P04, WICED_GPIO},
		[PLATFORM_GPIO_4]	= {WICED_P06, WICED_UART_2_TXD},
		[PLATFORM_GPIO_5]	= {WICED_P07, WICED_UART_2_RXD},
		[PLATFORM_GPIO_6]	= {WICED_P10, WICED_GPIO},
		[PLATFORM_GPIO_7]	= {WICED_P16, WICED_GPIO},
		[PLATFORM_GPIO_8]	= {WICED_P17, WICED_GPIO},
		[PLATFORM_GPIO_9]	= {WICED_P25, WICED_I2C_1_SDA},
		[PLATFORM_GPIO_10]	= {WICED_P26, WICED_I2C_1_SCL},
		[PLATFORM_GPIO_11]	= {WICED_P28, WICED_GPIO},
		[PLATFORM_GPIO_12]	= {WICED_P29, WICED_GPIO},
		[PLATFORM_GPIO_13]	= {WICED_P33, WICED_GPIO},
		[PLATFORM_GPIO_14]	= {WICED_P34, WICED_GPIO},
		[PLATFORM_GPIO_15]	= {WICED_P38, WICED_GPIO},
	};

const wiced_platform_gpio_config_t platform_gpio[WICED_PLATFORM_GPIO_MAX]=
	{
		[WICED_PLATFORM_GPIO_1] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_3].gpio_pin,
				.config			= (GPIO_OUTPUT_DISABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_2] =       /* Currently selected DHT22 Pin */
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_7].gpio_pin,
				.config			= (GPIO_INPUT_ENABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_3] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_8].gpio_pin,
				.config			= (GPIO_OUTPUT_DISABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_4] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_13].gpio_pin,
				.config			= (GPIO_OUTPUT_DISABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_5] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_14].gpio_pin,
				.config			= (GPIO_OUTPUT_DISABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_6] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_15].gpio_pin,
				.config			= (GPIO_OUTPUT_DISABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_7] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_2].gpio_pin,
				.config			= (GPIO_OUTPUT_DISABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_8] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_0].gpio_pin,
				.config			= (GPIO_OUTPUT_DISABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
	};

const size_t gpio_count =  (sizeof(platform_gpio) / sizeof(wiced_platform_gpio_config_t));


const wiced_platform_led_config_t platform_led[WICED_PLATFORM_LED_MAX]=
	{
		[WICED_PLATFORM_LED_1] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_1].gpio_pin,
				.config			= (GPIO_OUTPUT_ENABLE | GPIO_PULL_DOWN),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_LED_2] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_12].gpio_pin,
				.config			= (GPIO_OUTPUT_ENABLE | GPIO_PULL_DOWN),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_LED_3] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_11].gpio_pin,
				.config			= (GPIO_OUTPUT_ENABLE | GPIO_PULL_DOWN),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_LED_4] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_6].gpio_pin,
				.config			= (GPIO_OUTPUT_ENABLE | GPIO_PULL_DOWN),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
	};

const size_t led_count =  (sizeof(platform_led) / sizeof(wiced_platform_led_config_t));


const wiced_platform_button_config_t platform_button[]=
	{
	};

const size_t button_count =  (sizeof(platform_button) / sizeof(wiced_platform_button_config_t));

