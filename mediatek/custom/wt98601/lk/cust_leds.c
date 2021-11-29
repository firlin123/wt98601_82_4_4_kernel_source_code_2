//#include <platform/cust_leds.h>
#include <cust_leds.h>
#include <platform/mt_pwm.h>

//extern int DISP_SetBacklight(int level);
#ifdef WT_TORCH_CUST_LED_FLASHLIGHT
#include <cust_gpio_usage.h>
#if(MTK_MT6333_SUPPORT)
	extern int flashlight_led_set_MT6333(char *led_name, int level);
#endif
#ifdef GPIO_CAMERA_FLASH_MODE_PIN
	extern int flashlight_led_set_GPIO(char *led_name, int level);
#endif
#endif

extern int disp_bls_set_backlight(unsigned int level);

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64 
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256 

// Custom can decide the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT"
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1,{0,0,0,0,0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0,0,0,0,0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0,0,0,0,0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0,0,0,0,0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0,0,0,0,0}},
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1,{0,0,0,0,0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0,0,0,0,0}},
#ifdef WT_TORCH_CUST_LED_FLASHLIGHT
	{"flashlight",        MT65XX_LED_MODE_CUST_FLASHLIGHT, (int)flashlight_led_set_GPIO,{0}},
#endif
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

