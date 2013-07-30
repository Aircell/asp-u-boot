/* Aircell CloudSurfer GPIO Pins */
#define AIRCELL_5VA_ENABLE				134
#define AIRCELL_5VD_ENABLE				150
#define AIRCELL_18V_ENABLE				149
#define AIRCELL_SOFTWARE_RESET			156
#define AIRCELL_BATTERY_POWERED			94
#define AIRCELL_LCD_RESET				111
#define AIRCELL_POWER_APPLIED_DETECT	105
#define AIRCELL_LED_ENABLE				186
#define AIRCELL_EARPIECE_ENABLE			163
#define AIRCELL_RINGER_ENABLE			129
#define AIRCELL_VOLUME_UP_DETECT		107
#define AIRCELL_VOLUME_DOWN_DETECT		108
#define AIRCELL_HEADSET_DETECT			106
#define AIRCELL_TOUCH_RESET				61
#define AIRCELL_BATTERY_CUT_ENABLE		159
#define AIRCELL_PROX_INTERRUPT			98
#define AIRCELL_ACCEL_INTERRUPT			31
#define AIRCELL_WAKE_ON_LAN				10
#define AIRCELL_TOUCH_INTERRUPT			11
#define AIRCELL_BACKLIGHT_ENABLE		164
#define AIRCELL_MUTE					57

extern unsigned int pin_get_gpio_input(unsigned int pin);
extern unsigned int pin_set_gpio_dataout(unsigned int pin, unsigned int set);
extern unsigned int pin_init_gpio(unsigned int pin_num, unsigned int in_out);
extern void gpio_oe_force_all_input(void);
// Turn on VAUX1 voltage for Product ID
extern void init_vaux1_voltage(void);
extern void cloudsurfer_gpios(void);
