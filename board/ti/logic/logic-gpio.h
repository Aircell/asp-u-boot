// GPIO
extern unsigned int pin_get_gpio_input(unsigned int pin);
extern unsigned int pin_set_gpio_dataout(unsigned int pin, unsigned int set);
extern unsigned int pin_init_gpio(unsigned int pin_num, unsigned int in_out);
extern void gpio_oe_force_all_input(void);
// Turn on VAUX1 voltage for Product ID
extern void init_vaux1_voltage(void);
