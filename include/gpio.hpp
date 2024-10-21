class gpio
{
private:
  static void Set_pin_H(GPIO_TypeDef *port, uint8_t pin);
  static void Set_pin_L(GPIO_TypeDef *port, uint8_t pin);

public:
  enum struct gpio_mode // MODERN
  {
    //            General purpose output
    gpio_mode_pp_2 = 0x01,
    gpio_mode_pp_10 = 0x02,
    gpio_mode_pp_50 = 0x03,
    gpio_mode_od_2 = 0x05,
    gpio_mode_od_10 = 0x06,
    gpio_mode_od_50 = 0x07,
    //             Alternate Function output
    alternate_mode_pp_2 = 0x09,
    alternate_mode_pp_10 = 0x0A,
    alternate_mode_pp_50 = 0x0B,
    alternate_mode_od_2 = 0x0D,
    alternate_mode_od_10 = 0x0E,
    alternate_mode_od_50 = 0x0F,
    //                Input
    input_mode_analog = 0x00,
    input_mode_floating = 0x04,
    input_mode_pull_down = 0x28,
    input_mode_pull_up = 0x48 // 0x80
  };
  static void gpio_init(GPIO_TypeDef *port, uint8_t pin, gpio_mode mode);
  static void set_pin_state(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t state);
  static int get_state_pin(GPIO_TypeDef *GPIOx, uint8_t pin);
  //  void config_af(GPIO_TypeDef *GPIOx, uint8_t PIN, uint8_t AF);
};
