#include <stm32f1xx.h>
#include "gpio.hpp"

void gpio::Set_pin_H(GPIO_TypeDef *port, uint8_t pin)
{
  port->BSRR = (1 << pin);
}

void gpio::Set_pin_L(GPIO_TypeDef *port, uint8_t pin)
{
  port->BSRR = (1 << pin + 16);
}
// FIXME Добавить проверку пина,что бы не вылететь за диапозона
void gpio::gpio_init(GPIO_TypeDef *port, uint8_t pin, gpio_mode mode)
{
  RCC->APB2ENR |= (1 << (((uint32_t)port - APB2PERIPH_BASE) / 0x400));
  if (pin > 7)
  {
    port->CRH &= ~(0xf << ((pin - 8) * 4));
    port->CRH |= static_cast<uint32_t>(mode) << ((pin - 8) * 4);
  }
  else
  {
    port->CRL &= ~(0xf << (pin * 4));
    port->CRL |= static_cast<uint32_t>(mode) << (pin * 4);
  }
}

void gpio::set_pin_state(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t state)
{
  if (state == 1)
    Set_pin_H(GPIOx, pin);
  if (state == 0)
    Set_pin_L(GPIOx, pin);
}

int gpio::get_state_pin(GPIO_TypeDef *port, uint8_t pin)
{
  if ((port->IDR) & (1 << pin))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
