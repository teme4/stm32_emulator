#include <stm32f1xx.h>
#include "gpio.hpp"

// Set SYSTEM CLOCK = 72Mhz. AHB CLOCK = 72Mhz. APB1 CLOCK = 36Mhz. APB2 CLOCK = 72Mhz. USB CLOCK = 48Mhz.
// (Alternatively use my function "ClockInit()" in the example code, you will find it's definition in "RCC_CLOCK.h",
//  in folder Core/Inc).

gpio stm32gpio;

enum class gpio_SD : uint8_t // OSPEEDR
{
  SD_CMD = 2,    // PD2
  SD_CLK = 12,   // PC12
  SD_DATA0 = 8,  // PC8
  SD_DATA1 = 9,  // PC9
  SD_DATA2 = 10, // PC10
  SD_DATA3 = 11, // PC11
  SD_SW_1 = 7,   // PB7
  SD_SW_2 = 8    // PB8
};

enum class gpio_leds : uint8_t // OSPEEDR
{
  ST_KRK_ST_R = 0,   // PA0
  ST_KRK_ST_G = 1,   // PA1
  ST_KRK_TST_R = 2,  // PA2
  ST_KRK_TST_G = 3,  // PA3
  SUP_KRK_TST_R = 4, // PA4
  SUP_KRK_TST_G = 5, // PA5
  KI_KRK_R = 6,      // PA6
  KI_KRK_G = 7,      // PA7
  TX_1 = 4,          // PC4
  LINK_1_G = 5,      // PC5
  TX_2 = 0,          // PB0
  LINK_2_G = 1,      // PB1
  TX_3 = 10,         // PB10
  LINK_3_G = 11      // PB11
};

void gpio_init()
{
  // SD_Card
  stm32gpio.gpio_init(GPIOD, static_cast<uint8_t>(gpio_SD::SD_CMD), gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_CLK), gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA0), gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA1), gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA2), gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA3), gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_SD::SD_SW_1), gpio::gpio_mode::input_mode_pull_up);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_SD::SD_SW_2), gpio::gpio_mode::input_mode_pull_up);
  // LEds
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_R), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_G), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_R), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_G), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_R), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_G), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_R), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_G), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_leds::TX_1), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_leds::LINK_1_G), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::TX_2), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_2_G), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::TX_3), gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_3_G), gpio::gpio_mode::gpio_mode_pp_50);
}
void ALL_leds_off()
{
  // LEds_off
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_R), 0);
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_G), 0);
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_R), 1);
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_G), 0);
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_R), 0);
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_G), 0);
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_R), 0);
  stm32gpio.set_pin_state(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_G), 0);
  // LEds_off_eth
  stm32gpio.set_pin_state(GPIOC, static_cast<uint8_t>(gpio_leds::TX_1), 1);
  stm32gpio.set_pin_state(GPIOB, static_cast<uint8_t>(gpio_leds::TX_2), 1);
  stm32gpio.set_pin_state(GPIOB, static_cast<uint8_t>(gpio_leds::TX_3), 1);
  stm32gpio.set_pin_state(GPIOC, static_cast<uint8_t>(gpio_leds::LINK_1_G), 1);
  stm32gpio.set_pin_state(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_2_G), 1);
  stm32gpio.set_pin_state(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_3_G), 1);
}

void SystemInit_1(void)
{
  // Enable clock for GPIOA and USB
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;

  // Configure USB pull-up pin
  GPIOA->CRH &= ~GPIO_CRH_MODE12;
  GPIOA->CRH |= GPIO_CRH_MODE12_0;

  // Initialize USB
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);
  USB->CNTR = USB_CNTR_FRES; // Force reset
  USB->CNTR = 0;             // Release reset
  USB->ISTR = 0;             // Clear interrupt status register
}

// void USB_LP_CAN1_RX0_IRQHandler(void)
// {
//   // Handle USB interrupts here
//   if (USB->ISTR & USB_ISTR_CTR)
//   {
//     // Correct transfer interrupt
//     USB->ISTR = ~USB_ISTR_CTR; // Clear CTR flag
//   }
// }

int main()
{
  // SystemInit_1();
  gpio_init();
  ALL_leds_off();

  while (1)
  {
  }
}

extern "C"
{
  void HardFault_Handler(void)
  {
    int k = 0;
    while (1)
    {
      k++;
    }
  }
}
