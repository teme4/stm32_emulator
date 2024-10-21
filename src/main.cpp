#include <stm32f1xx.h>
#include "gpio.hpp"

// макрос адресации
#define REG(x) (*((volatile unsigned int)(x)))

// базовый адрес регистров драйвера
#define USB_BASE_ADDR 0x40005C00
// адрес начала области памяти драйвера USB
#define USB_PMA_ADDR 0x40006000

// регистры состояния конечных точек
#define EP0R REG(USB_BASE_ADDR)
#define EP1R REG(0x40005C04)
#define EP2R REG(0x40005C08)
#define EP3R REG(0x40005C0C)
#define EP4R REG(0x40005C10)
#define EP5R REG(0x40005C14)
#define EP6R REG(0x40005C18)
#define EP7R REG(0x40005C1C)
#define ENDPOINT(bEpNum) REG(USB_BASE_ADDR + (bEpNum) * 4)
#define PMA_BUF(INum) REG(USB_PMA_ADDR + (INum)4)
#define PMA_SBUF(SINum) (((volatile unsigned short int *)(USB_PMA_ADDR + (SINum) * 2)))

// остальные регистры
#define CNTR REG(USB_BASE_ADDR + 0x40)
#define ISTR REG(USB_BASE_ADDR + 0x44)
#define DADDR REG(USB_BASE_ADDR + 0x4C)
#define BTABLE REG(USB_BASE_ADDR + 0x50)

#define HSE_STARTUP_TIMEOUT ((uint32_t)100) /*!< Time out for HSE start up, in ms */
#define RCC_CFGR_PLLSRC_HSE ((uint32_t)0x00010000)
#define VECT_TAB_OFFSET 0x4000U /*!< Vector Table base offset field.This value must be a multiple of 0x200. */

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
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;
  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CFGR &= (uint32_t)0xF8FF0000;
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;
  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;
  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */

  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
  //  RCC->CR |= ((uint32_t)RCC_CR_HSION);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    //    HSEStatus = RCC->CR & RCC_CR_HSIRDY;
    StartUpCounter++;
  } while (((RCC->CR & RCC_CR_HSERDY) == 0) && (StartUpCounter < HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET) //	  if ((RCC->CR & RCC_CR_HSIRDY) != RESET)
  {
    // HSEStatus = (uint32_t)0x01;
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

    /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
    // PLLMULL9 - for 8MHz
    //     RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL9);
    // PLLMULL9 - for 8MHz
    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    /* Wait till PLL is ready */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    } /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
  /* Vector Table Relocation in Internal FLASH. */
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
