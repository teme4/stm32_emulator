#include "USB_DeviceDescriptor.hpp"
#include "gpio.hpp"
#include <stdlib.h>
#include <stm32f1xx.h>

// Размеры буферов
#define USB_BUFFER_SIZE 64
uint8_t USB_Tx_Buffer[USB_BUFFER_SIZE];
uint8_t USB_Rx_Buffer[USB_BUFFER_SIZE];
volatile uint16_t USB_err = 0;
// Прототипы функций
void USB_SetupHandler(void);
void USB_ControlOutHandler(void);
void USB_ControlInHandler(void);
void USB_SendData(uint8_t *data, uint16_t length);
void USB_Config(void);
void SystemClock_Config(void);
void USB_Interrupts_Config(void);
void USB_Reset(void);

void USB_Interrupts_Config(void) {
  // Настройка группового приоритета прерываний
  SCB->AIRCR =
      (0x5FA << SCB_AIRCR_VECTKEY_Pos) | (1 << 8); // NVIC_PriorityGroup_1
  // Установка приоритета прерывания для USB_LP_CAN1_RX0_IRQn
  NVIC->IP[USB_LP_CAN1_RX0_IRQn] = 1 << 4; // Приоритет 1
  // Включение прерывания USB_LP_CAN1_RX0_IRQn
  NVIC->ISER[USB_LP_CAN1_RX0_IRQn >> 5] = (uint32_t)1
                                          << (USB_LP_CAN1_RX0_IRQn & 0x1F);
}
void USB_Reset(void) {
  // Сброс регистров USB
  USB->CNTR = USB_CNTR_FRES;
  USB->ISTR = 0;
  USB->BTABLE = 0;
  USB->DADDR = 0;

  // Ожидание завершения сброса
  for (volatile uint32_t i = 0; i < 1000; i++)
    ;

  // Освобождение от сброса
  USB->CNTR = 0;
  USB->DADDR = USB_DADDR_EF; // Включение USB (EF: Enable Function)

  // Настройка регистров контрольных точек (пример для EP0 IN и OUT)
  USB->EP0R = USB_EP_CONTROL;
  USB->EP0R &= ~USB_EP_KIND;
  USB->EP0R &= ~USB_EP_DTOG_RX;
  USB->EP0R &= ~USB_EP_DTOG_TX;
  USB->EP0R |= USB_EP_CTR_RX;
  USB->EP0R |= USB_EP_CTR_TX;

  // Разрешение прерываний
  USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SOFM;
}

// void USB_Interrupts_Config(void)
//  {
//   // Включение тактирования для прерываний USB
//   RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
//   // Настройка приоритета и включение прерываний для USB
//   NVIC->IP[USB_LP_CAN1_RX0_IRQn] = 1 << 4; // Установка приоритета
//   NVIC->ISER[USB_LP_CAN1_RX0_IRQn >> 5] =
//       1 << (USB_LP_CAN1_RX0_IRQn & 0x1F); // Включение прерывания
// }

void SystemClock_Config(void) {
  // Enable HSE (High-Speed External) oscillator
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;

  // Enable Prefetch Buffer and set Flash Latency
  FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  // Set HCLK, PCLK1, and PCLK2
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

  // Configure PLL (HSE as PLL source, PLL multiplier = 9, 8 MHz * 9 = 72 MHz)
  RCC->CFGR |= RCC_CFGR_PLLSRC;
  RCC->CFGR &= ~RCC_CFGR_PLLXTPRE_HSE_DIV2;
  RCC->CFGR |= RCC_CFGR_PLLMULL9;

  // Enable PLL
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  // Select PLL as system clock source
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;

  // Set USB prescaler to get 48 MHz
  RCC->CFGR &= ~RCC_CFGR_USBPRE;
}

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

void USB_Config(void) {
  // Включаем тактирование SYSCFG
  // RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Включаем тактирование USB
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;

  // Сбрасываем регистры USB
  USB->CNTR = USB_CNTR_FRES;
  USB->ISTR = 0;
  USB->BTABLE = 0;
  USB->DADDR = 0;

  // Ожидание завершения сброса
  for (volatile uint32_t i = 0; i < 1000; i++)
    ;

  // Освобождение от сброса
  USB->CNTR = 0;
  USB->DADDR = USB_DADDR_EF; // Включение USB (EF: Enable Function)

  // Настройка регистров контрольных точек (пример для EP0 IN и OUT)
  USB->EP0R = USB_EP_CONTROL;
  USB->EP0R &= ~USB_EP_KIND;
  USB->EP0R &= ~USB_EP_DTOG_RX;
  USB->EP0R &= ~USB_EP_DTOG_TX;
  USB->EP0R |= USB_EP_CTR_RX;
  USB->EP0R |= USB_EP_CTR_TX;

  // Разрешение прерываний
  USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SOFM;

  // Настройка NVIC для USB
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);
}

#define USB_DESC_TYPE_DEVICE 0x01
#define USB_DESC_TYPE_CONFIGURATION 0x02
#define USB_DESC_TYPE_STRING 0x03
#define USB_DESC_TYPE_INTERFACE 0x04
#define USB_DESC_TYPE_ENDPOINT 0x05

void USB_SetupHandler(void) {
  uint8_t bmRequestType = USB->EP0R & 0xFF;
  uint8_t bRequest = (USB->EP0R >> 8) & 0xFF;
  uint16_t wValue = (USB->EP0R >> 16) & 0xFFFF;
  uint16_t wIndex = (USB->EP0R >> 32) & 0xFFFF;
  uint16_t wLength = (USB->EP0R >> 48) & 0xFFFF;

  if (bmRequestType == 0x80 && bRequest == 0x06) {
    // GET_DESCRIPTOR
    uint8_t desc_type = (wValue >> 8) & 0xFF;
    uint8_t desc_index = wValue & 0xFF;
    if (desc_type == USB_DESC_TYPE_DEVICE) {
      USB_SendData((uint8_t *)DeviceDescriptor, sizeof(DeviceDescriptor));
    } else if (desc_type == USB_DESC_TYPE_CONFIGURATION) {
      USB_SendData((uint8_t *)ConfigurationDescriptor,
                   sizeof(ConfigurationDescriptor));
    } else if (desc_type == USB_DESC_TYPE_DEVICE) {
      if (desc_index == 0) {
        USB_SendData((uint8_t *)StringDescriptor, sizeof(StringDescriptor));
      } else if (desc_index == 1) {
        USB_SendData((uint8_t *)StringDescriptorManufacturer,
                     sizeof(StringDescriptorManufacturer));
      } else if (desc_index == 2) {
        USB_SendData((uint8_t *)StringDescriptorProduct,
                     sizeof(StringDescriptorProduct));
      } else if (desc_index == 3) {
        USB_SendData((uint8_t *)StringDescriptorSerial,
                     sizeof(StringDescriptorSerial));
      }
    }
  } else if (bmRequestType == 0x00 && bRequest == 0x05) {
    // SET_ADDRESS
    USB->DADDR = (wValue & 0x7F) | USB_DADDR_EF;
  }
}

void USB_ControlOutHandler(void) {
  // Получение адреса таблицы дескрипторов
  uint32_t *pma_addr = (uint32_t *)(USB_PMAADDR + (USB->EP0R & USB_EP_DTOG_RX));

  // Получение размера пакета из таблицы дескрипторов
  uint16_t count = pma_addr[0] & 0x3FF;

  // Чтение данных из PMA
  for (uint16_t i = 0; i < count; i++) {
    USB_Rx_Buffer[i] = *(uint8_t *)(USB_PMAADDR + pma_addr[1] + i);
  }

  // Например, сохранение данных в буфер
}

void USB_ControlInHandler(void) {
  // Обработка данных для endpoint 0 IN
  uint32_t *pma_addr = (uint32_t *)(USB_PMAADDR + (USB->EP0R & USB_EP_DTOG_TX));
  for (uint16_t i = 0; i < USB_BUFFER_SIZE; i++) {
    *(uint8_t *)(USB_PMAADDR + pma_addr[1] + i) = USB_Tx_Buffer[i];
  }
  USB->EP0R = USB_EP_CTR_TX; //| USB_EP_EP_TYPE_CONTROL;
}

void USB_SendData(uint8_t *data, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    USB_Tx_Buffer[i] = data[i];
  }
}

void gpio_init() {
  // SD_Card
  stm32gpio.gpio_init(GPIOD, static_cast<uint8_t>(gpio_SD::SD_CMD),
                      gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_CLK),
                      gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA0),
                      gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA1),
                      gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA2),
                      gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_SD::SD_DATA3),
                      gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_SD::SD_SW_1),
                      gpio::gpio_mode::input_mode_pull_up);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_SD::SD_SW_2),
                      gpio::gpio_mode::input_mode_pull_up);
  // LEds
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_R),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_G),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_R),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_G),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_R),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_G),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_R),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_G),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_leds::TX_1),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOC, static_cast<uint8_t>(gpio_leds::LINK_1_G),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::TX_2),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_2_G),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::TX_3),
                      gpio::gpio_mode::gpio_mode_pp_50);
  stm32gpio.gpio_init(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_3_G),
                      gpio::gpio_mode::gpio_mode_pp_50);
  // USB
  stm32gpio.gpio_init(GPIOA, 11, gpio::gpio_mode::alternate_mode_pp_50);
  stm32gpio.gpio_init(GPIOA, 12, gpio::gpio_mode::alternate_mode_pp_50);
}
void ALL_leds_off() {
  // LEds_off
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_R),
                        gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_G),
                        gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_R),
                        gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_G),
                        gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_R),
                        gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_G),
                        gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_R),
                        gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_G),
                        gpio::gpio_lvl::Low);
  // LEds_off_eth
  stm32gpio.Set_pin_lvl(GPIOC, static_cast<uint8_t>(gpio_leds::TX_1),
                        gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::TX_2),
                        gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::TX_3),
                        gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOC, static_cast<uint8_t>(gpio_leds::LINK_1_G),
                        gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_2_G),
                        gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_3_G),
                        gpio::gpio_lvl::Hight);
}
uint32_t k = 0;
int main() {
  gpio_init();
  ALL_leds_off();
  SystemClock_Config();
  USB_Config();

  while (1) {

    k++;
    if (k > 5000) {
      ALL_leds_off();
      k = 0;
    }
  }
}

extern "C" {
void USB_LP_CAN1_RX0_IRQHandler(void) {

  if (USB->ISTR & USB_ISTR_CTR) {
    uint16_t endpoint = USB->ISTR & USB_ISTR_EP_ID;
    USB->ISTR = ~USB_ISTR_CTR;

    if (endpoint == 0) {
      if (USB->EP0R & USB_EP_CTR_RX) {
        if (USB->EP0R & USB_EP_SETUP) {
          USB_SetupHandler();
        } else {
          USB_ControlOutHandler();
        }
        USB->EP0R &= ~USB_EP_CTR_RX; // Clear CTR_RX flag
      }

      if (USB->EP0R & USB_EP_CTR_TX) {
        USB_ControlInHandler();
        USB->EP0R &= ~USB_EP_CTR_TX; // Clear CTR_TX flag
      }
    }
    USB_err = 1;
    // return;
  }
  if (USB->ISTR & USB_ISTR_PMAOVR) {
    USB->ISTR &= ~USB_ISTR_PMAOVR;
    // Handle PMAOVR status
    USB_err = 2;
    // return;
  }
  if (USB->ISTR & USB_ISTR_SUSP) {
    USB->ISTR &= ~USB_ISTR_SUSP;
    if (USB->DADDR & 0x7f) {
      USB->DADDR = 0;
      USB->CNTR &= ~0x800;
    }
    USB_err = 3;
    // return;
  }
  if (USB->ISTR & USB_ISTR_ERR) {
    USB->ISTR &= ~USB_ISTR_ERR;
    // Handle Error
    USB_err = 4;
    // return;
  }
  if (USB->ISTR & USB_ISTR_WKUP) {
    USB->ISTR &= ~USB_ISTR_WKUP;
    // Handle Wakeup
    USB_err = 5;
    // return;
  }
  if (USB->ISTR & USB_ISTR_SOF) {
    USB->ISTR &= ~USB_ISTR_SOF;
    // Handle SOF
    USB_err = 6;
    // return;
  }
  if (USB->ISTR & USB_ISTR_ESOF) {
    USB->ISTR &= ~USB_ISTR_ESOF;
    // Handle ESOF
    USB_err = 7;
    // return;
  }
  USB->ISTR = 0;
  USB_err = 0;
}
}
extern "C" {
void HardFault_Handler(void) {
  int l = 0;
  while (1) {
    l++;
  }
}
}
