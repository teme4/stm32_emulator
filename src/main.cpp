#include <stm32f1xx.h>
#include "gpio.hpp"

// Размеры буферов
#define USB_BUFFER_SIZE 64
uint8_t USB_Tx_Buffer[USB_BUFFER_SIZE];
uint8_t USB_Rx_Buffer[USB_BUFFER_SIZE];

// Прототипы функций
void USB_SetupHandler(void);
void USB_ControlOutHandler(void);
void USB_ControlInHandler(void);
void USB_SendData(uint8_t *data, uint16_t length);
void USB_Config(void);
void SystemClock_Config(void);
void USB_Interrupts_Config(void);

void USB_Interrupts_Config(void)
{
  // Включение тактирования для прерываний USB
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Настройка приоритета и включение прерываний для USB
  NVIC->IP[USB_LP_CAN1_RX0_IRQn] = 1 << 4;                                    // Установка приоритета
  NVIC->ISER[USB_LP_CAN1_RX0_IRQn >> 5] = 1 << (USB_LP_CAN1_RX0_IRQn & 0x1F); // Включение прерывания
}

void SystemClock_Config(void)
{
  // Включение HSE (High-Speed External) источника тактового сигнала
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;

  // Настройка Flash задержки
  FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  // Настройка делителей
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

  // Настройка PLL
  RCC->CFGR |= RCC_CFGR_PLLSRC;             // Источник PLL - HSE
  RCC->CFGR &= ~RCC_CFGR_PLLXTPRE_HSE_DIV2; // Предделитель HSE
  RCC->CFGR |= RCC_CFGR_PLLMULL9;           // Умножение PLL на 9 (8 MHz HSE -> 72 MHz PLL)

  // Включение PLL
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  // Выбор PLL в качестве системного тактового сигнала
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;

  // Настройка тактовой частоты USB на 48 МГц
  RCC->CFGR &= ~RCC_CFGR_USBPRE;
}

// Определение USB-дескрипторов
const uint8_t DeviceDescriptor[] = {
    0x12,       // Длина дескриптора
    0x01,       // Тип дескриптора (Device)
    0x00, 0x02, // USB версия 2.0
    0x00,       // Класс устройства (CDC)//  0x02,
    0x00,       // Подкласс устройства
    0x00,       // Протокол устройства
    0x40,       // Максимальный размер пакета для EP0
    0x83, 0x04, // Идентификатор производителя (VID)
    0x11, 0x57, // Идентификатор продукта (PID) // 0x40, 0x57, // Идентификатор продукта (PID)
    0x00, 0x02, // Версия устройства
    0x01,       // Индекс производителя
    0x02,       // Индекс продукта
    0x03,       // Индекс серийного номера
    0x01        // Количество конфигураций
};

const uint8_t ConfigDescriptor[] = {
    0x09,       // Длина дескриптора
    0x02,       // Тип дескриптора (Configuration)
    0x43, 0x00, // Общий размер дескриптора
    0x02,       // Количество интерфейсов
    0x01,       // Номер конфигурации
    0x00,       // Индекс строки конфигурации
    0xC0,       // Атрибуты
    0x32,       // Потребляемый ток (100 мА)

    // Интерфейсный дескриптор для CDC
    0x09, // Длина дескриптора
    0x04, // Тип дескриптора (Interface)
    0x00, // Номер интерфейса
    0x00, // Альтернативный номер интерфейса
    0x01, // Количество конечных точек
    0x02, // Класс интерфейса (CDC)
    0x02, // Подкласс интерфейса (ACM)
    0x01, // Протокол интерфейса (AT commands)
    0x00, // Индекс строки интерфейса

    // Дескриптор конечной точки для CDC
    0x07,       // Длина дескриптора
    0x05,       // Тип дескриптора (Endpoint)
    0x81,       // Адрес конечной точки (IN)
    0x03,       // Атрибуты конечной точки (Interrupt)
    0x08, 0x00, // Максимальный размер пакета
    0xFF        // Интервал опроса
};

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

void USB_Config(void)
{
  // Включение тактирования для USB и GPIOA
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;

  // Настройка USB D+ (PA12) для управления pull-up резистором
  GPIOA->CRH &= ~GPIO_CRH_MODE12;
  GPIOA->CRH |= GPIO_CRH_MODE12_0;

  // Настройка NVIC для USB
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);

  // Инициализация USB
  USB->CNTR = USB_CNTR_FRES;                                                     // Сброс USB
  USB->CNTR = 0;                                                                 // Освобождение от сброса
  USB->ISTR = 0;                                                                 // Очистка регистра состояния прерываний
  USB->BTABLE = 0;                                                               // Таблица дескрипторов (настройка BTABLE)
  USB->DADDR = USB_DADDR_EF;                                                     // Включение USB (EF: Enable Function)
  USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM; // Включение прерываний
}

void USB_SetupHandler(void)
{
  // Обработка SETUP-пакета
  uint8_t bmRequestType = USB->EP0R & 0xFF;
  uint8_t bRequest = (USB->EP0R >> 8) & 0xFF;
  uint16_t wValue = (USB->EP0R >> 16) & 0xFFFF;
  uint16_t wIndex = (USB->EP0R >> 32) & 0xFFFF;
  uint16_t wLength = (USB->EP0R >> 48) & 0xFFFF;

  // Обработка конкретных запросов
  if (bmRequestType == 0x80 && bRequest == 0x06)
  {
    // GET_DESCRIPTOR
    if ((wValue >> 8) == 0x01)
    {
      // Дескриптор устройства
      USB_SendData((uint8_t *)DeviceDescriptor, sizeof(DeviceDescriptor));
    }
    else if ((wValue >> 8) == 0x02)
    {
      // Дескриптор конфигурации
      USB_SendData((uint8_t *)ConfigDescriptor, sizeof(ConfigDescriptor));
    }
  }
  else if (bmRequestType == 0x00 && bRequest == 0x05)
  {
    // SET_ADDRESS
    USB->DADDR = (wValue & 0x7F) | USB_DADDR_EF;
  }
}

void USB_ControlOutHandler(void)
{
  // Получение адреса таблицы дескрипторов
  uint32_t *pma_addr = (uint32_t *)(USB_PMAADDR + (USB->EP0R & USB_EP_DTOG_RX));

  // Получение размера пакета из таблицы дескрипторов
  uint16_t count = pma_addr[0] & 0x3FF;

  // Чтение данных из PMA
  for (uint16_t i = 0; i < count; i++)
  {
    USB_Rx_Buffer[i] = *(uint8_t *)(USB_PMAADDR + pma_addr[1] + i);
  }

  // Например, сохранение данных в буфер
}

void USB_ControlInHandler(void)
{
  // Обработка данных для endpoint 0 IN
  uint32_t *pma_addr = (uint32_t *)(USB_PMAADDR + (USB->EP0R & USB_EP_DTOG_TX));
  for (uint16_t i = 0; i < USB_BUFFER_SIZE; i++)
  {
    *(uint8_t *)(USB_PMAADDR + pma_addr[1] + i) = USB_Tx_Buffer[i];
  }
  USB->EP0R = USB_EP_CTR_TX; //| USB_EP_EP_TYPE_CONTROL;
}

void USB_SendData(uint8_t *data, uint16_t length)
{
  for (uint16_t i = 0; i < length; i++)
  {
    USB_Tx_Buffer[i] = data[i];
  }
}

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
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_R), gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_G), gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_R), gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::KI_KRK_G), gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_R), gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_TST_G), gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_R), gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::SUP_KRK_TST_G), gpio::gpio_lvl::Low);
  // LEds_off_eth
  stm32gpio.Set_pin_lvl(GPIOC, static_cast<uint8_t>(gpio_leds::TX_1), gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::TX_2), gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::TX_3), gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOC, static_cast<uint8_t>(gpio_leds::LINK_1_G), gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_2_G), gpio::gpio_lvl::Hight);
  stm32gpio.Set_pin_lvl(GPIOB, static_cast<uint8_t>(gpio_leds::LINK_3_G), gpio::gpio_lvl::Hight);
}
uint32_t k = 0;
int main()
{
  gpio_init();
  ALL_leds_off();
  SystemClock_Config();
  USB_Interrupts_Config();
  USB_Config();

  while (1)
  {
    // if (USB->DADDR & USB_DADDR_EF) {
    //       // USB включен, можно выполнять обмен данными

    //   }
    k++;
    if (k > 5000)
    {
      ALL_leds_off();
      k = 0;
    }
  }
}

extern "C"
{
  void USB_LP_CAN1_RX0_IRQHandler(void)
  {
    if (USB->ISTR & USB_ISTR_CTR)
    {
      uint16_t endpoint = USB->ISTR & USB_ISTR_EP_ID;
      USB->ISTR = ~USB_ISTR_CTR;

      if (endpoint == 0)
      {
        if (USB->EP0R & USB_EP_CTR_RX)
        {
          if (USB->EP0R & USB_EP_SETUP)
          {
            USB_SetupHandler();
          }
          else
          {
            USB_ControlOutHandler();
          }
          USB->EP0R &= ~USB_EP_CTR_RX; // Clear CTR_RX flag
        }

        if (USB->EP0R & USB_EP_CTR_TX)
        {
          USB_ControlInHandler();
          USB->EP0R &= ~USB_EP_CTR_TX; // Clear CTR_TX flag
        }
      }
    }
  }
}

extern "C"
{
  void HardFault_Handler(void)
  {
    int l = 0;
    while (1)
    {
      l++;
    }
  }
}
