#include "gpio.hpp"
#include <stm32f1xx.h>

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

void USB_Interrupts_Config(void) {
  // Включение тактирования для прерываний USB
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Настройка приоритета и включение прерываний для USB
  NVIC->IP[USB_LP_CAN1_RX0_IRQn] = 1 << 4; // Установка приоритета
  NVIC->ISER[USB_LP_CAN1_RX0_IRQn >> 5] =
      1 << (USB_LP_CAN1_RX0_IRQn & 0x1F); // Включение прерывания
}

void SystemClock_Config(void) {
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
  RCC->CFGR |=
      RCC_CFGR_PLLMULL9; // Умножение PLL на 9 (8 MHz HSE -> 72 MHz PLL)

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

const uint8_t DeviceDescriptor[18] = {
    // Little Endian.
    0x12, // Descriptor lenght = 18 byte.
    0x01, // Descriptor type = Device.
    0x00, // Desctiptor type USB 2.0.
    0x02, // Desctiptor type USB 2.0.
    0x02, // Class Communications and CDC Control.
    0x02, // Subclass Abstract Control Model.
    0x00, // Protocol No class specific protocol required.
    0x40, // EP0 Max Packet Size 64 byte.
    0x83, // Vendor ID 0483.
    0x04, // Vendor ID 0483.
    0x40, // Product ID 5740.
    0x57, // Product ID 5740.
    0x00, // bcdDevice 200.
    0x02, // bcdDevice 200.
    0x00, // iManufacter.
    0x00, // iProduct.
    0x00, // iSerialNumber.
    0x01  // Number of possible configurations = 1.
};

// Определение USB-дескрипторов
// const uint8_t DeviceDescriptor[] = {
//     0x12,       // Длина дескриптора
//     0x01,       // Тип дескриптора (Device)
//     0x00, 0x02, // USB версия 2.0
//     0x02,       // Класс устройства (CDC)//  0x02,
//     0x02,       // Подкласс устройства
//     0x00,       // Протокол устройства
//     0x40,       // Максимальный размер пакета для EP0
//     0x83, 0x04, // Идентификатор производителя (VID)
//     0x11, 0x57, // Идентификатор продукта (PID) // 0x40, 0x57, //
//     Идентификатор продукта (PID) 0x00, 0x02, // Версия устройства 0x01, //
//     Индекс производителя 0x02,       // Индекс продукта 0x03,       // Индекс
//     серийного номера 0x01        // Количество конфигураций
// };

const uint8_t ConfigDescriptor[67] = {
    // Little Endian.
    0x09, // Descriptor lenght = 9 byte.
    0x02, // Descriptor Type = Configuration.
    0x43, // Total lenght of this configuration descriptor (included
          // Interface_Descriptor end Endpoint_Descriptor) = 67.
    0x00, // Total lenght of this configuration descriptor (included
          // Interface_Descriptor end Endpoint_Descriptor) = 67.
    0x02, // Number of interfaces that belong to this configuration = 2.
    0x01, // Index of this configuration = 1.
    0x00, // iConfiguration.
    0xC0, // Self/Powered.
    0x32, // Maximum current consumption = 100 mA.
    /////////////Interface_Descriptor///////////
    0x09, // Descriptor lenght = 9 byte.
    0x04, // Descriptor Type = Interface.
    0x00, // Interface number = 0.
    0x00, // Alternative interface = 0.
    0x01, // Endopoints used by this interface = 1.
    0x02, // Class Communications and CDC Control.
    0x02, // Subclass Abstract Control Model.
    0x01, // Protocol AT Commands: V.250 etc.
    0x00, // iInterface.
    /////////////Class/Specific_Descriptor///////////
    0x05, // Descriptor lenght = 5 byte.
    0x24, // Descriptor Type = "CS_INTERFACE".
    0x00, // Descriptor Subtype = "Header".
    0x10, // USB 1.1.
    0x01, // USB 1.1.
    /////////////Class/Specific_Descriptor///////////
    0x05, // Descriptor lenght = 5 byte.
    0x24, // Descriptor Type = "CS_INTERFACE".
    0x01, // Descriptor SubType = "Call Management Functional Descriptor".
    0x00, // bmCapabilities.
    0x01, // Indicates that multiplexed commands are handled via data interface
          // 01.
    /////////////Class/Specific_Descriptor///////////
    0x04, // Descriptor lenght = 5 byte.
    0x24, // Descriptor Type = "CS_INTERFACE".
    0x02, // Descriptor SubType = "Abstract Control Management functional
          // descriptor".
    0x02, // bmCapabilities.
    /////////////Class/Specific_Descriptor///////////
    0x05, // Descriptor lenght = 5 byte.
    0x24, // Descriptor Type = "CS_INTERFACE".
    0x06, // Descriptor SubType = "Union Descriptor Functional Descriptor".
    0x00, // bControlInterface. Interface number of the control.
    0x01, // bSubordinateInterface0. Interface number of the subordinate (Data
          // Class) interface.
    /////////////Endpoint_Descriptor///////////
    0x07, // Descriptor lenght = 7 byte.
    0x05, // Descriptor Type = "Endpoint".
    0x82, // In endpoint. Endpoint 2.
    0x03, // Transfer Type = "Interrupt".
    0x08, // Endpoint size = 8 byte.
    0x00, // Endpoint size = 8 byte.
    0x10, // Interval for polling endpoint = 16 * 1ms.
    /////////////Interface_Descriptor///////////
    0x09, // Descriptor lenght = 9 byte.
    0x04, // Descriptor Type = Interface.
    0x01, // Interface number = 1.
    0x00, // Alternative interface = 0.
    0x02, // Endopoints used by this interface = 2.
    0x0A, // Class "Data Interface".
    0x00, // Subclass.
    0x00, // Protocol "Non specified".
    0x00, // iInterface.
    /////////////Endpoint_Descriptor///////////
    0x07, // Descriptor lenght = 7 byte.
    0x05, // Descriptor Type = "Endpoint".
    0x01, // OUT endpoint. Endpoint 1.
    0x02, // Transfer Type = "Bulk".
    0x40, // Endpoint size = 64 byte.
    0x00, // Endpoint size = 64 byte.
    0x00, // Interval for polling endpoint.
    /////////////Endpoint_Descriptor///////////
    0x07, // Descriptor lenght = 7 byte.
    0x05, // Descriptor Type = "Endpoint".
    0x81, // In endpoint. Endpoint 1.
    0x02, // Transfer Type = "Bulk".
    0x40, // Endpoint size = 64 byte.
    0x00, // Endpoint size = 64 byte.
    0x00  // Interval for polling endpoint.
};

// const uint8_t ConfigDescriptor[] = {
//     0x09,       // Длина дескриптора
//     0x02,       // Тип дескриптора (Configuration)
//     0x43, 0x00, // Общий размер дескриптора
//     0x01,       // Количество интерфейсов//0x02
//     0x01,       // Номер конфигурации
//     0x00,       // Индекс строки конфигурации
//     0xC0,       // Атрибуты
//     0x32,       // Потребляемый ток (100 мА)

//     // Интерфейсный дескриптор для CDC
//     0x09, // Длина дескриптора
//     0x04, // Тип дескриптора (Interface)
//     0x00, // Номер интерфейса
//     0x00, // Альтернативный номер интерфейса
//     0x01, // Количество конечных точек
//     0x03, // Класс интерфейса (CDC)
//     0x00, // Подкласс интерфейса (ACM)
//     0x00, // Протокол интерфейса (AT commands)
//     0x00, // Индекс строки интерфейса

//     // Дескриптор конечной точки для CDC
//     0x07,       // Длина дескриптора
//     0x05,       // Тип дескриптора (Endpoint)
//     0x81,       // Адрес конечной точки (IN)
//     0x03,       // Атрибуты конечной точки (Interrupt)
//     0x08, 0x00, // Максимальный размер пакета
//     0x20        // Интервал опроса
// };

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
  // Включение тактирования для USBбSYSCFG и GPIOA
  // RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;
  // RCC->APB1ENR |= RCC_APB1ENR_SYSCFGEN;

  // Настройка USB D+ (PA12) для управления pull-up резистором
  // GPIOA->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11 | GPIO_CRH_MODE12 |
  // GPIO_CRH_CNF12); GPIOA->CRH |= GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_0; //
  // PA11 как вход с pull-up GPIOA->CRH |= GPIO_CRH_CNF12_1 | GPIO_CRH_MODE12_0;

  // Настройка NVIC для USB
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);

  // Инициализация USB
  USB->CNTR = USB_CNTR_FRES;
  while (USB->CNTR & USB_CNTR_PDWN)
    //
    // Сброс USB
    USB->CNTR = 0; // Освобождение от сброса
  USB->ISTR = 0; // Очистка регистра состояния прерываний
  USB->BTABLE = 0; // Таблица дескрипторов (настройка BTABLE)
  USB->DADDR =
      USB_DADDR_EF; // USB_DADDR_EF; // Включение USB (EF: Enable Function)
  USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM |
              USB_CNTR_WKUPM; //| USB_CNTR_SUSPM | USB_CNTR_WKUPM; // Включение
                              //прерываний
}

void USB_SetupHandler(void) {
  // Обработка SETUP-пакета
  uint8_t bmRequestType = USB->EP0R & 0xFF;
  uint8_t bRequest = (USB->EP0R >> 8) & 0xFF;
  uint16_t wValue = (USB->EP0R >> 16) & 0xFFFF;
  uint16_t wIndex = (USB->EP0R >> 32) & 0xFFFF;
  uint16_t wLength = (USB->EP0R >> 48) & 0xFFFF;

  // Обработка конкретных запросов
  if (bmRequestType == 0x80 && bRequest == 0x06) {
    // GET_DESCRIPTOR
    if ((wValue >> 8) == 0x01) {
      // Дескриптор устройства
      USB_SendData((uint8_t *)DeviceDescriptor, sizeof(DeviceDescriptor));
    } else if ((wValue >> 8) == 0x02) {
      // Дескриптор конфигурации
      USB_SendData((uint8_t *)ConfigDescriptor, sizeof(ConfigDescriptor));
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
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_R),gpio::gpio_lvl::Low);
  stm32gpio.Set_pin_lvl(GPIOA, static_cast<uint8_t>(gpio_leds::ST_KRK_ST_G),gpio::gpio_lvl::Low);
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
  USB_Interrupts_Config();
  USB_Config();
  // USB_SetupHandler();

  while (1) {
    // if (USB->DADDR & USB_DADDR_EF) {
    //       // USB включен, можно выполнять обмен данными

    //   }
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
    USB->ISTR &= ~USB_ISTR_CTR;

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
  }

  if (USB->ISTR & USB_ISTR_RESET) {
    // Переинициализируем регистры
    USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM;
    USB->ISTR = 0;
    // Создаем 0 конечную точку, типа CONTROL
    USB_SetupHandler();
    // Обнуляем адрес устройства
    USB->DADDR = USB_DADDR_EF;
  }
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
