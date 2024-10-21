#include "stm32f1xx.h"
#include "stdio.h"

enum class usart_SR
{
    PE                                   = (0b00 ),//Parity error
    FE                                   = (0b00 ),//Framing error
    NE                                   = (0b00 ),//Noise error flag
    ORE                                  = (0b00 ),//Overrun error
    IDLE                                 = (1<<2 ),//IDLE line detected
    RXNE                                 = (0<<0 ),//Read data register not empty
    TC                                   = (1<<6 ),//Transmission complete
    TXE                                  = (0<<0 ),//Transmit data register empty
    LBD                                  = (0<<0 ),//LIN break detection flag
    CTS                                  = (1<<11),//CTS flag
};

enum class usart_CR1
{
    SBK                                   = (0b00 ),//Send break
    RWU                                   = (0b01 ),//Receiver wakeup
    RE                                    = (0b10 ),//Receiver enable
    TE                                    = (0b11 ),//Transmitter enable
    IDLEIE                                = (1<<2 ),//IDLE interrupt enable
    RXNEIE                                = (0<<0 ),//RXNE interrupt enable
    TCIE                                  = (1<<6 ),//Transmission complete interrupt enable
    TXEIE                                 = (0<<0 ),//TXE interrupt enable
    PEIE                                  = (0<<0 ),//PE interrupt enable
    PS                                    = (1<<11),//Parity selection
    PCE                                   = (1<<11),//Parity control enable
    WAKE                                  = (1<<11),//Wakeup method
    M                                     = (1<<11),//Word length
    UE                                    = (1<<11),//USART enable
};

enum class usart_CR2
{
    ADD_0                                 = (0b00 ),//Send break
    ADD_1                                 = (0b01 ),//Receiver wakeup
    ADD_2                                 = (0b10 ),//Receiver enable
    ADD_3                                 = (0b11 ),//Transmitter enable
    IDLEIE                                = (1<<2 ),//IDLE interrupt enable
    RXNEIE                                = (0<<0 ),//RXNE interrupt enable
    TCIE                                  = (1<<6 ),//Transmission complete interrupt enable
    TXEIE                                 = (0<<0 ),//TXE interrupt enable
    PEIE                                  = (0<<0 ),//PE interrupt enable
    PS                                    = (1<<11),//Parity selection
    PCE                                   = (1<<11),//Parity control enable
    WAKE                                  = (1<<11),//Wakeup method
    M                                     = (1<<11),//Word length
    UE                                    = (1<<11),//USART enable
};

class usart
{
private:

public:
void uart_tx_byte(uint16_t data);
void uart_tx_bytes(const char * data);
void uart_enter(void);
void usart_init(void);
};

uint16_t USART_RX_TX_Str (uint8_t* rx_dt);
uint8_t gencrc(uint8_t *data, size_t len);
