#include <stm32f1xx.h>

//USART1
#define usart_tx_pin						  	9
#define usart_rx_pin						    10

//Parallel_BUS
#define D0							       	 0
#define D1							         1
#define D2							         2
#define D3					                 3
#define D4							       	 4
#define D5							         5
#define D6						             6
#define D7				                     7
#define CS				                     8
#define RS						             11
#define RD							       	 12
#define WR							         15

//595
#define data_pin						     8
#define clock_pin				             11
#define latcg_pin				             12
#define EN_1      				             2

//165
#define data_pin						     8
#define clock_pin				             11
#define CS_595				                 7
#define EN_595     				             12
#define EN_165    				             11
#define pl_165				                 10

//138
#define clk_165						         13 //Clock inut                 D0 12
#define data_km   		                     0   //data
#define data_flex                            14
#define data_db     		                 2 //data

#define SCK_595                              13
#define MISO_595   		                     14
#define MOSI_595    		                 15

void gpio_init(void);
