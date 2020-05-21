/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* This library is based on this library: 
*   https://github.com/aaronds/arduino-nrf24l01
* Which is based on this library: 
*   http://www.tinkerer.eu/AVRLib/nRF24L01
* -----------------------------------------------------------------------------
* Портировано под stm32f103 и libopencm3
*/
#ifndef NRF24
#define NRF24
#include <stdint.h>
/********************************************************************
Конфигурация выводов
*********************************************************************/
#define NRF_RCC_SPI RCC_SPI1
#define NRF_SPI SPI1
#define NRF_SPI_MOSI GPIO_SPI1_MOSI
#define NRF_SPI_MISO GPIO_SPI1_MISO
#define NRF_SPI_SCK GPIO_SPI1_SCK  

#define NRF_PORT GPIOA
#define NRF_CE GPIO4
#define NRF_CSN GPIO3

#define NRF_CE_LO() gpio_clear(NRF_PORT,NRF_CE)
#define NRF_CE_HI() gpio_set(NRF_PORT,NRF_CE)
#define NRF_CSN_LO() gpio_clear(NRF_PORT,NRF_CSN)
#define NRF_CSN_HI() gpio_set(NRF_PORT,NRF_CSN)
#define NRF_SPI_TRANSFER(data) spi_xfer(NRF_SPI, (data))
#define NRF_WSPI() while(SPI_SR(NRF_SPI) & SPI_SR_BSY)

#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 4
#define NRF_ADDR_LEN 3

static uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb3,0xb4,0x01};
//uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};


//#define nrf24_CONFIG ((1<<EN_CRC)|(0<<CRCO))

//#define NRF24_TRANSMISSON_OK 0
//#define NRF24_MESSAGE_LOST   1

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE 0x1D

//------------------------------------------------
//#define PRIM_RX 0x00 //RX/TX control (1: PRX, 0: PTX)
//#define PWR_UP 0x01 //1: POWER UP, 0:POWER DOWN
//#define RX_DR 0x40 //Data Ready RX FIFO interrupt
//#define TX_DS 0x20 //Data Sent TX FIFO interrupt
//#define MAX_RT 0x10 //Maximum number of TX retransmits interrupt
//------------------------------------------------


/* Bit Mnemonics */

/* configuratio nregister */
#define MASK_RX_DR  1<<6
#define MASK_TX_DS  1<<5
#define MASK_MAX_RT 1<<4
#define EN_CRC      1<<3
#define CRCO        1<<2
#define PWR_UP      1<<1
#define PRIM_RX     1

/* enable auto acknowledgment */
#define ENAA_P5     1<<5
#define ENAA_P4     1<<4
#define ENAA_P3     1<<3
#define ENAA_P2     1<<2
#define ENAA_P1     1<<1
#define ENAA_P0     1<<0

/* enable rx addresses */
#define ERX_P5      1<<5
#define ERX_P4      1<<4
#define ERX_P3      1<<3
#define ERX_P2      1<<2
#define ERX_P1      1<<1
#define ERX_P0      1<<0

/* setup of address width */
#define AW          0 /* 2 bits */

/* setup of auto re-transmission */
#define ARD         4 /* 4 bits */
#define ARC         0 /* 4 bits */

/* RF setup register */
#define PLL_LOCK    1<<4
#define RF_DR       1<<3
#define RF_PWR      1 /* 2 bits */   

/* general status register */
#define RX_DR       1<<6
#define TX_DS       1<<5
#define MAX_RT      1<<4
#define RX_P_NO     0b1110 /* 3 bits */
#define TX_FULL     1

/* transmit observe register */
#define PLOS_CNT    4 /* 4 bits */
#define ARC_CNT     0 /* 4 bits */

/* fifo status */
#define TX_REUSE   1<<6
#define FIFO_FULL   1<<5
#define TX_EMPTY    1<<4
#define RX_FULL     1<<1
#define RX_EMPTY    1<<0

/* dynamic length */
#define DPL_P0      0
#define DPL_P1      1
#define DPL_P2      2
#define DPL_P3      3
#define DPL_P4      4
#define DPL_P5      5


/* Instruction Mnemonics */
#define R_REGISTER    0x00 /* last 4 bits will indicate reg. address */
#define W_REGISTER    0x20 /* last 4 bits will indicate reg. address */
#define REGISTER_MASK 0x1F
#define RD_RX_PLOAD   0x61 // Define RX payload register address
#define WR_TX_PLOAD   0xA0 // Define TX payload register address
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define ACTIVATE      0x50 
#define R_RX_PL_WID   0x60
#define ACTIVATE      0x50 
#define NOP           0xFF




#define nrf_rx_address(addr, addr_len) nrf_write(RX_ADDR_P1,(addr),(addr_len))
#define  nrf_rrx_payload_width() nrf_rreg(R_RX_PL_WID);

void exti0_init();
void spi_nrf_init(void);
uint8_t nrf_rreg(uint8_t addr);
void nrf_wreg(uint8_t addr, uint8_t data);
void nrf_read(uint8_t addr,uint8_t *data,uint8_t count);
void nrf_write(uint8_t addr,uint8_t *data,uint8_t count);
void nrf_toggle_features(void);
nrf_flushtx(void);
void nrf_flushrx(void);
void nrf_rx_mode(void);
void nrf_tx_address(uint8_t* addr, uint8_t addr_len);
void delay_us(uint32_t delay);
uint8_t nrf_send(uint8_t *data,uint8_t len);
void nrf_write_bufer(uint8_t addr,uint8_t *data,uint8_t count);
uint8_t nrf_status();
void nrf_init(void);
//новая порция отладочных функций
void nrf_print_report();
void nrf_report_reg_str(uint8_t reg);
void nrf_report_addr_str(uint8_t addr);
uint8_t nrf_regname2int(char *str);
void nrf_get_cmd(char *str);
void nrf_recive();


#endif

