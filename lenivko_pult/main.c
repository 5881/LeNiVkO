#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
//#include <stdio.h>
#include "printf.h"
#include <string.h>
#include "nrf24.h"

#define PCF_ADDRES 0x20
uint8_t pcf_read(void);
void pcf_write(uint8_t);

uint8_t key=0xff;
uint8_t sleep_mode=0;
uint8_t testmes[5]={1,2,3,4,5};

char nrf_cmd_line[128];
void nrf_flush_cmd_bufer(){
	for(uint8_t i=0;i<128;i++) nrf_cmd_line[i]=0;
	//send_line("\r\n#>");
}

/***********************************************************************
 * Энергосбережение
 ***********************************************************************/

void sleep(){
	//sleep_mode=1;
	NRF_CE_LO();//выключаем приёмник в nrf24
	printf("Going to sleep\n\r");
	//Настраеваем режим сна STOP, выход из которого по прерыванию EXTI
	SCB_SCR|=SCB_SCR_SLEEPDEEP;
	PWR_CR&=~PWR_CR_PDDS;
	PWR_CR |= PWR_CR_LPDS;
	PWR_CR|=PWR_CR_CWUF;
	gpio_clear(GPIOB,GPIO12);//экономим ещё 0.3ма
	sleep_mode=1;//запоминаем что заснули
	__asm__("WFI");
	}
void wake(){
	//После выхода из сна надо перенастроить тактирование!!!
	rcc_clock_setup_in_hsi_out_48mhz();
	gpio_set(GPIOB,GPIO12);
	NRF_CE_HI();//включаем приёмник
	sleep_mode=0;
	}
/**********************************************************************
 * Инициализация переферии
 **********************************************************************/
 
void exti0_init(){//Прерывание по нисходящему фронту ноги IRQ nrf24l01
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	// Set GPIO0 (in GPIO port A) to 'input float'. 
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,	GPIO0);
	// Configure the EXTI subsystem. 
	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);
	}

void spi_nrf_init(void) {
	rcc_periph_clock_enable(NRF_RCC_SPI);
	/* Configure GPIOs:
	 *  SS=PA4,
	 *  SCK=PA5,
	 *  MISO=PA6
	 *  MOSI=PA7
	 */
	gpio_set_mode(NRF_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, NRF_SPI_MOSI|NRF_SPI_SCK);
	//CSN and CE pin nrf24l01
	//CSN GPIO4
	//CE GPIO3
	gpio_set_mode(NRF_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, NRF_CE|NRF_CSN);
	//MISO
	gpio_set_mode(NRF_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, NRF_SPI_MISO);
	spi_reset(NRF_SPI);
	//SPI MODE1
  	spi_init_master(NRF_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_32, 
					SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
					SPI_CR1_CPHA_CLK_TRANSITION_1,
					SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_software_slave_management(NRF_SPI);
  spi_set_nss_high(NRF_SPI);//Это важно, без него не заведётся
  spi_enable(NRF_SPI);
}

static void i2c_setup(void){
	/* Enable clocks for I2C2 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);
	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C1_SCL|GPIO_I2C1_SDA);
	//SDA PB7
	//SCL PB6
	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C1);
	//36 is APB1 speed in MHz
	i2c_set_speed(I2C1,i2c_speed_fm_400k,36);
	i2c_peripheral_enable(I2C1);
	}


void pcf_ext_init(void){
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	nvic_enable_irq(NVIC_EXTI1_IRQ);
	// Set GPIO0 (in GPIO port A) to 'input float'. 
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,	GPIO1);
	// Configure the EXTI subsystem. 
	exti_select_source(EXTI1, GPIOA);
	exti_set_trigger(EXTI1, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI1);
	pcf_write(0xff);//включаем подтяжку
	}


static void usart1_setup(void){
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
	/* Enable the USART1 interrupt. */
	//PA9 TX,PA10 RX
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
				  GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	nvic_enable_irq(NVIC_USART1_IRQ);
	usart_enable_rx_interrupt(USART1);
	//Finally enable the USART.
	usart_enable(USART1);
	}


/***********************************************************************
 * Прерывания
 ***********************************************************************/
 

void usart1_isr(void){
	//if(sleep_mode)wake();
	static uint8_t cnt=0;
	char ch;
	ch=usart_recv_blocking(USART1);
	if(ch!='\r')usart_send_blocking(USART1,ch);
	nrf_cmd_line[cnt++]=ch;
	//cnt++;
	if(cnt>127)cnt=0;
	gpio_toggle(GPIOB,GPIO12);
	if(ch=='\r'){
		nrf_cmd_line[cnt]=0;
		cnt=0;
		nrf_get_cmd(nrf_cmd_line);
		nrf_flush_cmd_bufer();
		}
	}

void exti0_isr(void){
	exti_reset_request(EXTI0);//Флаг прерывания надо сбросить вручную
	gpio_toggle(GPIOA,GPIO12);
	uint8_t status,temp,len;
	uint8_t data[32]={0};
	status=nrf_status();
	//если прерывание вфызвано передачей
	if(status==0x2e)printf("ISR EXTI0 0x%x\r\n#>",status); 
							else printf("ISR EXTI0 0x%x\r\n",status);
	if(status&(MAX_RT|TX_DS)){
		temp=nrf_rreg(CONFIG);
		nrf_wreg(CONFIG,temp|PRIM_RX);
		NRF_CE_HI();//включаем приём
		}	
	if(status&RX_DR){
		len=nrf_rrx_payload_width();//длинна пакета
		nrf_read(RD_RX_PLOAD,data,len);	
		printf("DATA RECIV %d: %s\r\n",len,data);
		}
	nrf_wreg(STATUS,status);
	nrf_flushtx();
	nrf_flushrx();
}

void exti1_isr(){
	if(sleep_mode)wake();
	exti_reset_request(EXTI1);//Флаг прерывания надо сбросить вручную
	key=pcf_read();
	printf("PCF INT1 read value: 0x%x\r\n",key);
	//if(key==0xff)sleep();
	}

//***********************************************************************
//Настройка функций ввода-вывода printf/scanf
void usart_send_char(char c){
	usart_send_blocking(USART1,c);	
	}
char usart_reciv_char(void){
	return usart_recv_blocking(USART1);
	}
void send_line(char*str){
	while(*str) usart_send_blocking(USART1,*str++);
	}
/***********************************************************************
 * Функции работы с PCF не считая прерываеия 
 ***********************************************************************/
 
void pcf_write(uint8_t data){
	i2c_transfer7(I2C1,PCF_ADDRES,&data,1,0,0);
	}
uint8_t pcf_read(void){
	uint8_t temp=0;
	i2c_transfer7(I2C1,PCF_ADDRES,0,0,&temp,1);
	return temp;
	}

//**********************************************************************
//Функции управления!
void key_proc(uint8_t *key){

/*keyboard layout
 * 0xfe 0x7f          0xFE  0xf7  0xEF
 * 0xfd 0xbf            0xFB    0xBF
 * 0xfb 0xdf          0xFD  0x7F  0xDF
 * 0xf7 0xef
 */
	
	if(*key==0xff) return;
	//if(key==0xff) sleep();
	printf("proc %d\r\n",*key);
	switch(*key){
		case 0xfe:
			nrf_send("BIRD",4);
			break;
		case 0xf7:
			nrf_send("PK2 82 0", 8); //key_up
			break;
		case 0x7f:
			nrf_send("PK2 81 0", 8); //key_down
			break;
		case 0xfd:
			nrf_send("WSR",3);
			break;
		case 0xbf:
			nrf_send("PK2 44 0", 8); //spase
			break;
		case 0xfb:
			nrf_send("PK2 42 0", 8); //back spase
			break;
		case 0xdf:
			nrf_send("MSHIFT",6);
			break;
		case 0xef:
			nrf_send("GIRL",4);
			break;
		}
	*key=0xff;
}

int main(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
									GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_set(GPIOB,GPIO12);
	usart1_setup();
	set_output(usart_send_char);
	printf("start0\n\r");
	//set_input(usart_reciv_char);
	//for(uint32_t i=0;i<0xffffff;i++)__asm__("nop");
	spi_nrf_init();
	gpio_clear(GPIOB,GPIO12);
	printf("start\n\r");
	nrf_init();
	printf("nrf init");
	exti0_init();
	
	
	delay_us(2000);
	NRF_CE_HI();
	nrf_print_report();
	i2c_setup();
	pcf_ext_init();
	
	//printf("sleep\n\r");
	//sleep();
	//printf("wake up\n\r");
	
	
	
	uint8_t counter=0;
//	uint8_t temp;
	nrf_flush_cmd_bufer();
	while (1) {
		gpio_toggle(GPIOB, GPIO12);
		key_proc(&key);
		if(key==0xff)sleep();
		for(uint32_t i=0;i<0xffff;i++)__asm__("nop");
		
	}

	return 0;
}
