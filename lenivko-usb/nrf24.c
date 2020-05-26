/***********************************************************************
 * Библиотека для раюоты с nrf24l01
 * функционал небогатый
 * 20 мая функционал существенно расширен
 * v 0.9
 ***********************************************************************/

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include "stdint.h"
#include "nrf24.h"

//#define DEBUG 1

#if DEBUG
#include "printf.h"
#endif



uint8_t nrf_rreg(uint8_t addr){
	uint8_t data;
	NRF_CSN_LO();
	data=NRF_SPI_TRANSFER(addr);
	if (addr!=STATUS) data=NRF_SPI_TRANSFER(NOP);
	NRF_WSPI();
	NRF_CSN_HI();
	return data;
	}

void nrf_wreg(uint8_t addr, uint8_t data){
	addr|=W_REGISTER;//включим бит записи в адрес
	NRF_CSN_LO();
	NRF_SPI_TRANSFER(addr);
	NRF_SPI_TRANSFER(data);
	NRF_WSPI();
	NRF_CSN_HI();
	}

void nrf_read(uint8_t addr,uint8_t *data,uint8_t count){
	NRF_CSN_LO();
	NRF_SPI_TRANSFER(addr);
	for(uint8_t i=0; i<count;i++)data[i]=NRF_SPI_TRANSFER(NOP);
	NRF_WSPI();
	NRF_CSN_HI();
	}

void nrf_write(uint8_t addr,uint8_t *data,uint8_t count){
	addr|=W_REGISTER;
	NRF_CSN_LO();
	NRF_SPI_TRANSFER(addr);
	for(uint8_t i=0; i<count;i++)NRF_SPI_TRANSFER(data[i]);
	NRF_WSPI();
	NRF_CSN_HI();
	}
	
void nrf_toggle_features(void){
  	NRF_CSN_LO();
  	/*Без этой команды не устанавливается произвольная
  	 * длинна пакета, инструкция не всегда срабатывает с первого раза
  	 */ 
	NRF_SPI_TRANSFER(ACTIVATE);//активирует регистр FEATURE
	NRF_SPI_TRANSFER(0x73);    
	NRF_WSPI();
	NRF_CSN_HI();
}

void nrf_adr_chenge(){//меняет местами адреса TX,PO и P1
	uint8_t temp[5];
	uint8_t temp2[5];
	nrf_read(RX_ADDR_P0,temp,5);
	nrf_read(RX_ADDR_P1,temp2,5);
	nrf_write(TX_ADDR,temp2,5);
	nrf_write(RX_ADDR_P0,temp2,5);
	nrf_write(RX_ADDR_P1,temp,5);
	nrf_report_addr_str(TX_ADDR);
	nrf_report_addr_str(RX_ADDR_P0);
	nrf_report_addr_str(RX_ADDR_P1);
	
}

nrf_flushtx(void){
	NRF_CSN_LO();
	NRF_SPI_TRANSFER(FLUSH_TX);
	NRF_WSPI();
	delay_us(10);
	NRF_CSN_HI();
	}

void nrf_flushrx(void){
	NRF_CSN_LO();
	NRF_SPI_TRANSFER(FLUSH_RX);
	NRF_WSPI();
	delay_us(10);
	NRF_CSN_HI();
	}

void nrf_rx_mode(void){
	nrf_wreg(CONFIG,0x0f);
	NRF_CE_HI();
	}

void nrf_tx_address(uint8_t* addr, uint8_t addr_len){
	nrf_write(RX_ADDR_P0, addr, addr_len);
	nrf_write(TX_ADDR, addr, addr_len);
	}

void delay_us(uint32_t delay){
	delay*=72/6;
	while(delay--)__asm__("nop");
	}


uint8_t nrf_send(uint8_t *data,uint8_t len){
	uint8_t fifo;
	NRF_CE_LO();
	nrf_flushtx();
	nrf_wreg(CONFIG,0x0e); //потом исправить на установку бит
	delay_us(25);
	nrf_write_bufer(WR_TX_PLOAD,data,len);
	NRF_CE_HI();
	delay_us(50);
	NRF_CE_LO();
	//Это нужно чтобы строка не осталась в буфере, а была точно отправлена
	while(!(nrf_rreg(FIFO_STATUS)&TX_EMPTY)){
		NRF_CE_HI();
		delay_us(25);
		NRF_CE_LO();
		}
	}

void nrf_write_bufer(uint8_t addr,uint8_t *data,uint8_t count){
	NRF_CSN_LO();
	NRF_SPI_TRANSFER(addr);
	for(uint8_t i=0; i<count;i++)NRF_SPI_TRANSFER(data[i]);
	NRF_WSPI();
	}

uint8_t nrf_status(){
	uint8_t data=0;
	NRF_CSN_LO();
	data=NRF_SPI_TRANSFER(NOP);
	NRF_WSPI();
	NRF_CSN_HI();
	return data;
	}

void nrf_init(void){
	uint8_t self_addr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Собственный адрес
	uint8_t remote_addr[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2}; // Адрес удалённой стороны
	NRF_CE_HI();
	delay_us(500);
	//FEATURE следует активировать с самого начала
	nrf_wreg(FEATURE, 0x04);
	while(nrf_rreg(FEATURE)!=0x4){//эта штука устанавливается порой не с первого раза
		nrf_toggle_features();//а без неё не работает
		//delay_us(500);
		nrf_wreg(FEATURE, 0x04);// разрешение произвольной длины пакета данных
		//delay_us(500);
		}
	nrf_wreg(CONFIG, 0x0f); // Set PWR_UP bit, enable CRC(2 byte) &Prim_RX:1 reciver
	//delay_us(500);
	nrf_wreg(EN_AA, 0x02); // Enable Pipe1
	nrf_wreg(EN_RXADDR, 0x03); // Enable Pipe1
	nrf_wreg(SETUP_AW, 0x03); // Setup address width=5 bytes
	nrf_wreg(SETUP_RETR, 0x5f); // 250us, 2 retrans
	nrf_wreg(RF_CH, 0); // частота 2400 MHz
	nrf_write(RX_ADDR_P0,remote_addr,5);
	nrf_write(TX_ADDR,remote_addr,5);
	nrf_write(RX_ADDR_P0,remote_addr,5);
	nrf_write(RX_ADDR_P1,self_addr,5);
	nrf_wreg(RF_SETUP, 0x06); //TX_PWR:0dBm, Datarate:1Mbps
	nrf_wreg(RX_PW_P0, 32);
	nrf_wreg(RX_PW_P1, 32); //32
	nrf_wreg(DYNPD, 0x03);//(1 << DPL_P0) | (1 << DPL_P1)); // включение произвольной длины для каналов 0 и 1
	//nrf_wreg(FEATURE, 0x04);// разрешение произвольной длины пакета данных
	//nrf_wreg(CONFIG, 0x0f);
	NRF_CE_HI();
	//if(nrf_rreg(CONFIG)==0x0f)printf("nrf24 init is ok\r\n");
	}


#if DEBUG
/**********************************************************************
 * Секция отладочных функций и управления по UART
 **********************************************************************/

#define NRF_REG_CNT 26
struct nrf_registr {
	char *reg;
	uint8_t num;
} nrf_registers[]= {
"CONFIG",      0x00,
"EN_AA",       0x01,
"EN_RXADDR",   0x02,
"SETUP_AW",    0x03,
"SETUP_RETR",  0x04,
"RF_CH",       0x05,
"RF_SETUP",    0x06,
"STATUS",      0x07,
"OBSERVE_TX",  0x08,
"CD",          0x09,
"RX_ADDR_P0",  0x0A,
"RX_ADDR_P1",  0x0B,
"RX_ADDR_P2",  0x0C,
"RX_ADDR_P3",  0x0D,
"RX_ADDR_P4",  0x0E,
"RX_ADDR_P5",  0x0F,
"TX_ADDR",     0x10,
"RX_PW_P0",    0x11,
"RX_PW_P1",    0x12,
"RX_PW_P2",    0x13,
"RX_PW_P3",    0x14,
"RX_PW_P4",    0x15,
"RX_PW_P5",    0x16,
"FIFO_STATUS", 0x17,
"DYNPD",       0x1C,
"FEATURE",	   0x1D
};

void nrf_print_report(){
	for(uint8_t i=0; i<0xa;i++) nrf_report_reg_str(i);
	for(uint8_t i=0x11;i<0x18;i++)nrf_report_reg_str(i);
	nrf_report_reg_str(0x1c);
	nrf_report_reg_str(0x1d);
	for(uint8_t i=0xa;i<0x11;i++) nrf_report_addr_str(i);
	}

void nrf_report_reg_str(uint8_t reg){
	uint8_t val;
	uint8_t regname_index=0;
	val=nrf_rreg(reg);
	char val_base2[9]="00000000";
	//здесь может закрасться ошибка
	while(reg!=nrf_registers[regname_index].num)regname_index++;
	for(int i=7;i>=0;i--)if(val&(1<<i))val_base2[7-i]='1'; else val_base2[7-i]='0'; 
	val_base2[8]=0;
	printf("%02X %s: 0x%02X 0b%s\r\n", reg, 
					nrf_registers[regname_index].reg, val, val_base2);
	}
void nrf_report_addr_str(uint8_t addr){
	uint8_t addr_len;
	uint8_t regname_index=0;
	uint8_t addr_val[5];
	
	while(addr!=nrf_registers[regname_index].num)regname_index++;
	addr_len=nrf_rreg(SETUP_AW)+2;//адрес бывает разной длинны
	//printf("addr_len=%d\r\n",addr_len);
	nrf_read(addr,addr_val,addr_len);
	printf("%02X %s:",addr, nrf_registers[regname_index].reg);
	for(uint8_t i=0;i<5;i++) printf(" %02X",addr_val[i]);
	printf("\n\r");
	}

uint8_t nrf_regname2int(char *str){
	for(uint8_t i=0; i<NRF_REG_CNT; i++) 
			if(strstr(str, nrf_registers[i].reg)) return nrf_registers[i].num;
	return 0xEE;
	}


void nrf_get_cmd(char *str){
	uint8_t addr[5];
	uint8_t reg_addr;
	uint8_t addr_len=5;
	uint8_t a,b,c,d,e,f;
	//ниже идёт парсинг строки с командой
	//пример
	//CMD ARG1 ARG2
	if(strstr(str, "----")) return;
	else if(strstr(str, "INFO")) nrf_print_report();
	else if(strstr(str, "RW")){// RW REGNAME VAL
		printf("RW func call\r\n");
		//a=regname2int(str);
		sscanf(strchr(str,'0'),"%X %X",&a,&b);
		printf("nrf_write(0x%x, 0x%X)",a,b);
		nrf_wreg(a,b);
		nrf_report_reg_str(a);
		}
	else if(strstr(str, "ADW")){// AW REGNAME VAL1 VAL2 VAL3 VAL4 VAL5
		addr_len=nrf_rreg(SETUP_AW)&3+2;//адрес бывает разной длинны
		switch(addr_len){
			case 3:
				sscanf(strchr(str,'0'),"%X %X %X %X",&reg_addr, 
											addr,addr+1,addr+2,addr+3);
				break;
			case 4:
				sscanf(strchr(str,'0'),"%X %X %X %X %X", &reg_addr, 
											addr,addr+1,addr+2,addr+3);
				break;
			default:
				sscanf(strchr(str,'0'),"%X %X %X %X %X %X", &reg_addr,
									addr,addr+1,addr+2,addr+3,addr+4);
				break;
		}
		nrf_write(reg_addr,addr,addr_len);
		printf("Addres is set!\r\n");
		nrf_report_addr_str(reg_addr);
		}
		else if(strstr(str, "RR")){
			sscanf(strchr(str,'0'),"%d",&reg_addr);
			nrf_report_reg_str(reg_addr);
			}
	//	else if(strstr(str, "CEH")) NRF_CE_HI();
	//	else if(strstr(str, "CEL")) NRF_CE_LO();
		else if(strstr(str, "CEP")){
			 NRF_CE_HI();
			 for(uint32_t i=0;i<0xffff;i++)__asm__("nop");
			 NRF_CE_LO();
			}
		else if(strstr(str, "SND"))nrf_send("1234567890absdefgh",18);
		else if(strstr(str, "SS")) nrf_send(str,30);
		else if(strstr(str, "RCV")) nrf_recive();
		else if(strstr(str, "ADRC")) nrf_adr_chenge();
		else printf("command not found");
	//nrf_flush_cmd_bufer();
	printf("\r\n#>");
	}


	
//приём без прерывания
void nrf_recive(){
	uint8_t temp[32];
	uint8_t len=5;
	uint8_t a;
	//nrf_flushrx();
	nrf_wreg(CONFIG,0x0F);
	NRF_CE_HI();
	for(uint8_t i=0;i<32;i++)temp[i]=0;
	printf("start leasting\r\n");
	while(nrf_status()!=RX_DR)__asm__("nop");
	nrf_report_reg_str(STATUS);
	a=nrf_status();
	nrf_wreg(STATUS,a);//Сбросим прерываеие
	len=nrf_rrx_payload_width();
	nrf_read(RD_RX_PLOAD,temp,len);
	NRF_CE_LO();
	printf("data len: %d\r\n",len);
	printf("data: %s\r\n",temp);
	nrf_wreg(STATUS,temp);
	nrf_flushtx();
	nrf_flushrx();
	}

#endif
