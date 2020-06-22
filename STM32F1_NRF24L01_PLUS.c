/*
 
The SPI commands are shown in Table 20. 
Every new command must be started by a high to low transition
on CSN.
The STATUS register is serially shifted out on the MISO pin simultaneously to the SPI command word shifting
to the MOSI pin.
 
<Command word: MSBit to LSBit (one byte)>
<Data bytes: LSByte to MSByte, MSBit in each byte first>

TX:
-The CSN pin on the 24L01 must be high to start out with. Then, you
bring the CSN pin low to alert the 24L01 that it is about to receive SPI data. (Note: this
pin will stay low throughout the entire transaction.) 
-If you are receiving data bytes for this
instruction, you must then send one byte to the 24L01 for every one byte that you wish to
get out of the 24L01. If you are just sending the 24L01 data, you simply send your data
bytes and generally don’t worry about what it sends back to you.
-When receiving data from the 24L01, it makes absolutely no difference 
what is contained in the data bytes you send after the command byte, 
just so long as you send the correct number of them.
-Once all data is received CSN goes high again

*/

/*
 *	TODO:
 *			Remove dependancy on hardcoded data buffers
 *			and let user just pass pointers to their own
 *			
 *			make a macro to call delay in tx_payload function 
 *			and have user define it
 *			using their own delay function
 *			
 **/



#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_utils.h>
#include <stm32f1xx_ll_spi.h>
#include <stm32f1xx_ll_exti.h>
#include <stm32f1xx_ll_system.h>
//------------| COMM LIBS |----------
#include "CL_CONFIG.h"
#include "CL_delay.h"
#include "CL_systemClockUpdate.h"
#include "CL_printMsg.h"
#include  "CL_nrf24l01p.h"
//-----------| NRF Macros / Variables |-----------
/* 
  [ RX:HIGH:receive packets ]
  [ TX:HIGH:transmitt packets on low to high transt.]
*/
uint8_t NRFSTATUS = 0x00;
uint8_t tx_data_buff[32];
uint8_t rx_data_buff[32];

uint8_t multibyte_buff[10] = { 0 };
uint8_t DUMMYBYTE = 0xFF;


uint8_t flag = 0x00;
//-----------| Prototypes |-----------
void init_pins(void);
void init_spi1(void);
void spiSendMultiDummy(uint32_t len, uint8_t *buff);
void spiSendMultiData(uint8_t * data, uint32_t len);
void spiSend(uint8_t  data);
uint8_t spiRead(void);
void printRegister(uint8_t reg);





//------------| Debug stuff |----------
void init_debug_led(void);
void blinkLed(void);
#define BETX
//#define BERX


/* ______________________________________________________________ */
int main(void)
{
	uint8_t buff[5] = { 3, 3, 3, 3, 3 };
	uint8_t payload[] = { 0, 255 }; //"hello";
	setSysClockTo72();
	CL_delay_init();
	CL_printMsg_init_Default(false);
	init_debug_led();
	
	init_pins();
	init_spi1();
	NRF_CSN_HIGH();
	NRF_CE_LOW();
	NRF_cmd_FLUSH_TX();
	NRF_cmd_FLUSH_RX();


	
	
#ifdef BETX

	CL_nrf24l01p_init_tx_type myNRF;
	myNRF.address_width = FIVE_BYTES;
	myNRF.enable_auto_ack = true;
	myNRF.crc_scheme = 0;  //1 byte
	myNRF.enable_crc = true;
	myNRF.tx_addr_byte_1 = 0x28;
	myNRF.tx_addr_byte_2_5 = 0xAABBCCDD;
	myNRF.enable_max_rt_interrupt = true;
	myNRF.enable_tx_ds_interrupt = true;
	myNRF.rf_channel = 0x7B;
	NRF_init_tx(&myNRF);



	nrfTX.transmit(payload, 2);
	

	
#endif
	
#ifdef BERX
	//NRF_setup_rx();

	
	
	CL_nrf24l01p_init_rx_type myRX;
	myRX.address_width = FIVE_BYTES;
	myRX.crc_scheme = 0;
	myRX.enable_auto_ack = true;
	myRX.enable_crc = true;
	myRX.enable_rx_dr_interrupt = true;
	myRX.rx_pipe = RX_PIPE_5;
	myRX.tx_addr_byte_1 = 0x28;
	myRX.tx_addr_byte_2_5 = 0xAABBCCDD;
	myRX.rf_channel = 0x7B;
	myRX.payload_width = 18;
	NRF_init_rx(&myRX);
	
	nrfRX.listen();
	

#endif


	 
	
	for (;;)
	{

		#ifdef BETX
				if (flag == 0x55)
				{			
					flag = 0x00;	
					//NRF_cmd_read_single_byte_reg(NRF_CONFIG);
					printRegister(NRF_STATUS);		
					CL_printMsg(" \n----\n");
					NRF_cmd_modify_reg(NRF_STATUS, TX_DS, 1);   
					NRF_cmd_modify_reg(NRF_STATUS, MAX_RT, 1);   	
		
					
					delayMS(100);
					//NRF_cmd_write_TX_PAYLOAD(payload, 19);
					payload[0]++;
					payload[1]--;
					nrfTX.transmit(payload, 2);
		
				}
	
		#endif

		#ifdef BERX
		
				if (flag == 0x55)
				{			
					flag = 0x00;					
					NRF_cmd_modify_reg(NRF_STATUS, RX_DR, 1);								
				  	NRF_cmd_read_RX_PAYLOAD(rx_data_buff, 18);
					NRF_cmd_FLUSH_RX();					
					for(int i = 0 ; i < 19 ; i++)
					{
						CL_printMsg(" %c", rx_data_buff[i]);
						rx_data_buff[i] = 0;
					}
					nrfRX.listen();					
				}
		#endif
		
	}
}
/* ______________________________________________________________ */

void printRegister(uint8_t reg)
{
	uint8_t temp; 
	switch(reg)
	{
	case  NRF_STATUS : 
		temp = NRF_cmd_read_single_byte_reg(reg);
	
		CL_printMsg("\n________________Status Register : 0x%02X________________\n" , temp);
		CL_printMsg(" RX_DR  |  TX_DS  |  MAX_RT  |   RX_P_NO   |  TX_FULL \n");
		CL_printMsg("  %d         %d         %d           %d %d %d        %d    \n", (temp >> 6 & 0x01), (temp >> 5 & 0x01), (temp >> 4 & 0x01), (temp >> 3 & 0x01), (temp >> 2 & 0x01), (temp >> 1 & 0x01), (temp >> 0 & 0x01));
		CL_printMsg("------------------------------------------------------\n");
		break;
		
	case NRF_CONFIG :
		temp = NRF_cmd_read_single_byte_reg(NRF_CONFIG);
		CL_printMsg("\n_____________________________________Config Register : 0x%02X____________________________________\n", temp);
		CL_printMsg(" RES  |  MASK_RX_DR  |  MASK_TX_DS  |   MASK_MAX_RT   |  EN_CRC  |  CRCO  |  PWR_UP  |  PRIM_RX \n");
		CL_printMsg("  %d           %d             %d                %d              %d          %d        %d           %d \n", (temp >> 7 & 0x01),(temp >> 6 & 0x01), (temp >> 5 & 0x01), (temp >> 4 & 0x01), (temp >> 3 & 0x01), (temp >> 2 & 0x01), (temp >> 1 & 0x01), (temp >> 0 & 0x01));
		CL_printMsg("----------------------------------------------------------------------------------------------\n");
		break;
		
	default :
		temp = NRF_cmd_read_single_byte_reg(reg);
		CL_printMsg("\n__________Register Value : 0x%02X_____________\n", temp);
		CL_printMsg(" 7  |  6  |  5  |   4   |  3  |  2  |  1  |  0 \n");
		CL_printMsg(" %d     %d     %d      %d      %d     %d     %d     %d\n", (temp >> 7 & 0x01), (temp >> 6 & 0x01), (temp >> 5 & 0x01), (temp >> 4 & 0x01), (temp >> 3 & 0x01), (temp >> 2 & 0x01), (temp >> 1 & 0x01), (temp >> 0 & 0x01));
		CL_printMsg("------------------------------------------------\n");
		break;
	}
}
/* ______________________________________________________________ */

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  Hardware specific functions   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void spiSendMultiDummy( uint32_t len , uint8_t *buff) 
{
	//when you are sending dummybytes its because youre
	//interested in the response hence they replies are
	//sent to a buffer
	__IO uint8_t *spidr = ((__IO uint8_t *)&NRF_SPI->DR);
	uint8_t  i = 0;
	for(int i = 0 ; i < len; i++)
	{
		NRF_SPI->DR = DUMMYBYTE;
	//	while (!(NRF_SPI->SR & SPI_SR_TXE)) ;
		buff[i] = spiRead(); //does this work or is it too soon?
		//len--;
		//i++;
		while (NRF_SPI->SR & SPI_SR_BSY) ;
				
	}

}
/* ______________________________________________________________ */
void spiSendMultiData(uint8_t * data, uint32_t len)
{ 
	//consider what to do with returned bytes usually when 
	//sending data returned bytes are not needed 
	__IO uint8_t *spidr = ((__IO uint8_t *)&NRF_SPI->DR);

	for(uint8_t i = 0 ; i < len ; i++)
	{		
		
		NRF_SPI->DR = *data++;
		
		
		//spiRead();
		//len--;
		while (NRF_SPI->SR & SPI_SR_BSY) ;	
	
		
	}
}
/* ______________________________________________________________ */
void spiSend(uint8_t  data)
{	
	__IO uint8_t *spidr = ((__IO uint8_t *)&NRF_SPI->DR);
	//while (!(NRF_SPI->SR & SPI_SR_TXE)) ;
	NRF_SPI->DR = data;
	while (NRF_SPI->SR & LL_SPI_SR_BSY) ;
	
}
/* ______________________________________________________________ */
uint8_t spiRead(void)
{	
	return (uint8_t)(NRF_SPI->DR);
	
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

/* ______________________________________________________________ */


/* ______________________________________________________________ */
void init_pins(void)
{
	//clocks
	NRF_PINS_CLOCK_ENABLE();
	
	LL_GPIO_InitTypeDef nrfPins;	
	LL_GPIO_StructInit(&nrfPins);	
	
	// CE pin as output : active high/ normally low
	// CSN chip select for spi active low / normally high
	nrfPins.Pin			= NRF_CE_PIN | NRF_CSN_PIN;
	nrfPins.Mode		= LL_GPIO_MODE_OUTPUT;
	nrfPins.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;	
	nrfPins.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
	LL_GPIO_Init(NRF_CE_PORT, &nrfPins); // CE & CSN on same port only need this once
	NRF_CE_LOW();
	NRF_CSN_HIGH();
	
	// IRQ pin as input with interrupt enabled
	nrfPins.Pin			= NRF_IRQ_PIN;
	nrfPins.Mode = LL_GPIO_MODE_INPUT;
	//nrfPins.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(NRF_IRQ_PORT, &nrfPins);
		
	LL_EXTI_InitTypeDef myEXTI = { 0 };
	LL_EXTI_StructInit(&myEXTI);
	myEXTI.Line_0_31		= LL_EXTI_LINE_4;
	myEXTI.LineCommand		= ENABLE;
	myEXTI.Mode				= LL_EXTI_MODE_IT;
	myEXTI.Trigger			= LL_EXTI_TRIGGER_FALLING;
	LL_EXTI_Init(&myEXTI);	

	NVIC_EnableIRQ(EXTI4_IRQn);
	
	
}
/* ______________________________________________________________ */
void init_spi1(void)
{
	// CLOCK  [ Alt Function ] [ GPIOA ] [ SPI1 ]
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN; 	
	
	// GPIO [ PA5:SCK:output:push ] [ PA6:MISO:input:float/pullup ] [ PA7:MOSI:output:push ]
	LL_GPIO_InitTypeDef spiGPIO;
	LL_GPIO_StructInit(&spiGPIO);
	
	spiGPIO.Pin			= NRF_MOSI_PIN | NRF_CLK_PIN;
	spiGPIO.Mode		= LL_GPIO_MODE_ALTERNATE;
	spiGPIO.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	spiGPIO.Speed		= LL_GPIO_SPEED_FREQ_MEDIUM;
	
	LL_GPIO_Init(GPIOA, &spiGPIO);
	
	spiGPIO.Pin		= NRF_MISO_PIN;
	spiGPIO.Mode	= LL_GPIO_MODE_FLOATING;
	spiGPIO.Pull	= LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOA, &spiGPIO);
		
	// SPI
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI);
	
	mySPI.Mode		= LL_SPI_MODE_MASTER;
	mySPI.NSS		= LL_SPI_NSS_SOFT;
	mySPI.BaudRate	= LL_SPI_BAUDRATEPRESCALER_DIV32;

	LL_SPI_Init(NRF_SPI, &mySPI);
	
	
	LL_SPI_Enable(NRF_SPI);	
	
}
/* ______________________________________________________________ */
void EXTI4_IRQHandler(void)
{
	EXTI->PR = EXTI_PR_PIF4; //clear pending interrupt
#ifdef BERX
	
		NRF_STOP_LISTENING();    //CE LOW
#endif
	
	GPIOC->ODR ^= 1 << 13;
	flag = 0x55;
	
}
/* ______________________________________________________________ */
void init_debug_led(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	LL_GPIO_InitTypeDef debugLED;
	LL_GPIO_StructInit(&debugLED);
	debugLED.Pin        = LL_GPIO_PIN_13;
	debugLED.Mode       = LL_GPIO_MODE_OUTPUT;
	debugLED.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	debugLED.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOC, &debugLED);
	GPIOC->BSRR = LL_GPIO_PIN_13;
	
}
/* ______________________________________________________________ */
void blinkLed(void)
{
	for (int i = 0; i < 10; i++)
	{
		GPIOC->ODR ^= 1<<13;
		delayMS(1000);		
	}
		
	
}