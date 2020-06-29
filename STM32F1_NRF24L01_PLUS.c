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

#define NRF_SPI			SPI1
#define NRF_CE_PIN		LL_GPIO_PIN_2  //chip enable pin to make the transceiver transmit or listen
#define NRF_CE_PORT		GPIOA
#define NRF_IRQ_PIN		LL_GPIO_PIN_4 // [ interrupt pin active Low ]
#define NRF_IRQ_PORT	GPIOA
#define NRF_CSN_PIN		LL_GPIO_PIN_3 //Low when SPI active
#define NRF_CSN_PORT    GPIOA
// [ SPI : keep at or below 2Mbs ]

#define NRF_CLK_PIN		LL_GPIO_PIN_5
#define NRF_MOSI_PIN	LL_GPIO_PIN_7
#define NRF_MISO_PIN	LL_GPIO_PIN_6

#define NRF_PINS_CLOCK_ENABLE() (RCC->APB2ENR |= RCC_APB2ENR_IOPAEN )  //given the pins are on GPIOA




#define NRF_CE_HIGH()  ( NRF_CE_PORT->BSRR = NRF_CE_PIN	)
#define NRF_CE_LOW()   ( NRF_CE_PORT->BRR = NRF_CE_PIN	)
#define NRF_CSN_HIGH() ( NRF_CSN_PORT->BSRR = NRF_CSN_PIN )
#define NRF_CSN_LOW()  ( NRF_CSN_PORT->BRR = NRF_CSN_PIN )

#define NRF_START_LISTENING()  ( NRF_CE_PORT->BSRR = NRF_CE_PIN	)
#define NRF_STOP_LISTENING()   ( NRF_CE_PORT->BRR = NRF_CE_PIN	)

uint8_t NRFSTATUS = 0x00;
uint8_t tx_data_buff[32];
uint8_t rx_data_buff[32];

uint8_t multibyte_buff[10] = { 0 };
uint8_t flag = 0x55;


//-----------| NRF Prototypes |-----------
void init_pins(void);
void init_spi1(void);

void spiSendMultiByte(uint8_t * data_to_send, uint32_t len, uint8_t *rx_buffer);
void spiSend(uint8_t  data);
uint8_t spiRead(void);
void CE_pin_HIGH(void);
void CE_pin_LOW(void);
void CSN_pin_HIGH(void);
void CSN_pin_LOW(void);


//------------| Debug stuff |----------
void init_debug_led(void);
void blinkLed(void);
void printRegister(uint8_t reg);



//#define BETX
#define BERX


/* ______________________________________________________________ */
int main(void)
{
	uint8_t buff[5] = { 3, 3, 3, 3, 3 };
	uint8_t payload[32] = "Edwin";
	setSysClockTo72();
	CL_delay_init();
	CL_printMsg_init_Default(false);
	init_debug_led();
	
	init_pins();
	init_spi1();
	NRF_CSN_HIGH();
	NRF_CE_LOW();
	
	delayMS(200);

	
	CL_nrf24l01p_init_type myRX;
	
	myRX.set_address_width = FIVE_BYTES;
	myRX.set_crc_scheme = ENCODING_SCHEME_1_BYTE;
	myRX.set_enable_auto_ack = true;
	myRX.set_enable_crc = true;
	myRX.set_rf_channel = 0x7B;
	myRX.set_enable_dynamic_pl_width = true;	
	
	
	
	myRX.set_enable_rx_dr_interrupt = true;
	myRX.set_enable_rx_mode = true; //---------------
	myRX.set_rx_pipe = PIPE_5;
	myRX.set_rx_addr_byte_1 = 0x28;
	myRX.set_rx_addr_byte_2_5 = 0xAABBCCDD;
	myRX.set_payload_width = 2;	
	
	myRX.set_enable_tx_mode = true ; //-------------
	myRX.set_enable_max_rt_interrupt = true;
	myRX.set_enable_tx_ds_interrupt = true;
	myRX.set_tx_addr_byte_1 = 0x28;
	myRX.set_tx_addr_byte_2_5 = 0xAABBCCDD;
	
	myRX.spi_spiSend = &spiSend;	
	myRX.spi_spiRead = &spiRead;
	myRX.spi_spiSendMultiByte = &spiSendMultiByte;
	
	myRX.pin_CE_HIGH = &CE_pin_HIGH;
	myRX.pin_CE_LOW  = &CE_pin_LOW;
	myRX.pin_CSN_HIGH = &CSN_pin_HIGH;
	myRX.pin_CSN_LOW = &CSN_pin_LOW;
	
	
	
	NRF_init(&myRX);
	
	myRX.cmd_act_as_RX(true); //-----------
	
	
	//myRX.cmd_transmit(payload,strlen(payload));
	myRX.cmd_listen();


	uint8_t counter = 0x00;
	 
	
	for (;;)
	{

		#ifdef BETX
				if (flag == 0x55)
				{			
					flag = 0x00;	
					//printRegister(NRF_STATUS);
					myRX.cmd_clear_interrupts();						
					sprintf(payload, "Edwin %d", counter++);
					
					myRX.cmd_transmit(payload, strlen(payload));
				
					delayMS(200);
				}
	
		#endif

		#ifdef BERX
		
				if (flag == 0x55)
				{		
					CL_printMsg("interrupted\n");
					flag = 0x00;					
					myRX.cmd_clear_interrupts();
					uint8_t len = NRF_cmd_read_dynamic_pl_width();
					
					
					if (len < 33)
					{
						
				
						CL_printMsg("len : %d \n", len);
						myRX.cmd_read_payload(rx_data_buff, len);
						NRF_cmd_FLUSH_RX();					
						for (int i = 0; i < len; i++)
						{
							CL_printMsg(" %c ", rx_data_buff[i]);
							rx_data_buff[i] = 0;
						}
						CL_printMsg("\n-\n");
						
					}
					myRX.cmd_listen();					
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

/* ______________________________________________________________ */

/* ______________________________________________________________ */


void spiSend(uint8_t  data)
{	
	__IO uint8_t *spidr = ((__IO uint8_t *)&NRF_SPI->DR);
	//while (!(NRF_SPI->SR & SPI_SR_TXE)) ;
	NRF_SPI->DR = data;
	while (NRF_SPI->SR & LL_SPI_SR_BSY) ;
	
}


void spiSendMultiByte(uint8_t * data_to_send, uint32_t len, uint8_t *rx_buffer)
{ 
	//consider what to do with returned bytes usually when 
	//sending data returned bytes are not needed 
	__IO uint8_t *spidr = ((__IO uint8_t *)&NRF_SPI->DR);

	for(uint8_t i = 0 ; i < len ; i++)
	{				
		NRF_SPI->DR = *data_to_send++;
	   //if you mcu is really fast you might want to the the chekc busy flag code here	
		rx_buffer[i] = spiRead();
		while (NRF_SPI->SR & SPI_SR_BSY) ;	//this is the check busy flag code		
	}
}

uint8_t spiRead(void)
{	
	return (uint8_t)(NRF_SPI->DR);	
}



void CE_pin_HIGH(void)
{
	GPIOA->BSRR = LL_GPIO_PIN_2;
}
void CE_pin_LOW(void)
{
	GPIOA->BRR = LL_GPIO_PIN_2;
}
void CSN_pin_HIGH(void)
{
	GPIOA->BSRR = LL_GPIO_PIN_3;
}

void CSN_pin_LOW(void)
{
	GPIOA->BRR = LL_GPIO_PIN_3;
}




//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

/* ______________________________________________________________ */


/* ______________________________________________________________ */


void init_pins(void)
{
	//clock enable GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
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
	nrfPins.Pull = LL_GPIO_PULL_UP;
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
	flag = 0x55;
	GPIOC->ODR ^= 1 << 13;

	
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