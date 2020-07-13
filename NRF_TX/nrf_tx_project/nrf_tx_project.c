
//---------| STM32 LL API
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>
#include <stm32f1xx_ll_exti.h>
#include <stm32f1xx_ll_utils.h>

//---------| My personal common Libs
#include "CL_CONFIG.h"
#include "CL_printMsg.h"
#include "CL_systemClockUpdate.h"
#include "CL_nrf24l01p.h"

//---------| freeRTOS
#include "FreeRTOS.h"
#include "task.h"


//---------| NRF marcos
#define NRF_SPI			SPI1
#define NRF_CE_PIN		LL_GPIO_PIN_2  //chip enable pin to make the transceiver transmit or listen
#define NRF_CE_PORT		GPIOA
#define NRF_IRQ_PIN		LL_GPIO_PIN_4 // [ interrupt pin active Low ]
#define NRF_IRQ_PORT	GPIOA
#define NRF_CSN_PIN		LL_GPIO_PIN_3 //Low when SPI active
#define NRF_CSN_PORT    GPIOA
#define NRF_CLK_PIN		LL_GPIO_PIN_5
#define NRF_MOSI_PIN	LL_GPIO_PIN_7
#define NRF_MISO_PIN	LL_GPIO_PIN_6

#define LED1	LL_GPIO_PIN_6
#define LED2	LL_GPIO_PIN_5


//----------| Globals 
uint8_t tx_data_buff[32];
uint8_t rx_data_buff[32];
uint8_t payload[32];

CL_nrf24l01p_init_type nrf;    // make an instance of the driver
//----------| Prototypes
void init_spi1(void);
void init_pins(void);
void spiSendMultiByte(uint8_t * data_to_send, uint32_t len, uint8_t *rx_buffer);
void spiSend(uint8_t  data);
uint8_t spiRead(void);
void CE_pin_HIGH(void);
void CE_pin_LOW(void);
void CSN_pin_HIGH(void);
void CSN_pin_LOW(void);
void initializeNRF(void);

void init_leds(void);
void blink_led1(void *args);
void blink_led2(void *args);
int main(void)
{
	
	//------ init common libs
	setSysClockTo72();	
	CL_printMsg_init_Default(false);
	
	//init spi for NRF
	init_pins();
	init_spi1();
   
	// init NRF 
	initializeNRF();
	



	init_leds();
	xTaskCreate(blink_led1, "LED_blink_1", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(blink_led2, "LED_blink_2", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
	// Start the scheduler.
	vTaskStartScheduler();

	


	for (;;)
	{

	}
}//------------------------------------------------------------------------------------------
void blink_led1(void *args)
{
	while (1)
	{	
		LL_GPIO_TogglePin(GPIOB, LED1);
		CL_printMsg("\nTask1 p\n");
		vTaskDelay(pdMS_TO_TICKS(100));
		CL_printMsg("\nTask1 a\n");
	}
}//------------------------------------------------------------------------------------------
void blink_led2(void *args)
{
	while (1)
	{	
		LL_GPIO_TogglePin(GPIOB, LED2);
		CL_printMsg("\nTask2 p\n");
		vTaskDelay(pdMS_TO_TICKS(1000));
		CL_printMsg("\nTask2 a\n");
	}
	
}//------------------------------------------------------------------------------------------
void init_leds(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; 
	LL_GPIO_InitTypeDef leds_struct;
	LL_GPIO_StructInit(&leds_struct);
	
	leds_struct.Pin        = LED1 | LED2;
	leds_struct.Mode       = LL_GPIO_MODE_OUTPUT;
	leds_struct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	leds_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	
	LL_GPIO_Init(GPIOB, &leds_struct);
	

	
}
void initializeNRF(void)
{
	
	//common stuff
	nrf.set_address_width = FIVE_BYTES;
	nrf.set_crc_scheme = ENCODING_SCHEME_1_BYTE;
	nrf.set_enable_auto_ack = true;
	nrf.set_rf_channel = 0x7B;
	nrf.set_enable_dynamic_pl_width = true;		
	nrf.set_enable_crc = true;
	
	//rx stuff
	/*
	nrf.set_enable_rx_dr_interrupt = true;
	nrf.set_enable_rx_mode = true; 
	nrf.set_rx_pipe = PIPE_5;
	nrf.set_rx_addr_byte_1 = 0x28;
	nrf.set_rx_addr_byte_2_5 = 0xAABBCCDD;
	nrf.set_payload_width = 2;	
	*/
	//tx stuff
	nrf.set_enable_tx_mode = true; 
	nrf.set_enable_max_rt_interrupt = true;
	nrf.set_enable_tx_ds_interrupt = true;
	nrf.set_tx_addr_byte_1 = 0x28;
	nrf.set_tx_addr_byte_2_5 = 0xAABBCCDD;
	
	//hardware specific functions
	nrf.spi_spiSend = &spiSend;	
	nrf.spi_spiRead = &spiRead;
	nrf.spi_spiSendMultiByte = &spiSendMultiByte;
	
	nrf.pin_CE_HIGH		= &CE_pin_HIGH;
	nrf.pin_CE_LOW		= &CE_pin_LOW;
	nrf.pin_CSN_HIGH	= &CSN_pin_HIGH;
	nrf.pin_CSN_LOW		= &CSN_pin_LOW;	
	
	NRF_init(&nrf);   // initialize

	// act as rx and start listening
	/*
	nrf.cmd_act_as_RX(true);
	nrf.cmd_listen();
	*/

	//act as TX and send data
	nrf.cmd_act_as_RX(false);	
	
	nrf.cmd_transmit(payload, strlen(payload));
	
}
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
	
}//------------------------------------------------------------------------------------------
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
	LL_GPIO_Init(NRF_CE_PORT, &nrfPins);  // CE & CSN on same port only need this once

	
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
	
	LL_GPIO_SetOutputPin(GPIOA, NRF_CSN_PIN);
	LL_GPIO_ResetOutputPin(GPIOA, NRF_CE_PIN);
	NVIC_EnableIRQ(EXTI4_IRQn); 	//enable IRQ on Pin 4
}//------------------------------------------------------------------------------------------
void spiSend(uint8_t  data)
{	
	__IO uint8_t *spidr = ((__IO uint8_t *)&NRF_SPI->DR);
	//while (!(NRF_SPI->SR & SPI_SR_TXE)) ;
	NRF_SPI->DR = data;
	while (NRF_SPI->SR & LL_SPI_SR_BSY) ;
	
}//------------------------------------------------------------------------------------------
void spiSendMultiByte(uint8_t * data_to_send, uint32_t len, uint8_t *rx_buffer)
{ 
	//consider what to do with returned bytes usually when 
	//sending data returned bytes are not needed 
	__IO uint8_t *spidr = ((__IO uint8_t *)&NRF_SPI->DR);

	for (uint8_t i = 0; i < len; i++)
	{				
		NRF_SPI->DR = *data_to_send++;
		//if you mcu is really fast you might want to the the chekc busy flag code here	
		 rx_buffer[i] = spiRead();
		while (NRF_SPI->SR & SPI_SR_BSY) ;	//this is the check busy flag code		
	}
}//------------------------------------------------------------------------------------------
uint8_t spiRead(void)
{	
	return (uint8_t)(NRF_SPI->DR);	
}//------------------------------------------------------------------------------------------
void CE_pin_HIGH(void)
{
	NRF_CE_PORT->BSRR = NRF_CE_PIN;
}//------------------------------------------------------------------------------------------
void CE_pin_LOW(void)
{
	NRF_CE_PORT->BRR = NRF_CE_PIN;
}//------------------------------------------------------------------------------------------
void CSN_pin_HIGH(void)
{
	NRF_CSN_PORT->BSRR = NRF_CSN_PIN;
}//------------------------------------------------------------------------------------------
void CSN_pin_LOW(void)
{
	NRF_CSN_PORT->BRR = NRF_CSN_PIN;
}//------------------------------------------------------------------------------------------
void EXTI4_IRQHandler(void)
{
	EXTI->PR = EXTI_PR_PIF4;  //clear pending interrupt

	//flag = 0x55;
	GPIOC->ODR ^= 1 << 13;	
}//------------------------------------------------------------------------------------------