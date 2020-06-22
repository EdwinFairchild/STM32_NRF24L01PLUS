#ifndef CL_nrf24l01p_H_
#define CL_nrf24l01p_H_

#include  <stdint.h>
#include <stdbool.h>
#include "stm32f1xx.h"
#include <stm32f1xx_ll_gpio.h> // this not going to work or be portable unless LL is used





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


//-----------| NRF COMMANDS |----------
#define R_REGISTER			0x00  //must be ANDed with a registr address to get correct command value
#define W_REGISTER			0x20  //must be ORed with a register address to get correct command value
#define R_RX_PAYLOAD		0x61
#define W_TX_PAYLOAD		0xA0
#define FLUSH_TX			0xE1
#define FLUSH_RX			0xE2
#define REUSE_TX_PL			0xE3
#define R_RX_PL_WID			0x60
#define W_ACK_PAYLOAD		0xA8 //0b10101xxx where xxx is pipe number valid from 000 to 101 (0 to 5)
#define NOP					0xFF
#define W_TX_PAYLOAD_NACK	0xB0

//-----------| NRF REGISTERS |----------
#define REGISTER_MASK 0x1F
#define NRF_CONFIG  0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
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
#define FEATURE     0x1D
//---------| NRF BITS |---------------
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

#define AUOTACK_PIPE_0 0
#define AUOTACK_PIPE_1 1
#define AUOTACK_PIPE_2 2
#define AUOTACK_PIPE_3 3
#define AUOTACK_PIPE_4 4
#define AUOTACK_PIPE_5 5
//-------------------------------
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

//I like this naming convention better
#define RX_PIPE_0	ERX_P0 
#define RX_PIPE_1	ERX_P1
#define RX_PIPE_2	ERX_P2
#define RX_PIPE_3	ERX_P3
#define RX_PIPE_4	ERX_P4
#define RX_PIPE_5	ERX_P5

//-------------------------------
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

//SETUP_AW byte codes

#define THREE_BYTES 0b01
#define FOUR_BYTES  0b10
#define FIVE_BYTES  0b11


//generic function pointer for a function with no  return and no argumetns
typedef   void(*fptr)(); 


//------------------------------------------------------------------------
//------------------------------------------------------------------------



typedef struct //user structure to setup transmitter
{
	bool	enable_crc;
	bool	enable_auto_ack;
	
	uint8_t crc_scheme;
	
	uint8_t address_width;
	
	//these will be assembled pewer datawidth setting
	uint8_t tx_addr_byte_1;
	uint32_t tx_addr_byte_2_5;

	bool enable_tx_ds_interrupt;
	bool enable_max_rt_interrupt;
	uint8_t rf_channel;


}CL_nrf24l01p_init_tx_type;



//this will be used to point to transmit payload function
typedef  void(*tx_data_ptr)(uint8_t *data, uint8_t len); 

//this will be used to point to set address function
typedef   void(*tx_set_addr_ptr)(uint32_t addr_high , uint8_t addr_low); 


typedef struct //user structure to control transmitter
{

	fptr clear_interrupts;
	fptr get_status;
	tx_set_addr_ptr set_addr;
	tx_data_ptr transmit;

}CL_nrf_tx;

CL_nrf_tx nrfTX;


//------------------------------------------------------------------------
//------------------------------------------------------------------------



typedef struct //user structure to setup transmitter
{
	bool	enable_crc;
	bool	enable_auto_ack;
	
	uint8_t crc_scheme;
	
	uint8_t address_width;
	uint8_t payload_width;
	
	//these will be assembled pewer datawidth setting
	uint8_t tx_addr_byte_1;
	uint32_t tx_addr_byte_2_5;

	bool enable_rx_dr_interrupt;
	uint8_t rx_pipe;
	uint8_t rf_channel;

}CL_nrf24l01p_init_rx_type;



// will use this for the read rx payload function
typedef  void(*rx_data_ptr)(uint8_t *data, uint8_t len); 


typedef struct //user structure to control transmitter
{

	fptr clear_interrupts;
	fptr get_status;
	tx_set_addr_ptr set_addr;
	fptr listen; //use generic pointer for this
	rx_data_ptr read_payload;
	
}CL_nrf_rx;

CL_nrf_rx nrfRX;


//------------------------------------------------------------------------
//------------------------------------------------------------------------

uint8_t  NRF_cmd_read_single_byte_reg(uint8_t reg);
void NRF_cmd_read_multi_byte_reg(uint8_t reg, uint8_t numBytes, uint8_t *buff);
uint8_t  NRF_cmd_read_status(void);
void NRF_cmd_write_5byte_reg(uint8_t reg, uint8_t value);
void NRF_cmd_write_TX_ADDR(uint8_t *addr, uint8_t len);  //5 bytes
void NRF_cmd_modify_reg(uint8_t reg, uint8_t bit, uint8_t state);
void NRF_cmd_write_entire_reg(uint8_t reg, uint8_t value);

void NRF_cmd_read_RX_PAYLOAD(uint8_t *data, uint8_t len);
void NRF_cmd_write_TX_PAYLOAD(uint8_t *data, uint8_t len);
void NRF_cmd_setup_addr_width(uint8_t width);
void NRF_cmd_FLUSH_TX(void);
void NRF_cmd_FLUSH_RX(void);
void NRF_cmd_reuse_TX_PL(void);
void NRF_cmd_listen(void);
uint8_t NRF_cmd_get_status(void);

void NRF_setup_rx(void);
void NRF_setup_tx(void);
void NRF_setup_config_reg(void);

void NRF_init_tx(CL_nrf24l01p_init_tx_type* nrf_type);
void NRF_init_rx(CL_nrf24l01p_init_rx_type *nrf_type);
void NRF_set_tx_addr(uint32_t addr_high, uint8_t addr_low); 
void NRF_set_rx_addr(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low);

#endif