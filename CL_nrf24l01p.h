#ifndef CL_nrf24l01p_H_
#define CL_nrf24l01p_H_

#include  <stdint.h>
#include <stdbool.h>


//-----------| NRF COMMANDS |----------
#define R_REGISTER			0x00  //must be ORed with a registr address to get correct command value / but since its zero then it really does nothing
#define W_REGISTER			0x20  //must be ORed with a register address to get correct command value
#define R_RX_PAYLOAD		0x61
#define W_TX_PAYLOAD		0xA0
#define FLUSH_TX			0xE1
#define FLUSH_RX			0xE2
#define REUSE_TX_PL			0xE3 //unused as of now 
#define R_RX_PL_WID			0x60
#define W_ACK_PAYLOAD		0xA8 //unused as of now
#define NOP					0xFF
#define W_TX_PAYLOAD_NACK	0xB0 // will implement soon

#define ACTIVATE			0x50
#define ACTIVATE_BYTE       0x73  // the activate command must be followed by this byte

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


//---------| NRF BIT Definitions |---------------
//CONFIG Register : CONFIG
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
//ENABLE AUTO ACK Register : EN_AA
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
//Enabled RX Addresses Register : EN_RXADDR
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
//Setup of Address Widths Register : SETUP_AW
#define AW          0
//Setup of Automatic Retransmission Register : SETUP_RETR
#define ARD         4
#define ARC         0
//RF Channel Register :RF_CH
#define RF_CH_BITS  0
//RF Setup Register : RF_SETUP
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define LNA_HCURR   0
//STATUS register : STATUS
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
//Transmit observe register : OBSERVE_TX
#define PLOS_CNT    4
#define ARC_CNT     0
// CD
#define CD_BIT      0
//FIFO Status Register : FIFO_STATUS
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
//Enable dynamic payload length Register : DYNPD
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
//Feature Register : FEATURE
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0


//---------------Custom Defines -----------------------
//I like this naming convention better and can apply to all the enable registers
#define PIPE_0	0x00 
#define PIPE_1	0x01
#define PIPE_2	0x02
#define PIPE_3	0x03
#define PIPE_4	0x04
#define PIPE_5	0x05

//some offsets that make fr more readability
#define RX_ADDR_OFFSET  0x0A
#define RX_PW_OFFSET    0x11

//SETUP_AW byte codes
#define THREE_BYTES 0b01
#define FOUR_BYTES  0b10
#define FIVE_BYTES  0b11

#define ENCODING_SCHEME_1_BYTE  0x00
#define ENCODING_SCHEME_2_BYTE  0x01


#define DUMMYBYTE  0xF1

//functions pointers... duh
typedef		void(*fptr)(); //you will use 4 of these
typedef		void(*spiSend_ptr)(uint8_t data);
typedef		void(*spiSednMultiByte_ptr)(uint8_t * data, uint32_t len, uint8_t *rx_buff);	
typedef		uint8_t(*spiRead_ptr)();

typedef		void(*actAsRx_ptr)(bool state);
typedef		void(*tx_data_ptr)(uint8_t *data, uint8_t len); 
typedef		void(*tx_set_addr_ptr)(uint32_t addr_high, uint8_t addr_low, bool auto_ack); 
typedef		void(*rx_data_ptr)(uint8_t *data, uint8_t len); 
typedef		void(*rx_set_addr_ptr)(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low); 
typedef		uint8_t(*getStatus_ptr)();
typedef		uint8_t(*getpayload_width_ptr)();
typedef		uint8_t(*getpipe_current_pl_ptr)();


typedef struct //user structure to setup and control the NRF24
{
	//common settings
	bool		set_enable_crc;                 // true / false
	uint8_t		set_crc_scheme;					// ENCODING_SCHEME_1_BYTE / ENCODING_SCHEME_2_BYTE
	bool		set_enable_auto_ack;			// true / false
	uint8_t		set_rf_channel;					// 0 - 126  (2.400GHz to 2.525GHz)(2400 + RF_CH)
	uint8_t		set_address_width;				// THREE_BYTES / FOUR_BYTES / FIVE_BYTES
	bool		set_enable_dynamic_pl_width;	// true / false
	
	//rx settings
	bool		set_enable_rx_mode;				// true / false 
	uint8_t		set_rx_pipe;					// PIPE_1 / PIPE_2 / PIPE_3 .. . .. 
	uint8_t		set_rx_addr_byte_1;				// low byte will go in RX_ADDR_P#
	uint32_t	set_rx_addr_byte_2_5;			// high byte will go in Pipe 1
	uint8_t		set_payload_width;				// 1 - 32 
	bool		set_enable_rx_dr_interrupt;		// true / false
	
	//tx settings
	bool		set_enable_tx_mode;				// true / false
	uint8_t		set_tx_addr_byte_1;				// low byte will go in TX_ADDR register
	uint32_t	set_tx_addr_byte_2_5;			// high byte low byte will go in TX_ADDR register 
												// and PIPE 0 if auto ack is enabled	
	bool		set_enable_max_rt_interrupt;	// true / false
	bool		set_enable_tx_ds_interrupt;		// true / false	
	//commands : function points that can be used as function calls
	fptr					cmd_clear_interrupts;
	getStatus_ptr			cmd_get_status;
	rx_set_addr_ptr			cmd_set_rx_addr;
	tx_set_addr_ptr			cmd_set_tx_addr;
	fptr					cmd_listen;  
	getpayload_width_ptr    cmd_get_payload_width;
	getpipe_current_pl_ptr  cmd_get_pipe_num_current_pl;
	rx_data_ptr				cmd_read_payload;
	tx_data_ptr				cmd_transmit;	
	actAsRx_ptr				cmd_act_as_RX;
	fptr                    cmd_flush_rx;
	fptr                    cmd_flush_tx;	
	
	//hardware specific functions needed : user has no need to call these : for driver use only
	spiSend_ptr				spi_spiSend;
	spiSednMultiByte_ptr	spi_spiSendMultiByte;
	spiRead_ptr				spi_spiRead;
	
	fptr					pin_CE_HIGH;
	fptr					pin_CE_LOW;
	fptr					pin_CSN_HIGH;
	fptr					pin_CSN_LOW;
}CL_nrf24l01p_init_type;




//this is an internal structure to control and call external functions, i find it much simpler
//and perhaps more efficient, otherwise I would need to pass my HUGE ASS main structure to all the internal
//functions. so i chose to do it this way.
typedef struct 
{
	spiSend_ptr				spiSend;
	spiSednMultiByte_ptr	spiSendMultiByte;
	spiRead_ptr				spiRead;
	
	fptr					NRF_CE_HIGH;
	fptr					NRF_CE_LOW;
	fptr					NRF_CSN_HIGH;
	fptr					NRF_CSN_LOW;
	
}NRF_type;

NRF_type NRF;


//------------------------------------------------------------------------
//----------------| Function Prototypes |----------------------------------

uint8_t  NRF_cmd_read_single_byte_reg(uint8_t reg); 
void NRF_cmd_read_multi_byte_reg(uint8_t reg, uint8_t numBytes, uint8_t *buff);
void NRF_cmd_write_5byte_reg(uint8_t reg, uint8_t value); 
void NRF_cmd_modify_reg(uint8_t reg, uint8_t bit, uint8_t state); 
void NRF_cmd_write_entire_reg(uint8_t reg, uint8_t value); 

uint8_t NRF_cmd_get_pipe_current_pl(void);
uint8_t  NRF_cmd_read_dynamic_pl_width(void);
uint8_t NRF_cmd_get_status(void);
 
void NRF_cmd_read_RX_PAYLOAD(uint8_t *data, uint8_t len);
void NRF_cmd_write_TX_PAYLOAD(uint8_t *data, uint8_t len);
void NRF_cmd_setup_addr_width(uint8_t width); //
void NRF_cmd_FLUSH_TX(void);
void NRF_cmd_FLUSH_RX(void);
void NRF_cmd_reuse_TX_PL(void);
void NRF_cmd_activate(void);
void NRF_cmd_listen(void);
void NRF_cmd_clear_interrupts(void);
void NRF_cmd_act_as_RX(bool state);

void NRF_set_tx_addr(uint32_t addr_high, uint8_t addr_low , bool auto_ack); 
void NRF_set_rx_addr(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low);
void NRF_init(CL_nrf24l01p_init_type *nrf_type);


#endif