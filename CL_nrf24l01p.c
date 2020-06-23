#include  "CL_nrf24l01p.h"

extern uint8_t NRFSTATUS;
//extern void spiSendMultiDummy(uint8_t * data, uint32_t len, uint8_t *rx_buff);
//extern void spiSend(uint8_t  data);
extern uint8_t spiRead(void);
extern void spiSendMultiByte(uint8_t * data, uint32_t len, uint8_t *rx_buff);

extern void delayMS(uint32_t ms);
extern void delayUS(uint32_t ms);
extern void CL_printMsg(char *msg, ...);


uint32_t TX_ADDR_HIGHBYTES = 0x00000000;
uint8_t TX_ADDR_LOW_BYTE = 0x00;
uint8_t dummy_array[100];
uint8_t rx_buff[100];

/* ______________________________________________________________ */
void NRF_cmd_read_multi_byte_reg(uint8_t reg, uint8_t numBytes, uint8_t *buff)//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ NOT used
{
	NRF_CSN_LOW();
	
	nrfRX.spiSend(reg);   //read data register
	NRFSTATUS = spiRead();
	
	nrfRX.spiSend(0xff); 
	buff[0] = spiRead();
	
	nrfRX.spiSend(0xff); 
	buff[1] = spiRead();
	
	nrfRX.spiSend(0xff); 
	buff[2] = spiRead();
	
	nrfRX.spiSend(0xff); 
	buff[3] = spiRead();
	
	nrfRX.spiSend(0xff); 
	buff[4] = spiRead();
	
	//nrfRX.spiSendMultiDummy(numBytes, buff);   //faster than for loop below 	
	
	NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
uint8_t  NRF_cmd_read_single_byte_reg(uint8_t reg)
{
	NRF_CSN_LOW();
	
	nrfRX.spiSend(reg);    // send register name
	NRFSTATUS = spiRead();
	nrfRX.spiSend(DUMMYBYTE);
   
	NRF_CSN_HIGH();
	return spiRead();
}
/* ______________________________________________________________ */
void NRF_cmd_write_TX_ADDR(uint8_t *addr, uint8_t len)//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ NOT used
{
	
	
	NRF_CSN_LOW();
	
	nrfRX.spiSend(TX_ADDR | W_REGISTER);     //read data register
	NRFSTATUS = spiRead();
	spiSendMultiByte(addr, len, rx_buff);    //faster than for loop with individual nrfRX.spiSend
	 
	NRF_CSN_HIGH();
	
}
/* ______________________________________________________________ */
void NRF_cmd_write_reg(uint8_t reg, uint8_t value)
{
	
	
	NRF_CSN_LOW();
	
	nrfRX.spiSend(reg | W_REGISTER);      //read data register
	NRFSTATUS = spiRead();
	nrfRX.spiSend(value);    //faster than for loop with individual nrfRX.spiSend
	 
	NRF_CSN_HIGH();
	
}
/* ______________________________________________________________ */
void NRF_cmd_modify_reg(uint8_t reg, uint8_t bit, uint8_t state)
{
	// thhis is a "read-modify-write" procedure 
	
	
	//READ
	uint8_t reg_value =  NRF_cmd_read_single_byte_reg(reg);
	
	
	//MODIFY
	if (state)
	{
		reg_value |= (1 << bit);
	}
	else
	{
		reg_value &= ~(1 << bit);
	}
	
	
	
	//WRITE
	NRF_CSN_LOW();	
	
	nrfRX.spiSend(reg | W_REGISTER);      //write data register
	NRFSTATUS = spiRead();
	
	nrfRX.spiSend(reg_value); 
	NRFSTATUS = spiRead();

	NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
void NRF_cmd_write_entire_reg(uint8_t reg, uint8_t value)
{
	NRF_CSN_LOW();	
	
	nrfRX.spiSend(reg | W_REGISTER);       //write data register
	NRFSTATUS = spiRead();
	
	nrfRX.spiSend(value); 
	NRFSTATUS = spiRead();

	NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
void NRF_cmd_write_5byte_reg(uint8_t reg, uint8_t value)
{
	NRF_CSN_LOW();	
	
	nrfRX.spiSend(reg | W_REGISTER);        //write data register
	NRFSTATUS = spiRead();
	
	nrfRX.spiSend(value); 
	NRFSTATUS = spiRead();
	nrfRX.spiSend(0x02); 
	NRFSTATUS = spiRead();
	nrfRX.spiSend(0x00); 
	NRFSTATUS = spiRead();
	nrfRX.spiSend(0x00); 
	NRFSTATUS = spiRead();
	nrfRX.spiSend(0x00); 
	NRFSTATUS = spiRead();

	NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
void NRF_cmd_setup_addr_width(uint8_t width) 
{
	NRF_CSN_LOW();	
	
	nrfRX.spiSend(SETUP_AW | W_REGISTER);        //write data register
	NRFSTATUS = spiRead();
	
	nrfRX.spiSend(width & 0x03); 
	NRFSTATUS = spiRead();

	NRF_CSN_HIGH();
}
void NRF_cmd_read_RX_PAYLOAD(uint8_t *rx_buffer, uint8_t len)
{
	NRF_STOP_LISTENING();
	//NRF_CE_LOW(); //disable receiver mode
	NRF_CSN_LOW();
	
	nrfRX.spiSend(R_RX_PAYLOAD);
	NRFSTATUS = spiRead();
	
	//for some reason i get an empty byte at the beggning of rx payload so ill just read it here
	//so its not included in my "len"
	nrfRX.spiSend(DUMMYBYTE);
	NRFSTATUS = spiRead();
	
	spiSendMultiByte(dummy_array, len, rx_buffer);    //check if this works or if im reading SPI DR too soon after witing to it
	
	NRF_CSN_HIGH();	
}
/* ______________________________________________________________ */
void NRF_cmd_write_TX_PAYLOAD(uint8_t *data, uint8_t len)
{	

	NRF_CE_LOW();
	NRF_CSN_LOW();
	
	nrfRX.spiSend(W_TX_PAYLOAD);     
	NRFSTATUS = spiRead();
	spiSendMultiByte(data, len, rx_buff);     //faster than for loop with individual nrfRX.spiSend
	 
	NRF_CSN_HIGH();
	//CE set high to start transmition if in TX mode
	// must be held high for a bit then back low
	NRF_CE_HIGH();
	delayUS(50);
	NRF_CE_LOW();
}
void NRF_cmd_FLUSH_TX(void)
{
	NRF_CSN_LOW();
	
	nrfRX.spiSend(FLUSH_TX);      //read data register
	NRFSTATUS = spiRead();	 
	
	NRF_CSN_HIGH();
}
void NRF_cmd_FLUSH_RX(void)
{
	NRF_CSN_LOW();
	
	nrfRX.spiSend(FLUSH_RX);       //read data register
	NRFSTATUS = spiRead();	 
	
	NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
void NRF_cmd_reuse_TX_PL(void)
{
	
}
/* ______________________________________________________________ */
uint8_t  NRF_cmd_read_status(void)
{
	NRF_CSN_LOW();
	
	nrfRX.spiSend(NOP);      //read data register
	NRFSTATUS = spiRead();
	
	NRF_CSN_HIGH();
	return NRFSTATUS;
}
/* ______________________________________________________________ */
void NRF_setup_config_reg(void)
{
	
}
/* ______________________________________________________________ */
void NRF_init_tx(CL_nrf24l01p_init_tx_type *nrf_type)
{
	uint8_t payload[] = "EDWINFAIRCHILD.COM\n";   //// delete eventually
	NRF_CE_LOW();
	NRF_cmd_modify_reg(NRF_CONFIG, PWR_UP, 1);     //turn on
	NRF_cmd_modify_reg(NRF_CONFIG, PRIM_RX, 0);     //set as TX
	NRF_cmd_modify_reg(NRF_CONFIG, CRCO, 0);       //set CRC scheme
	NRF_cmd_modify_reg(NRF_CONFIG, EN_CRC, nrf_type->enable_crc);      //turn on CRC

	NRF_cmd_modify_reg(NRF_CONFIG, MASK_TX_DS, !(nrf_type->enable_tx_ds_interrupt));  //enable TX_DS interrupt on IRQ pin
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_MAX_RT, !(nrf_type->enable_max_rt_interrupt));    //enable MAX_RT interrupt on IRQ pin
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_RX_DR, 1);      //disable RX_DR interrupt on IRQ pin

	NRF_cmd_setup_addr_width(nrf_type->address_width);   //address width

	
	// TODO :@@@@@@@@@@@@@@@@@@@@@@ if auto ack is enabled
	//enable auto ack on pipe 0
	//enable auto ack
	NRF_cmd_write_entire_reg(EN_AA, 0x00);   //clear all pipes of auto ack
	if(nrf_type->enable_auto_ack)
		NRF_cmd_modify_reg(EN_AA, 0, 1);  //enable auto ack on pipe 1 if auto ack is enabled

	// TODO :@@@@@@@@@@@@@@@@@@@@@@@@@@@setup wait time inbetween retries
	//setup retransmit max count in SETUP_RETR register
	NRF_cmd_write_entire_reg(SETUP_RETR, 0x2F);   // done here 0x2F = 15 retries and 750uS wait time

	NRF_cmd_write_entire_reg(NRF_STATUS, 0x70);    //clear any interrupts
	
	NRF_cmd_write_entire_reg(RF_CH, nrf_type->rf_channel);

	//----------------------------end of setup essentials
	NRF_set_tx_addr(nrf_type->tx_addr_byte_2_5, nrf_type->tx_addr_byte_1);
	
	
	//initiate control object
	nrfTX.set_addr = &NRF_set_tx_addr;
	nrfTX .transmit = &NRF_cmd_write_TX_PAYLOAD;

	delayMS(100);  // voltage ramp up and crystal stabalize


}
void NRF_init_rx(CL_nrf24l01p_init_rx_type *nrf_type)
{
	//RX and pipe  1 are interwined
	NRF_STOP_LISTENING(); 
	
	//initialize NRF controller
	nrf_type->cmd_listen = &NRF_cmd_listen; 
	nrf_type->cmd_read_payload = &NRF_cmd_read_RX_PAYLOAD;
	nrf_type->cmd_set_addr = &NRF_set_rx_addr;
	nrf_type->cmd_get_status  = &NRF_cmd_get_status;
	
	nrfRX.spiSend = *nrf_type->spi_spiSend;
	//nrfRX.spiRead = *nrf_type->spi_spiRead;
	//CONFIG register
	NRF_cmd_modify_reg(NRF_CONFIG, PWR_UP, 1);    // turn on 
	NRF_cmd_modify_reg(NRF_CONFIG, PRIM_RX, 1);   //set as RX		
	NRF_cmd_modify_reg(NRF_CONFIG, CRCO, nrf_type->set_crc_scheme);       //set CRC scheme
	NRF_cmd_modify_reg(NRF_CONFIG, EN_CRC, nrf_type->set_enable_crc);      //turn on	CRC	
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_TX_DS, 1);  //dsiable TX_DS interrupt on IRQ pin
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_MAX_RT, 1);    //disable MAX_RT interrupt on IRQ pin
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_RX_DR, !(nrf_type->set_enable_rx_dr_interrupt)); //enable RX_DR interrupt on IRQ pin
	
	
	//address width
    NRF_cmd_write_entire_reg(SETUP_AW, nrf_type->set_address_width);
	//NRF_cmd_setup_addr_width(nrf_type->address_width);  

	//NRF_cmd_write_entire_reg(EN_RXADDR, 0x00);   //disable all piepes to have a known state on this register
	NRF_cmd_modify_reg(EN_RXADDR, nrf_type->set_rx_pipe, 1);   //enable pipe 5
	
	
   //Pipe number and pipe payload width register is offset by 17
	NRF_cmd_write_entire_reg((nrf_type->set_rx_pipe + 17 ), nrf_type->set_payload_width);    //pipe 5 payload width	
	
	//rf channel
	NRF_cmd_write_entire_reg(RF_CH, nrf_type->set_rf_channel);
	
	
	//@@@@@@@@   auto ack 
	NRF_cmd_write_entire_reg(EN_AA, 0x00);  //clear all pipes of auto ack
	
	if(nrf_type->set_enable_auto_ack)
	{	
		NRF_cmd_modify_reg(EN_AA, ENAA_P1, 1);   //enable auto ack on pipe 1	
		NRF_cmd_modify_reg(EN_AA, nrf_type->set_rx_pipe, 1); //enable auto ack on pipe 5	
	}
	
	NRF_set_rx_addr(nrf_type->set_rx_pipe , nrf_type->set_rx_addr_byte_2_5, nrf_type->set_rx_addr_byte_1);	

		
	NRF_cmd_write_entire_reg(NRF_STATUS, 0x70);   //clear any interrupts	
	
	
	
	delayMS(100);


}
void NRF_set_tx_addr(uint32_t addr_high, uint8_t addr_low) //  pipe 0 allso must match this for auto ack
{
	NRF_CSN_LOW();

	nrfRX.spiSend(TX_ADDR | W_REGISTER);           //write data register
	NRFSTATUS = spiRead();

	nrfRX.spiSend(addr_low);
	NRFSTATUS = spiRead();
	nrfRX.spiSend(addr_high & 0xFF);
	NRFSTATUS = spiRead();
	nrfRX.spiSend((addr_high >> 8) & 0xFF);
	NRFSTATUS = spiRead();
	nrfRX.spiSend((addr_high >> 16) & 0xFF);
	NRFSTATUS = spiRead();
	nrfRX.spiSend((addr_high >> 24) & 0xFF);
	NRFSTATUS = spiRead();
	NRF_CSN_HIGH();

	delayMS(1);
	NRF_CSN_LOW();


	//pipe 0 must have same address as TX_ADDR
	nrfRX.spiSend(RX_ADDR_P0 | W_REGISTER);             //write pipe 0 register
	NRFSTATUS = spiRead();

	nrfRX.spiSend(addr_low);
	nrfRX.spiSend(addr_high & 0xFF);
	nrfRX.spiSend((addr_high >> 8) & 0xFF);
	nrfRX.spiSend((addr_high >> 16) & 0xFF);
	nrfRX.spiSend((addr_high >> 24) & 0xFF);

	NRFSTATUS = spiRead();



	NRF_CSN_HIGH();
}

void NRF_set_rx_addr(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low) //  pipe 0 allso must match this for auto ack
{
	uint8_t temp;
	
	if (rx_pipe > 1) // because pipe 0 and 1 just need to written directly with 5 bytes
		{
			//RX_ADDR_P# is offest by 10 with the pipe number
			NRF_cmd_write_entire_reg((rx_pipe + 10), addr_low);   	//for pipe 2 to 5
	
			NRF_CSN_LOW();	
	
			//the high bytes of the address go in pipe 1
			nrfRX.spiSend(RX_ADDR_P1 | W_REGISTER);             
			NRFSTATUS = spiRead();
	
			nrfRX.spiSend(addr_low); 	
			temp = spiRead();
	
		
			nrfRX.spiSend(addr_high & 0xFF); 
			temp = spiRead();
	
		
			nrfRX.spiSend((addr_high >> 8) & 0xFF); 
			temp = spiRead();
	
		
		
			nrfRX.spiSend((addr_high >> 16) & 0xFF); 
			temp = spiRead();
	
		
			nrfRX.spiSend((addr_high >> 24) & 0xFF); 
			temp = spiRead();
			NRF_CSN_HIGH();
		}
	else
	{
		//this is for pipe 1 or 0 
		NRF_CSN_LOW();	
	
		//the high bytes of the address go in pipe 1
		nrfRX.spiSend((rx_pipe + 10) | W_REGISTER);             
		NRFSTATUS = spiRead();
	
		nrfRX.spiSend(addr_low); 	
		temp = spiRead();
	
		
		nrfRX.spiSend(addr_high & 0xFF); 
		temp = spiRead();
	
		
		nrfRX.spiSend((addr_high >> 8) & 0xFF); 
		temp = spiRead();
	
		
		
		nrfRX.spiSend((addr_high >> 16) & 0xFF); 
		temp = spiRead();
	
		
		nrfRX.spiSend((addr_high >> 24) & 0xFF); 
		temp = spiRead();
		NRF_CSN_HIGH();
	}
	
	
}

void NRF_cmd_listen(void)
{
	NRF_START_LISTENING(); 
}

uint8_t NRF_cmd_get_status(void)
{
	return NRF_cmd_read_single_byte_reg(NRF_STATUS);
}
