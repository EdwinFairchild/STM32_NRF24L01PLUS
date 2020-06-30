#include  "CL_nrf24l01p.h"

extern uint8_t NRFSTATUS;
//extern void spiSendMultiDummy(uint8_t * data, uint32_t len, uint8_t *rx_buff);
//extern void spiSend(uint8_t  data);

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
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(reg);   //read data register
	NRFSTATUS = NRF.spiRead();
	
	NRF.spiSend(0xff); 
	buff[0] = NRF.spiRead();
	
	NRF.spiSend(0xff); 
	buff[1] = NRF.spiRead();
	
	NRF.spiSend(0xff); 
	buff[2] = NRF.spiRead();
	
	NRF.spiSend(0xff); 
	buff[3] = NRF.spiRead();
	
	NRF.spiSend(0xff); 
	buff[4] = NRF.spiRead();
	
	//NRF.spiSendMultiDummy(numBytes, buff);   //faster than for loop below 	
	
	NRF.NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
uint8_t  NRF_cmd_read_single_byte_reg(uint8_t reg)
{
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(reg);    // send register name
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend(DUMMYBYTE);
   
	NRF.NRF_CSN_HIGH();
	return NRF.spiRead();
}
uint8_t  NRF_cmd_read_dynamic_pl_width(void)
{
	return NRF_cmd_read_single_byte_reg(R_RX_PL_WID) ;
}
/* ______________________________________________________________ */

/* ______________________________________________________________ */
void NRF_cmd_write_reg(uint8_t reg, uint8_t value)
{
	
	
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(reg | W_REGISTER);      //read data register
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend(value);    //faster than for loop with individual NRF.spiSend
	 
	NRF.NRF_CSN_HIGH();
	
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
	NRF.NRF_CSN_LOW();	
	
	NRF.spiSend(reg | W_REGISTER);      //write data register
	NRFSTATUS = NRF.spiRead();
	
	NRF.spiSend(reg_value); 
	NRFSTATUS = NRF.spiRead();

	NRF.NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
void NRF_cmd_write_entire_reg(uint8_t reg, uint8_t value)
{
	NRF.NRF_CSN_LOW();	
	
	NRF.spiSend(reg | W_REGISTER);       //write data register
	NRFSTATUS = NRF.spiRead();
	
	NRF.spiSend(value); 
	NRFSTATUS = NRF.spiRead();

	NRF.NRF_CSN_HIGH();
}
/* ______________________________________________________________ */
void NRF_cmd_write_5byte_reg(uint8_t reg, uint8_t value)
{
	NRF.NRF_CSN_LOW();	
	
	NRF.spiSend(reg | W_REGISTER);        //write data register
	NRFSTATUS = NRF.spiRead();
	
	NRF.spiSend(value); 
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend(0x02); 
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend(0x03); 
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend(0x04); 
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend(0x05); 
	NRFSTATUS = NRF.spiRead();

	NRF.NRF_CSN_HIGH();
}
/* ______________________________________________________________ */

void NRF_cmd_read_RX_PAYLOAD(uint8_t *rx_buffer, uint8_t len)
{
	NRF.NRF_CE_LOW();
	
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(R_RX_PAYLOAD);
	NRFSTATUS = NRF.spiRead();
	
	//for some reason i get an empty byte at the beggning of rx payload so ill just read it here
	//so it doesnt use up a spot in my "len" count
	NRF.spiSend(DUMMYBYTE);
	NRFSTATUS = NRF.spiRead();

	NRF.spiSendMultiByte(dummy_array, len, rx_buffer);    
	
	NRF.NRF_CSN_HIGH();	
}
/* ______________________________________________________________ */
void NRF_cmd_write_TX_PAYLOAD(uint8_t *data, uint8_t len)
{	

	NRF.NRF_CE_LOW();
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(W_TX_PAYLOAD);     
	NRFSTATUS = NRF.spiRead();
	NRF.spiSendMultiByte(data, len, rx_buff);     //faster than for loop with individual NRF.spiSend
	 
	NRF.NRF_CSN_HIGH();
	//CE set high to start transmition if in TX mode
	// must be held high for a bit then back low
	NRF.NRF_CE_HIGH();
	//delayUS(50);
	NRF.NRF_CE_LOW();
}


/* ______________________________________________________________ */
void NRF_cmd_reuse_TX_PL(void)
{
	
}
void NRF_cmd_activate(void)
{
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(ACTIVATE);    
	NRFSTATUS = NRF.spiRead();	
	NRF.spiSend(ACTIVATE_BYTE);    
	NRFSTATUS = NRF.spiRead();	
	
	NRF.NRF_CSN_HIGH();
}
/* ______________________________________________________________ */

/* ______________________________________________________________ */

/* ______________________________________________________________ */



void NRF_init(CL_nrf24l01p_init_type *nrf_type)
{	
	//initialize NRF command functions
	nrf_type->cmd_clear_interrupts	= &NRF_cmd_clear_interrupts;
	nrf_type->cmd_get_status		= &NRF_cmd_get_status;
	nrf_type->cmd_set_rx_addr		= &NRF_set_rx_addr;
	nrf_type->cmd_set_tx_addr		= &NRF_set_tx_addr;
	nrf_type->cmd_listen			= &NRF_cmd_listen; 
	nrf_type->cmd_get_payload_width = &NRF_cmd_read_dynamic_pl_width;
	nrf_type->cmd_get_pipe_num_current_pl = &NRF_cmd_get_pipe_current_pl;
	nrf_type->cmd_read_payload		= &NRF_cmd_read_RX_PAYLOAD;
	nrf_type->cmd_transmit			= &NRF_cmd_write_TX_PAYLOAD;
	nrf_type->cmd_act_as_RX 		= &NRF_cmd_act_as_RX;
	nrf_type->cmd_flush_rx 			= &NRF_cmd_FLUSH_RX;
	nrf_type->cmd_flush_tx 			= &NRF_cmd_FLUSH_TX;
	NRF.spiSend				= *nrf_type->spi_spiSend;
	NRF.spiRead				= *nrf_type->spi_spiRead;
	NRF.spiSendMultiByte	= *nrf_type->spi_spiSendMultiByte;
	NRF.NRF_CE_HIGH			= *nrf_type->pin_CE_HIGH;
	NRF.NRF_CE_LOW			= *nrf_type->pin_CE_LOW;
	NRF.NRF_CSN_HIGH		= *nrf_type->pin_CSN_HIGH;
	NRF.NRF_CSN_LOW			= *nrf_type->pin_CSN_LOW;	
	
	
	NRF.NRF_CE_LOW();  //start SPI comms
	
	//common configurations	
	NRF_cmd_modify_reg(NRF_CONFIG, PWR_UP, 1);    // turn on 
	//delayMS(100);
	NRF_cmd_modify_reg(NRF_CONFIG, CRCO, nrf_type->set_crc_scheme);      //set CRC scheme
	NRF_cmd_modify_reg(NRF_CONFIG, EN_CRC, nrf_type->set_enable_crc);    //turn on CRC	
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_TX_DS, !(nrf_type->set_enable_tx_ds_interrupt));    //dsiable TX_DS interrupt on IRQ pin
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_MAX_RT, !(nrf_type->set_enable_max_rt_interrupt));   //disable MAX_RT interrupt on IRQ pin
	NRF_cmd_modify_reg(NRF_CONFIG, MASK_RX_DR, !(nrf_type->set_enable_rx_dr_interrupt));   //enable RX_DR interrupt on IRQ pin
	NRF_cmd_write_entire_reg(RF_CH, nrf_type->set_rf_channel);  	//rf channel		
	NRF_cmd_write_entire_reg(EN_AA, 0x00);     //disable auto ack by derfault, might be enabled below if user wants
	NRF_cmd_write_entire_reg(NRF_STATUS, 0x70);      //clear any interrupts
    NRF_cmd_write_entire_reg(SETUP_AW, nrf_type->set_address_width);    //address width		
	

	// SET UP AS RECEIVER
	if (nrf_type->set_enable_rx_mode)
	{		
		NRF_cmd_modify_reg(EN_RXADDR, nrf_type->set_rx_pipe, 1);    //enable rx pipe 
		NRF_set_rx_addr(nrf_type->set_rx_pipe, nrf_type->set_rx_addr_byte_2_5, nrf_type->set_rx_addr_byte_1);			
		
		if (nrf_type->set_enable_dynamic_pl_width)
		{
			NRF_cmd_activate();
			NRF_cmd_modify_reg(FEATURE, EN_DPL, 1); //enable dynamic PL feature
			NRF_cmd_modify_reg(DYNPD, nrf_type->set_rx_pipe, 1); //enable dynamic PL for pipe
			NRF_cmd_modify_reg(DYNPD, PIPE_1, 1);  //enable dynamic PL for pipe
			//auto ack MUST to be enabled if using dynamic payload
			NRF_cmd_modify_reg(EN_AA, ENAA_P1, 1);//enable auto ack on pipe 1	
			NRF_cmd_modify_reg(EN_AA, nrf_type->set_rx_pipe, 1);     //enable auto ack on pipe 5	
		}
		else
		{
			NRF_cmd_write_entire_reg((nrf_type->set_rx_pipe + RX_PW_OFFSET), nrf_type->set_payload_width);   //write the static payload width
			
			//if using static PL width use can still use auto ack but now its optional
			if(nrf_type->set_enable_auto_ack)
			{	
				NRF_cmd_modify_reg(EN_AA, ENAA_P1, 1);       //enable auto ack on pipe 1	
				NRF_cmd_modify_reg(EN_AA, nrf_type->set_rx_pipe, 1);     //enable auto ack on pipe 5	
			}
		}	
	}
	// SET UP AS TRANSMITTER
	if(nrf_type->set_enable_tx_mode)
	{		
		NRF_cmd_write_entire_reg(SETUP_RETR, 0x2F);	
		
		NRF_set_tx_addr(nrf_type->set_tx_addr_byte_2_5, nrf_type->set_tx_addr_byte_1, nrf_type->set_enable_auto_ack);
		
		if (nrf_type->set_enable_dynamic_pl_width)
		{
			NRF_cmd_activate();
			NRF_cmd_modify_reg(FEATURE, EN_DPL, 1);    //enable dynamic PL feature
			NRF_cmd_modify_reg(DYNPD, PIPE_0, 1);		
		}			
	}
}
void NRF_set_tx_addr(uint32_t addr_high, uint8_t addr_low, bool auto_ack)
{	
	NRF.NRF_CSN_LOW(); //start SPI comms by a LOW on CSN

	NRF.spiSend(TX_ADDR | W_REGISTER); //send write command to ADDR
	NRFSTATUS = NRF.spiRead();

	/*  5 byte address is devided into a uint8_t low byte
	 *  and a uint32_t high byte
	 *  since the SPI can only send 1 byte at a time
	 *  we first send the low byte
	 *  and then extract the other bytes from the uint32 
	 *  and send them one by one LSB first
	 */
	
	NRF.spiSend(addr_low); 
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend(addr_high & 0xFF);
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend((addr_high >> 8) & 0xFF);
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend((addr_high >> 16) & 0xFF);
	NRFSTATUS = NRF.spiRead();
	NRF.spiSend((addr_high >> 24) & 0xFF);
	NRFSTATUS = NRF.spiRead();
	NRF.NRF_CSN_HIGH();
	
	/* If auto ack is enabled then the same address that was written 
	 * to TX_ADDR above must also be written to PIPE 0 because that
	 * is the pipe that it will receive the auto ack on. This cannot
	 * be changed it is hardwaired to receive acks on pipe 0 */
	
	if (auto_ack)
	{	
		NRF_cmd_modify_reg(EN_AA, ENAA_P0, 1);        //enable auto ack on pipe 0	
		NRF.NRF_CSN_LOW();

		//write address into pipe 0
		NRF.spiSend(RX_ADDR_P0 | W_REGISTER);             
		NRFSTATUS = NRF.spiRead();

		NRF.spiSend(addr_low);
		NRF.spiSend(addr_high & 0xFF);
		NRF.spiSend((addr_high >> 8) & 0xFF);
		NRF.spiSend((addr_high >> 16) & 0xFF);
		NRF.spiSend((addr_high >> 24) & 0xFF);

		NRFSTATUS = NRF.spiRead();

		NRF.NRF_CSN_HIGH(); //end spi
	}	
}

void NRF_set_rx_addr(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low) 
{
	uint8_t temp;
	
	if (rx_pipe > 1) // because pipe 0 and 1 just need to written directly with 5 bytes
		{
			//RX_ADDR_P# is offest by 10 = RX_ADDR_OFFSET with the pipe number
			NRF_cmd_write_entire_reg((rx_pipe + RX_ADDR_OFFSET), addr_low);    	//for pipe 2 to 5
	
			NRF.NRF_CSN_LOW();	
	
			//the high bytes of the address go in pipe 1
			NRF.spiSend(RX_ADDR_P1 | W_REGISTER);             
			NRFSTATUS = NRF.spiRead();
	
			NRF.spiSend(addr_low); 	
			temp = NRF.spiRead();
	
		
			NRF.spiSend(addr_high & 0xFF); 
			temp = NRF.spiRead();
	
		
			NRF.spiSend((addr_high >> 8) & 0xFF); 
			temp = NRF.spiRead();
	
		
		
			NRF.spiSend((addr_high >> 16) & 0xFF); 
			temp = NRF.spiRead();
	
		
			NRF.spiSend((addr_high >> 24) & 0xFF); 
			temp = NRF.spiRead();
			NRF.NRF_CSN_HIGH();
		}
	
	else
	{
		//this is for pipe 1 or 0 
		NRF.NRF_CSN_LOW();	
	
		
		NRF.spiSend((rx_pipe + RX_ADDR_OFFSET) | W_REGISTER);             
		NRFSTATUS = NRF.spiRead();
	
		NRF.spiSend(addr_low); 	
		temp = NRF.spiRead();
	
		
		NRF.spiSend(addr_high & 0xFF); 
		temp = NRF.spiRead();
	
		
		NRF.spiSend((addr_high >> 8) & 0xFF); 
		temp = NRF.spiRead();
	
		
		
		NRF.spiSend((addr_high >> 16) & 0xFF); 
		temp = NRF.spiRead();
	
		
		NRF.spiSend((addr_high >> 24) & 0xFF); 
		temp = NRF.spiRead();
		NRF.NRF_CSN_HIGH();
	}
	
	
}

void NRF_cmd_listen(void)
{
	NRF.NRF_CE_HIGH(); 
}

uint8_t NRF_cmd_get_status(void)
{
	return NRF_cmd_read_single_byte_reg(NRF_STATUS);
}

void NRF_cmd_clear_interrupts(void)
{
	NRF_cmd_write_entire_reg(NRF_STATUS, 0x70);
}

uint8_t NRF_cmd_get_pipe_current_pl(void)
{
	return NRF_cmd_read_single_byte_reg(RX_P_NO);
}
void NRF_cmd_FLUSH_TX(void)
{
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(FLUSH_TX);    
	NRFSTATUS = NRF.spiRead();	 
	
	NRF.NRF_CSN_HIGH();
}

void NRF_cmd_FLUSH_RX(void)
{
	NRF.NRF_CSN_LOW();
	
	NRF.spiSend(FLUSH_RX);       
	NRFSTATUS = NRF.spiRead();	 
	
	NRF.NRF_CSN_HIGH();
}
void NRF_cmd_act_as_RX(bool state)
{
	if (state)
	{
		NRF_cmd_modify_reg(NRF_CONFIG, PRIM_RX, 1); 
	}
		
	else
	{
		NRF_cmd_modify_reg(NRF_CONFIG, PRIM_RX, 0); 
	}
		
}
 