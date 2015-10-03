/**
 * \file
 *
 * \brief Board initialization
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.kn2c.com/">kn2c</a>
 */

#include "init.h"

void port_init(void)
{
// 	ioport_init();
// 	ioport_set_port_dir(IOPORT_PORTA, MASK_PORTA, IOPORT_DIR_OUTPUT);
// 	ioport_set_port_dir(IOPORT_PORTB, MASK_PORTB, IOPORT_DIR_OUTPUT);
// 	ioport_set_port_dir(IOPORT_PORTC, MASK_PORTC, IOPORT_DIR_OUTPUT);
// 	ioport_set_port_dir(IOPORT_PORTD, MASK_PORTD, IOPORT_DIR_OUTPUT);
// 	ioport_set_port_dir(IOPORT_PORTE, MASK_PORTE, IOPORT_DIR_OUTPUT);
// 	ioport_set_port_dir(IOPORT_PORTF, MASK_PORTF, IOPORT_DIR_OUTPUT);
// 	ioport_set_port_dir(IOPORT_PORTR, MASK_PORTR, IOPORT_DIR_OUTPUT);
//	ioport_set_pin_sense_mode(NRF24L01_IRQ_LINE, IOPORT_SENSE_RISING);
	PORTA.DIR = MASK_PORTA;
	PORTB.DIR = MASK_PORTB;
	PORTC.DIR = MASK_PORTC;
	PORTD.DIR = MASK_PORTD;
	PORTE.DIR = MASK_PORTE;
	PORTF.DIR = MASK_PORTF;
	PORTR.DIR = MASK_PORTR;

	ioport_set_value(NRF24L01_CS_LINE  , IOPORT_PIN_LEVEL_HIGH);
	ioport_set_value(NRF24L01_MOSI_LINE, IOPORT_PIN_LEVEL_HIGH);
	ioport_set_value(NRF24L01_SCK_LINE , IOPORT_PIN_LEVEL_HIGH);
	
	ioport_configure_pin(NRF24L01_IRQ_LINE, PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc);
	PORTD.INTCTRL  = PORT_INT0LVL_HI_gc;
	PORTD.INT0MASK = ioport_pin_to_mask(NRF24L01_IRQ_LINE);
}

void spi_init(void)
{
	sysclk_enable_peripheral_clock(&NRF24L01_SPI);
	spi_xmega_set_baud_div(&NRF24L01_SPI,8000000UL,F_CPU);
	spi_enable_master_mode(&NRF24L01_SPI);
	spi_enable(&NRF24L01_SPI);
}

void nrf_init (void)
{
	//NRF24L01_CE_LOW;       //disable transceiver modes
	delay_ms(11);      //power on reset delay needs 10.3ms//amin changed 100ms to 11ms
	NRF24L01_Clear_Interrupts();
	NRF24L01_Flush_TX();
	NRF24L01_Flush_RX();
	//NRF24L01_CE_LOW;
	if (RobotID < 6)
	NRF24L01_Init_milad(_RX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	else if(RobotID > 5)
	NRF24L01_Init_milad(_RX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_WriteReg(W_REGISTER | DYNPD,0x01);
	NRF24L01_WriteReg(W_REGISTER | FEATURE,0x06);

	NRF24L01_CE_HIGH;//rx mode  ?
	delay_us(130);
}

void robot_id_set(void)
{
	Address[4] = (RobotID << 4 ) | RobotID ;
}