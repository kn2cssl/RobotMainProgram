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
	delay_ms(11); 
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

void evsys_init(void)
{
  sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_EVSYS);
  //EVSYS.CH3MUX = EVSYS_CHMUX_PORTD_PIN2_gc; If it is needed to trig adc from NRF24L01 IRQ
  EVSYS.CH1MUX = EVSYS_CHMUX_TCD1_OVF_gc;
  EVSYS.CH2MUX = EVSYS_CHMUX_TCE0_OVF_gc;
}

void adc_init(void)
{
	//! ADCA
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	adc_read_configuration(&ADCA, &adc_conf);
	adcch_read_configuration(&ADCA, ADC_CH0, &adcch_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12, ADC_REF_AREFA);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_EVENT_SWEEP, 4, 0);
	adc_enable_internal_input(&adc_conf, ADC_INT_TEMPSENSE);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&ADCA, &adc_conf);
	
	//! MPU temperature
	adcch_set_input(&adcch_conf, ADCCH_POS_TEMPSENSE, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);
	//! Battery voltage
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN3, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCA, ADC_CH1, &adcch_conf);
	//! M2
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN5, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCA, ADC_CH2, &adcch_conf);
	//! M3
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN6, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCA, ADC_CH3, &adcch_conf);
	
	adc_enable(&ADCA);
	
	//! ADCB
	adc_read_configuration(&ADCB, &adc_conf);
	adcch_read_configuration(&ADCB, ADC_CH0, &adcch_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_ON, ADC_RES_12, ADC_REF_AREFA);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_EVENT_SWEEP, 2, 0);
	adc_enable_internal_input(&adc_conf, ADC_INT_TEMPSENSE);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&ADCB, &adc_conf);
	
	//! M0
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN2, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCB, ADC_CH0, &adcch_conf);
	//! M1
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCB, ADC_CH1, &adcch_conf);
	
	adc_enable(&ADCB);
	
}


void tc_init(void)
{
  /* Boost & buck pins:
   * charge : C2
   *(other TC settings should be done according to the type of use )
   */
  tc_enable(&TCC0);
  tc_set_wgm(&TCC0, TC_WG_SS);
  tc_write_clock_source(&TCC0, TC_CLKSEL_DIV64_gc);
  
 
  /** Charge : pin C4 (kick)
	* (other TC settings should be done according to the type of use )
	*/
  tc_enable(&TCC1);
  tc_set_wgm(&TCC1, TC_WG_SS);
  tc_write_clock_source(&TCC1, TC_CLKSEL_DIV64_gc);
  
  //! Clock : 250ms
  tc_enable(&TCD0);
  tc_set_overflow_interrupt_callback(&TCD0, every_250ms);
  tc_set_wgm(&TCD0, TC_WG_NORMAL);
  tc_write_period(&TCD0, 31250);
  tc_set_overflow_interrupt_level(&TCD0, TC_INT_LVL_LO);
  tc_write_clock_source(&TCD0, TC_CLKSEL_DIV256_gc);
  
  //! Boost & buck circuit timer : 1ms
  tc_enable(&TCD1);
  tc_set_wgm(&TCD1, TC_WG_NORMAL);
  tc_write_period(&TCD1, 500);
  tc_write_clock_source(&TCD1, TC_CLKSEL_DIV64_gc);//! Overflow every 1ms
  
  tc_enable(&TCF0);
  tc_set_wgm(&TCF0, TC_WG_NORMAL);
  tc_write_period(&TCF0, 0xFFFF);
  tc_write_clock_source(&TCF0, TC_CLKSEL_EVCH1_gc);//! Frequency of EVENT_CHANEL1 = 1kHz
  
  //! Functions timing
  tc_enable(&TCE0);
  tc_set_wgm(&TCE0, TC_WG_NORMAL);
  tc_write_period(&TCE0, 0xFFFF);
  tc_write_clock_source(&TCE0, TC_CLKSEL_DIV1_gc);
  
  tc_enable(&TCE1);
  tc_set_wgm(&TCE1, TC_WG_NORMAL);
  tc_write_period(&TCE1, 0xFFFF);
  tc_write_clock_source(&TCE1, TC_CLKSEL_EVCH2_gc);
	
}
