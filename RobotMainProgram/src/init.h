/*
 * init.h
 *
 * Created: 10/2/2015 3:59:44 PM
 *  Author: QWA
 */ 


#ifndef INIT_H_
#define INIT_H_

#include "nrf24l01.h"
#include <asf.h>
#include "functions.h"

#define CHARGE_PERIOD(_A_) tc_write_period(&TCC0, _A_)
#define KICK_PERIOD(_A_)   tc_write_period(&TCC0, _A_)
#define CHARGE_DUTY_CYCLE(_A_) tc_write_cc(&TCC0, TC_CCC, _A_)
#define KICK_DUTY_CYCLE(_A_)   tc_write_cc(&TCC0, TC_CCD, _A_)
#define CHARGE_START tc_enable_cc_channels(&TCC0,TC_CCCEN)
#define CHARGE_STOP tc_disable_cc_channels(&TCC0,TC_CCCEN)
#define KICK_START   tc_enable_cc_channels(&TCC0,TC_CCDEN)
#define KICK_STOP   tc_disable_cc_channels(&TCC0,TC_CCDEN)
#define CHIP_PERIOD(_A_)   tc_write_period(&TCC1, _A_)
#define CHIP_DUTY_CYCLE(_A_)   tc_write_cc(&TCC1, TC_CCA, _A_)
#define CHIP_START   tc_enable_cc_channels(&TCC1,TC_CCAEN)
#define CHIP_STOP   tc_disable_cc_channels(&TCC1,TC_CCAEN)

void port_init(void);
void spi_init(void);
void nrf_init (void);
void robot_id_set(void);
void evsys_init(void);
void adc_init(void);
void tc_init(void);
extern char Address[_Address_Width];

#endif /* INIT_H_ */