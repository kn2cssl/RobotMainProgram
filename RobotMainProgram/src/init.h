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
#include <conf_board.h>

void port_init(void);
void spi_init(void);
void nrf_init (void);
void robot_id_set(void);
void evsys_init(void);
void adc_init(void);
extern char Address[_Address_Width];

#endif /* INIT_H_ */