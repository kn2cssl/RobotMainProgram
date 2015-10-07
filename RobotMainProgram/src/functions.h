/*
 * functions.h
 *
 * Created: 10/1/2015 10:52:26 AM
 *  Author: QWA
 */ 


#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <asf.h>
#include "nrf24l01.h"
#include "controller.h"

void data_transmission ( void ) ;
void fpga_connection ( void ) ;
void wireless_connection ( void ) ;
void data_packing ( void ) ;
void data_unpacking ( void ) ;
void free_wheel_function ( void ) ;
void Timer_on (void) ;
void Timer_show (void) ;
void read_all_adc(void);

#define high 1
#define	low	 0

enum Data_Flow {new_wireless_data , new_jyro_data , new_controller_loop , packing_data , communication , unpacking_data , other_programs };

typedef union High_Low{
	uint8_t byte[2] ;
	int16_t full ;
} HL;

struct Robot_Data
{
	//! Wireless data
	uint8_t RID;
	HL Vx_sp ;
	HL Vy_sp ;
	HL Wr_sp ;
	HL Vx ;
	HL Vy ;
	HL Wr ;
	HL alpha ;
	uint8_t KICK;
	uint8_t CHIP;
	uint8_t SPIN;
	uint8_t free_wheel ;
	uint8_t ASK;
	
	//! GYRO data
	uint8_t GVxh;
	uint8_t GVxl;
	uint8_t GVyh;
	uint8_t GVyl;
	uint8_t GWh;
	uint8_t GWl;
	
	//! Wheels' speed setpoint
	HL W0_sp	;
	HL W1_sp	;
	HL W2_sp	;
	HL W3_sp	;
	
	//Wheels' speed
	HL W0	;
	HL W1	;
	HL W2	;
	HL W3	;
	
	//! Motors' current
	HL I0;
	HL I1;
	HL I2;
	HL I3;
	
	//! MCU's temperature
	HL MCU_temperature;
	
	//! Battery voltage
	HL bat_v;
	
	//! Spin_back's speed setpoint
	HL SB_sp	;
	
	//! Spin_back's speed
	HL SB	;
	
};

//! FPGA connection variables
extern uint8_t send_packet[40];
extern uint8_t receive_packet[40];
extern int packet_counter ;
extern uint16_t timer ;
extern HL temp_data[10];

//! Wireless connection variables
extern char spi_rx_buf[_Buffer_Size] ;
extern char spi_tx_buf[_Buffer_Size];
extern char Address[_Address_Width];

//! System variables
extern int summer;
extern int wireless_time_out ;
extern int free_wheel;
extern HL number_of_sent_packet  , number_of_received_packet ;
extern enum Data_Flow data;
extern struct Robot_Data Robot;


//! Test variables
extern uint16_t adc_m0, adc_m1, adc_m2, adc_m3, adc_bat, adc_temperature;

#endif /* FUNCTIONS_H_ */