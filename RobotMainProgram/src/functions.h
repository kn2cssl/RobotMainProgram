/*
 * functions.h
 *
 * Created: 10/1/2015 10:52:26 AM
 *  Author: QWA
 */ 


#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <asf.h>
#include "init.h"
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
void battery_voltage_update(void);
void every_250ms(void);
void boost_buck_manager(void);
void motors_current_check(void);

#define high 1
#define	low	 0

#define KICK_TIME_LIMIT 300//! What should it be??
#define CHIP_TIME_LIMIT 300//! What should it be??
#define MAX_CHARGING_TIME 5000//! 5 seconds
#define BOOST_BUCK_TIMER TCF0_CNT
#define WIRLESS_TIMEOUT_TIMER RTC.CNT


enum Data_Flow {new_wireless_data , new_jyro_data , new_controller_loop , packing_data , communication , unpacking_data , other_programs };

typedef union High_Low{
	uint8_t byte[2] ;
	int16_t full ;
} HL;

typedef union Float_High_Low{
	uint8_t byte[2] ;
	float full ;
} FHL;

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
	
	uint8_t orc_length;
	
	//Wheels' speed
	HL W0	;
	HL W1	;
	HL W2	;
	HL W3	;
	
	//motors fault
	uint16_t W0_warning	;
	uint16_t W1_warning	;
	uint16_t W2_warning	;
	uint16_t W3_warning	;
	
	//! Motors' current
	FHL I0;
	FHL I1;
	FHL I2;
	FHL I3;
	
	//! MCU's temperature
	HL MCU_temperature;
	
	//! Battery voltage
	FHL bat_v;
	HL  batx1000;
	
	//! Spin_back's speed setpoint
	int8_t SB_sp	;
	
	//! Spin_back's speed
	HL SB	;
	
	//! wireless signal_strength
	uint8_t ss;
	//! wireless_reset_counter
	uint8_t wrc; 
	
	//! SPARTAN3 & Atxmega64 signal_strength
	//! Number of sent packet from Atxmega64 to SPARTAN3
	uint8_t nsp;
	//! Number of received packet from SPARTAN3 
	uint8_t nrp;
	
	// Charging time of boost circuit
	uint16_t ct;
	
};

//! FPGA connection variables
extern uint8_t send_packet[40];
extern uint8_t receive_packet[40];
extern int packet_counter ;
extern HL temp_data[10];

//! Wireless connection variables
extern char spi_rx_buf[_Buffer_Size] ;
extern char spi_tx_buf[_Buffer_Size];
extern char Address[_Address_Width];

//! System variables
extern int summer;
struct free_wheel_cause
{
	bool wireless_timeout ;
	bool motor_fault ;
	bool low_battery;
};

extern uint8_t number_of_sent_packet  , number_of_received_packet ;
extern enum Data_Flow data;
extern struct Robot_Data Robot;
extern struct free_wheel_cause free_wheel;
//! ADC variables
extern float adc_m0, adc_m1, adc_m2, adc_m3, adc_bat, adc_temperature, adc_bandgap;
extern float adc_m0_offset, adc_m1_offset, adc_m2_offset, adc_m3_offset;
extern float adc_gain, adc_offset;
extern bool current_offset_check ;
//! Time
extern uint64_t seconds;

//! boost & buck variables
struct boost_buck_status
{
  bool failure;
  bool charge_flag;
  bool kick_flag;
  bool chip_flag;
  uint16_t charge_counter;
};

//! Test variables
extern int test ;
extern HL tt[5];
#endif /* FUNCTIONS_H_ */