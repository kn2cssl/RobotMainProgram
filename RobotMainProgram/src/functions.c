// /*
//  * functions.c
//  *
//  * Created: 10/1/2015 10:49:30 AM
//  *  Author: QWA
//  */ 

#include "functions.h"

//! FPGA connection variables
uint8_t send_packet[40];
uint8_t receive_packet[40];
int packet_counter ;
uint16_t timer_h ;
uint16_t timer_l ;
HL temp_data[10];

//! Wireless connection variables
char spi_rx_buf[_Buffer_Size] ;
char spi_tx_buf[_Buffer_Size];
char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};

//! Wireless monitoring
uint8_t signal_strength;

//! System variables
int summer=0;
struct free_wheel_cause free_wheel = {.low_battery = false, .motor_fault = false, .wireless_timeout = false};
uint8_t number_of_sent_packet , number_of_received_packet ;
enum Data_Flow data = new_controller_loop;
struct Robot_Data Robot={.bat_v.full=12.60, .orc_length=0b00010000};

//! ADC variables
float adc_m0, adc_m1, adc_m2, adc_m3, adc_bat, adc_temperature, adc_bandgap;
float adc_m0_offset, adc_m1_offset, adc_m2_offset, adc_m3_offset;
float adc_gain, adc_offset ;
bool current_offset_check = false;
//! Time
uint64_t seconds;

//! boost & buck variables
struct boost_buck_status bbs={.charge_flag=true};

//! Test variables
int test = 0;
float i_model_M0 ;
float i_model_M1 ;
float i_model_M2 ;
float i_model_M3 ;
HL tt[5];
inline void wireless_connection ( void )
{
	uint8_t status_L = NRF24L01_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
	if((status_L & _RX_DR) == _RX_DR)
	{
		ioport_set_value(LED_WHITE, high);
		WIRLESS_TIMEOUT_TIMER = 0;
		wdt_reset();
		//! read payload through SPI,
		NRF24L01_Read_RX_Buf(spi_rx_buf, _Buffer_Size);
		free_wheel.wireless_timeout = false ;
		if(spi_rx_buf[0] == RobotID )
		{
			ioport_set_value(LED_RED, high);
			Robot.RID				= spi_rx_buf[0];
			Robot.Vx_sp.byte[high]  = spi_rx_buf[1];
			Robot.Vx_sp.byte[low]	= spi_rx_buf[2];
			Robot.Vy_sp.byte[high]  = spi_rx_buf[3];
			Robot.Vy_sp.byte[low]	= spi_rx_buf[4];
			Robot.Wr_sp.byte[high]  = spi_rx_buf[5];
			Robot.Wr_sp.byte[low]	= spi_rx_buf[6];
			Robot.Vx.byte[high]		= spi_rx_buf[7];
			Robot.Vx.byte[low]		= spi_rx_buf[8];
			Robot.Vy.byte[high]		= spi_rx_buf[9];
			Robot.Vy.byte[low]		= spi_rx_buf[10];
			Robot.Wr.byte[high]		= spi_rx_buf[11];
			Robot.Wr.byte[low]		= spi_rx_buf[12];
			Robot.alpha.byte[high]  = spi_rx_buf[13];
			Robot.alpha.byte[low]	= spi_rx_buf[14];
			Robot.KICK				= spi_rx_buf[15];
			Robot.CHIP				= spi_rx_buf[16];
			/*Robot.SPIN*/Robot.orc_length		= spi_rx_buf[17];//! test !!!!!!!!!!
			NRF24L01_Write_TX_Buf(spi_tx_buf, _Buffer_Size);
			signal_strength++;
		}
		
	}
	
	
	if ((status_L&_MAX_RT) == _MAX_RT)
	{
		NRF24L01_Flush_TX();
	}
}

// run time : 2276 clk
inline void data_transmission (void)
{
	HL show[11];
	show[0].full =u[0][0]*1000 ;
	show[1].full =u[1][0]*1000 ;
	show[2].full =u[2][0]*1000 ;
	show[3].full =u[3][0]*1000 ;
	
	show[4].full =x[0][0]*1000 ;
	show[5].full =x[1][0]*1000 ;
	show[6].full =x[2][0]*1000 ;
	
	show[7].full =x_OB[0][0]*1000 ;
	show[8].full =x_OB[1][0]*1000 ;
	show[9].full =x_OB[2][0]*1000 ;
	
	show[10].full = cycle_time_us ;

	//! Debug data
	spi_tx_buf[0]  = show[10].byte[high];//
	spi_tx_buf[1]  = show[10].byte[low]; //
	spi_tx_buf[2]  = show[0].byte[high];//
	spi_tx_buf[3]  = show[0].byte[low];	//
	spi_tx_buf[4]  = show[1].byte[high];//
	spi_tx_buf[5]  = show[1].byte[low];	//
	spi_tx_buf[6]  = show[2].byte[high];//
	spi_tx_buf[7]  = show[2].byte[low]; //
	//! Monitoring data
	spi_tx_buf[8]  = show[3].byte[high];//Robot.Vx.byte[high];   //
	spi_tx_buf[9]  = show[3].byte[low]; //Robot.Vx.byte[low];    //
	spi_tx_buf[10] = show[4].byte[high];//Robot.Vy.byte[high];   //
	spi_tx_buf[11] = show[4].byte[low];	//Robot.Vy.byte[low];    //
	spi_tx_buf[12] = show[5].byte[high];//Robot.Wr.byte[high];   //
	spi_tx_buf[13] = show[5].byte[low];	//Robot.Wr.byte[low];    //
	spi_tx_buf[14] = show[6].byte[high];//
	spi_tx_buf[15] = show[6].byte[low]; //
	spi_tx_buf[16] = show[7].byte[high];//
	spi_tx_buf[17] = show[7].byte[low]; //
	spi_tx_buf[18] = show[8].byte[high];//
	spi_tx_buf[19] = show[8].byte[low]; //
	spi_tx_buf[20] = show[9].byte[high];
	spi_tx_buf[21] = show[9].byte[low]; 
	spi_tx_buf[22] = Robot.W0.byte[high];
	spi_tx_buf[23] = Robot.W0.byte[low];
	spi_tx_buf[24] = Robot.W1.byte[high];
	spi_tx_buf[25] = Robot.W1.byte[low];
	spi_tx_buf[26] = Robot.W2.byte[high];
	spi_tx_buf[27] = Robot.W2.byte[low];
	spi_tx_buf[28] = Robot.W3.byte[high];
	spi_tx_buf[29] = Robot.W3.byte[low];
	spi_tx_buf[30] = Robot.batx1000.byte[high];        //.nsp ;
	spi_tx_buf[31] = Robot.batx1000.byte[low];         //.nrp ;
	
}

// run time : 457 clk
inline void data_packing ( void )
{
	HL MAKsumA	;
	HL MAKsumB	;
	
	//Robot.W0_sp.full = 1000;//test !!!!!!!!!!!!!!!!!!!!!!!!!
	//Robot.W1_sp.full = 1000;//test !!!!!!!!!!!!!!!!!!!!!!!!!
	//Robot.W2_sp.full = 1000;//test !!!!!!!!!!!!!!!!!!!!!!!!!
	//Robot.W3_sp.full = 1000;//test !!!!!!!!!!!!!!!!!!!!!!!!!
	
	MAKsumA.full = Robot.W0_sp.byte[high] + Robot.W1_sp.byte[high] + Robot.W2_sp.byte[high] + Robot.W3_sp.byte[high] + Robot.SB_sp
	+ Robot.W0_sp.byte[low ] + Robot.W1_sp.byte[low ] + Robot.W2_sp.byte[low ] + Robot.W3_sp.byte[low ] + Robot.orc_length	;

	MAKsumB.full = Robot.W0_sp.byte[high]*10 + Robot.W1_sp.byte[high]*9 + Robot.W2_sp.byte[high]*8 + Robot.W3_sp.byte[high]*7 + Robot.SB_sp*6
	+ Robot.W0_sp.byte[low ]*5  + Robot.W1_sp.byte[low ]*4 + Robot.W2_sp.byte[low ]*3 + Robot.W3_sp.byte[low ]*2 + Robot.orc_length	;
	//in even cases micro puts data on F0 to F6 and clear data_clk pin (F7) to 0 ,so micro puts '0'+'data' on port F
	//so there is no need for "CLK_PORT.OUTCLR = CLK_PIN ;"
	send_packet[0]  = 0b01010101 ;	//first start sign
	send_packet[2]  = 0b01010101 ;	//second start sign
	
	send_packet[4] = (  ((Robot.W0_sp.byte[high]    & 0x80) >> 7) |
	((Robot.W1_sp.byte[high]	& 0x80) >> 6) |
	((Robot.W2_sp.byte[high]	& 0x80) >> 5) |
	((Robot.W3_sp.byte[high]	& 0x80) >> 4) |
	((Robot.SB_sp           	& 0x80) >> 3) |
	((MAKsumA.byte[high]		& 0x80) >> 2) |
	((MAKsumB.byte[high]		& 0x80) >> 1) ) & 0b01111111;
	
	send_packet[6] = ( ((Robot.W0_sp.byte[low]	    & 0x80) >> 7) |
	((Robot.W1_sp.byte[low]	    & 0x80) >> 6) |
	((Robot.W2_sp.byte[low]	    & 0x80) >> 5) |
	((Robot.W3_sp.byte[low]	    & 0x80) >> 4) |
	((Robot.orc_length  	    & 0x80) >> 3) |
	((MAKsumA.byte[low]		    & 0x80) >> 2) |
	((MAKsumB.byte[low]		    & 0x80) >> 1) ) & 0b01111111;
	
	send_packet[8]  = Robot.W0_sp.byte[high]		& 0b01111111 ;
	send_packet[10] = Robot.W1_sp.byte[high]		& 0b01111111 ;
	send_packet[12] = Robot.W2_sp.byte[high]		& 0b01111111 ;
	send_packet[14] = Robot.W3_sp.byte[high]		& 0b01111111 ;
	send_packet[16] = Robot.SB_sp           		& 0b01111111 ;
	send_packet[18] = MAKsumA.byte[high]			& 0b01111111 ;
	send_packet[20] = MAKsumB.byte[high]			& 0b01111111 ;
	
	send_packet[22] = Robot.W0_sp.byte[low]		    & 0b01111111 ;
	send_packet[24] = Robot.W1_sp.byte[low]		    & 0b01111111 ;
	send_packet[26] = Robot.W2_sp.byte[low]		    & 0b01111111 ;
	send_packet[28] = Robot.W3_sp.byte[low]		    & 0b01111111 ;
	send_packet[30] = Robot.orc_length  		    & 0b01111111 ;
	send_packet[32] = MAKsumA.byte[low]			    & 0b01111111 ;
	send_packet[34] = MAKsumB.byte[low]			    & 0b01111111 ;
}

// run time : 60 clk
inline void fpga_connection ( void )
{
	if (packet_counter % 2 == 0)//sending
	{
		PORTF_OUT = send_packet[packet_counter] ;
	}
	else                       //receiving
	{
		ioport_set_value(FPGA_CLK, high);//CLK_PORT.OUTSET = CLK_PIN ;
		receive_packet[packet_counter] = PORTX_IN ;
	}

	if (packet_counter == 35)
	{
		number_of_sent_packet ++ ;
		data = unpacking_data ;
	}
}


// run time : 386 clk
inline void data_unpacking (void)
{
	//unpacking data from FPGA
	//High bytes
	temp_data[0].byte[high]  = ( receive_packet[9]  & 0b01111111 ) | ( ( receive_packet[5] & 0b00000001 ) << 7 ) ;
	temp_data[1].byte[high]  = ( receive_packet[11] & 0b01111111 ) | ( ( receive_packet[5] & 0b00000010 ) << 6 ) ;
	temp_data[2].byte[high]  = ( receive_packet[13] & 0b01111111 ) | ( ( receive_packet[5] & 0b00000100 ) << 5 ) ;
	temp_data[3].byte[high]  = ( receive_packet[15] & 0b01111111 ) | ( ( receive_packet[5] & 0b00001000 ) << 4 ) ;
	temp_data[4].byte[high]  = ( receive_packet[17] & 0b01111111 ) | ( ( receive_packet[5] & 0b00010000 ) << 3 ) ;
	temp_data[5].byte[high]  = ( receive_packet[19] & 0b01111111 ) | ( ( receive_packet[5] & 0b00100000 ) << 2 ) ;
	temp_data[6].byte[high]  = ( receive_packet[21] & 0b01111111 ) | ( ( receive_packet[5] & 0b01000000 ) << 1 ) ;
	/*	temp_data[7]  = ( receive_packet[21] & 0b01111111 ) ;*/
	//Low bytes
	temp_data[0].byte[low]	 = ( receive_packet[23] & 0b01111111 ) | ( ( receive_packet[7] & 0b00000001 ) << 7 ) ;
	temp_data[1].byte[low]	 = ( receive_packet[25] & 0b01111111 ) | ( ( receive_packet[7] & 0b00000010 ) << 6 ) ;
	temp_data[2].byte[low]	 = ( receive_packet[27] & 0b01111111 ) | ( ( receive_packet[7] & 0b00000100 ) << 5 ) ;
	temp_data[3].byte[low]	 = ( receive_packet[29] & 0b01111111 ) | ( ( receive_packet[7] & 0b00001000 ) << 4 ) ;
	temp_data[4].byte[low]	 = ( receive_packet[31] & 0b01111111 ) | ( ( receive_packet[7] & 0b00010000 ) << 3 ) ;
	temp_data[5].byte[low]	 = ( receive_packet[33] & 0b01111111 ) | ( ( receive_packet[7] & 0b00100000 ) << 2 ) ;
	temp_data[6].byte[low]	 = ( receive_packet[35] & 0b01111111 ) | ( ( receive_packet[7] & 0b01000000 ) << 1 ) ;
	/*	temp_data[15] = ( receive_packet[39] & 0b01111111 ) ;*/
	
	//generating check_sum
	uint16_t MAKsumA ;
	uint16_t MAKsumB;
	
	MAKsumA = temp_data[0].byte[high] + temp_data[1].byte[high] + temp_data[2].byte[high] + temp_data[3].byte[high] + temp_data[4].byte[high] +
	temp_data[0].byte[low ] + temp_data[1].byte[low ] + temp_data[2].byte[low ] + temp_data[3].byte[low ] + temp_data[4].byte[low ]  ;
	
	MAKsumB = temp_data[0].byte[high]*10 + temp_data[1].byte[high]*9 + temp_data[2].byte[high]*8 + temp_data[3].byte[high]*7 + temp_data[4].byte[high]*6 +
	temp_data[0].byte[low ]*5  + temp_data[1].byte[low ]*4 + temp_data[2].byte[low ]*3 + temp_data[3].byte[low ]*2 + temp_data[4].byte[low ]  ;
	//saving checked data
	if( ( MAKsumA == temp_data[5].full ) && ( MAKsumB == temp_data[6].full))
	{
		Robot.W0.full = temp_data[0].full ;
		Robot.W1.full = temp_data[1].full ;
		Robot.W2.full = temp_data[2].full ;
		Robot.W3.full = temp_data[3].full ;
		Robot.SB.full = temp_data[4].full ;
		number_of_received_packet ++ ;
	}
}


inline void free_wheel_function ( void )
{
	if (free_wheel.low_battery || free_wheel.motor_fault || free_wheel.wireless_timeout || !current_offset_check || (Robot.Vx_sp.full == 258 && Robot.Vy_sp.full == 772))
	{
		Robot.W0_sp.byte[high]		= 1;
		Robot.W0_sp.byte[low ]		= 2;
		Robot.W1_sp.byte[high]		= 3;
		Robot.W1_sp.byte[low ]		= 4;
		Robot.W2_sp.byte[high]		= 0;
		Robot.W2_sp.byte[low ]		= 0;
		Robot.W3_sp.byte[high]		= 0;
		Robot.W3_sp.byte[low ]		= 0;
	}
}

//starting counter
inline void Timer_on(void)
{
	TCE0_CNT = 0 ;
	TCE1_CNT = 0 ;
}

//stopping counter and showing time through USART
//running time : about 26400 clk
inline void Timer_show (void)
{
	cycle_time_us = TCE1_CNT * 1e+3 + TCE0_CNT * 2 ;
	cycle_time_s = TCE1_CNT/1000.0 + TCE0_CNT/500000.0 ;
}

inline void read_all_adc(void)
{
	//! To manually trigger adc through event channel (CH0)
	EVSYS.DATA = 0x01;
	EVSYS.STROBE = 0x01;

	adc_temperature  =  adc_get_result(&ADCA, ADC_CH0) - adc_offset ;
	adc_bat          = (adc_get_result(&ADCA, ADC_CH1) - adc_offset) / adc_gain ;
	adc_m2           = (adc_get_result(&ADCA, ADC_CH2) - adc_offset) / adc_gain ;
	adc_m3           = (adc_get_result(&ADCA, ADC_CH3) - adc_offset) / adc_gain ;
	adc_m0           = (adc_get_result(&ADCB, ADC_CH0) - adc_offset) / adc_gain ;
	adc_m1           = (adc_get_result(&ADCB, ADC_CH1) - adc_offset) / adc_gain ;
	adc_bandgap      = (adc_get_result(&ADCB, ADC_CH2) - adc_offset) / adc_gain ;
	adc_clear_interrupt_flag(&ADCA, ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3);
	adc_clear_interrupt_flag(&ADCB, ADC_CH0 | ADC_CH1 | ADC_CH2);
}

inline void battery_voltage_update(void)
{
	//! Low pass filter for eliminating noise and impact of current  drawing of motors and boost circuit
	Robot.bat_v.full = (adc_bat*1.22/.22  - Robot.bat_v.full)*0.001 + Robot.bat_v.full ;// voltage dividing : 1M & 220K
	Robot.batx1000.full =Robot.bat_v.full*1000;
	if (Robot.bat_v.full < 10.5)
	{
 		 ioport_set_value(BUZZER, high);
		 if (Robot.bat_v.full < 10) free_wheel.low_battery = true ;
	}
	else
	{
		ioport_set_value(BUZZER, low);
		free_wheel.low_battery = false ;
	}
}

// PBUG function every_250ms is an interrupting one, maybe i disable interrupts in main loop at some points, Attention!!
inline void every_250ms(void)
{
	seconds++;  
	
	//! Monitoring
    Robot.nsp = number_of_sent_packet;
    Robot.nrp = number_of_received_packet;
    Robot.ss = signal_strength;
	number_of_sent_packet = 0;
	number_of_received_packet = 0;
	signal_strength = 0;	
}


// TODO all periods and duty cycles should be chosen carefully (in boost_buck_manager)
inline void boost_buck_manager(void)
{
	if (!bbs.failure)
	{
		//! calculating charging time
		if (ioport_get_pin_level(CHARGE_LIMIT))
		{
			if (bbs.charge_flag)
			{
				bbs.charge_counter++;
				if (bbs.charge_counter>300)
				{
					bbs.charge_counter = 0 ;
					bbs.charge_flag = false ;
					bbs.charging_time = BOOST_BUCK_TIMER ;
					BOOST_BUCK_TIMER = 0 ;
				}
			}
				
		}
			
		if (!bbs.chip_flag && !bbs.kick_flag && !ioport_get_pin_level(CHARGE_LIMIT))
		{
			CHARGE_PERIOD(376);
			CHARGE_DUTY_CYCLE(365);
			CHARGE_START;
			
			// TODO it may create a delay before kick or chip
			bbs.charge_flag = true;
			if (BOOST_BUCK_TIMER > MAX_CHARGING_TIME)
			{
				bbs.failure = true ;
			}
		}
		else
		{
			CHARGE_STOP;
				
			//! Kick
			if ((Robot.KICK || ioport_get_pin_level(BIG_BUTTON)) && !bbs.kick_flag && !bbs.chip_flag && !bbs.charge_flag)
			{
				KICK_PERIOD(100);
				KICK_DUTY_CYCLE(100);
				KICK_START;
				BOOST_BUCK_TIMER = 0;
				bbs.kick_flag = true;
			}
				
			if (bbs.kick_flag && (BOOST_BUCK_TIMER > KICK_TIME_LIMIT))
			{
				KICK_STOP;
				BOOST_BUCK_TIMER = 0;
				bbs.kick_flag = false;
				bbs.charge_flag = true;
			}
				
			//! Chip
			if (Robot.CHIP && !bbs.kick_flag && !bbs.chip_flag && !bbs.charge_flag)
			{
				CHIP_PERIOD(123);
				CHIP_DUTY_CYCLE(123);
				CHIP_START;
				BOOST_BUCK_TIMER = 0;
				bbs.chip_flag = true;
			}
				
			if (bbs.chip_flag && (BOOST_BUCK_TIMER > CHIP_TIME_LIMIT))
			{
				CHIP_STOP;
				BOOST_BUCK_TIMER = 0;
				bbs.chip_flag = false;
				bbs.charge_flag = true;
			}
		}
	}
	else
	{
		CHARGE_STOP;
		KICK_STOP;
		CHIP_STOP;
		bbs.charge_flag = false;
		if (BOOST_BUCK_TIMER > 1000)
		{
			// TODO turn on the buzzer
			ioport_toggle_pin(BUZZER);
			BOOST_BUCK_TIMER = 0;
		}
		
	}

  
}

// TODO motors_current_check(void) is not completed and is not tested
inline void motors_current_check(void)
{
	if (current_offset_check)
	{
		//currents
		// voltage dividing : 560k & 470k => (560 + 470) / 560 = 103 / 56
		// Current sensor : 185 mV/A output sensitivity
		// 103 / 56 / 0.185 = 9.942084942
		Robot.I0.full = ((adc_m0 - adc_m0_offset)* 9.942084942 - Robot.I0.full)*0.1 + Robot.I0.full;
		Robot.I1.full = ((adc_m1 - adc_m1_offset)* 9.942084942 - Robot.I1.full)*0.1 + Robot.I1.full;
		Robot.I2.full = ((adc_m2 - adc_m2_offset)* 9.942084942 - Robot.I2.full)*0.1 + Robot.I2.full;
		Robot.I3.full = ((adc_m3 - adc_m3_offset)* 9.942084942 - Robot.I3.full)*0.1 + Robot.I3.full;
		// TODO amazing and strange result in comparison with current-sensor's result
		//  i_m=(V-V_emf)/R=(V-?_m/k_n )/R  : another way of calculating current
		// 	i_model_M0 = (float) (u[0][0] - Robot.W0.full*N/ kn) / res ;
		// 	i_model_M1 = (float) (u[1][0] - Robot.W1.full*N/ kn) / res ;
		// 	i_model_M2 = (float) (u[2][0] - Robot.W2.full*N/ kn) / res ;
		// 	i_model_M3 = (float) (u[3][0] - Robot.W3.full*N/ kn) / res ;

		if ( fabs(Robot.I0.full)>0.3) Robot.W0_warning += fabs(Robot.I0.full) * 5;
		else if(Robot.W0_warning) Robot.W0_warning --;
		
		if ( fabs(Robot.I1.full)>0.3) Robot.W1_warning += fabs(Robot.I1.full) * 5;
		else if(Robot.W1_warning) Robot.W1_warning --;
		
		if ( fabs(Robot.I2.full)>0.3) Robot.W2_warning += fabs(Robot.I2.full) * 5;
		else if(Robot.W2_warning) Robot.W2_warning --;
		
		if ( fabs(Robot.I3.full)>0.3) Robot.W3_warning += fabs(Robot.I3.full) * 5;
		else if(Robot.W3_warning) Robot.W3_warning --;
		
		if(Robot.W0_warning > 60000 || Robot.W1_warning > 60000 || Robot.W2_warning > 60000 || Robot.W3_warning > 60000)
		{
			free_wheel.motor_fault = true;
		}
	}
}