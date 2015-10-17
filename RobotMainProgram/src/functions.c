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
uint16_t timer ;
HL temp_data[10];

//! Wireless connection variables
char spi_rx_buf[_Buffer_Size] ;
char spi_tx_buf[_Buffer_Size];
char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};


//! System variables
int summer=0;
int wireless_time_out = 0;
int free_wheel=0;
HL number_of_sent_packet ={{0,0}} , number_of_received_packet ;
enum Data_Flow data;
struct Robot_Data Robot={.bat_v.full=12.6, .orc_length=0b00010000};

//! ADC variables
float adc_m0, adc_m1, adc_m2, adc_m3, adc_bat = 12.6 , adc_temperature;

//! Time
uint64_t seconds;

//! boost & buck variables
struct bust_buck_status bbs;

//! Test variables



inline void wireless_connection ( void )
{
	uint8_t status_L = NRF24L01_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
	if((status_L & _RX_DR) == _RX_DR)
	{
		ioport_set_value(LED_WHITE, high);
		wireless_time_out = 0 ;
		wdt_reset();
		//! read payload through SPI,
		NRF24L01_Read_RX_Buf(spi_rx_buf, _Buffer_Size);
		free_wheel=0 ;
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
			data_transmission();
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
	//! put debug data to show arrays
	HL show[5];
	show[0].full = Robot.Vx_sp.full;
	show[1].full = Robot.Vy_sp.full;
	show[2].full = Robot.Wr_sp.full;
	show[3].full = Robot.Vx.full;
	show[4].full = adc_m0 ;
	
	//! Debug data
	spi_tx_buf[0]  = show[0].byte[high];
	spi_tx_buf[1]  = show[0].byte[low];
	spi_tx_buf[2]  = show[1].byte[high];
	spi_tx_buf[3]  = show[1].byte[low];
	spi_tx_buf[4]  = show[2].byte[high];
	spi_tx_buf[5]  = show[2].byte[low]; 
	spi_tx_buf[6]  = show[3].byte[high];
	spi_tx_buf[7]  = show[3].byte[low]; 
	//! Monitoring data
	spi_tx_buf[8]  = Robot.W0.byte[high];
	spi_tx_buf[9]  = Robot.W0.byte[low];
	spi_tx_buf[10] = Robot.W1.byte[high];
	spi_tx_buf[11] = Robot.W1.byte[low];
	spi_tx_buf[12] = Robot.W2.byte[high];
	spi_tx_buf[13] = Robot.W2.byte[low];
	spi_tx_buf[14] = Robot.W3.byte[high];
	spi_tx_buf[15] = Robot.W3.byte[low];
	spi_tx_buf[16] = Robot.I0.byte[high];
	spi_tx_buf[17] = Robot.I0.byte[low];
	spi_tx_buf[18] = Robot.I1.byte[high];
	spi_tx_buf[19] = Robot.I1.byte[low];
	spi_tx_buf[20] = Robot.I2.byte[high];
	spi_tx_buf[21] = Robot.I2.byte[low];
	spi_tx_buf[22] = Robot.I3.byte[high];
	spi_tx_buf[23] = Robot.I3.byte[low];
	spi_tx_buf[24] = Robot.MCU_temperature.byte[high];
	spi_tx_buf[25] = Robot.MCU_temperature.byte[low];
	spi_tx_buf[26] = Robot.bat_v.byte[high];
	spi_tx_buf[27] = Robot.bat_v.byte[low];
	spi_tx_buf[28] = number_of_sent_packet.byte[high] ;
	spi_tx_buf[29] = number_of_sent_packet.byte[low] ;
	spi_tx_buf[30] = number_of_received_packet.byte[high];
	spi_tx_buf[31] = number_of_received_packet.byte[low];
	
	NRF24L01_Write_TX_Buf(spi_tx_buf, _Buffer_Size);
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
		number_of_sent_packet.full ++ ;
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
		number_of_received_packet.full ++ ;
	}
}


inline void free_wheel_function ( void )
{
	if (free_wheel || (Robot.Vx_sp.full == 258 && Robot.Vy_sp.full == 772))
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
	TCE1_CNT = 0 ;
}

//stopping counter and showing time through USART
//running time : about 26400 clk
inline void Timer_show (void)
{
	timer = TCE1_CNT - 17; // 17 clk is for excessive clk counted
}

inline void read_all_adc(void)
{
	//! To manually trigger adc through event channel (CH0)
	EVSYS.DATA = 0x01;
	EVSYS.STROBE = 0x01;
	
/** If statement is removed for decreeing processing time. 
 *  First data will be zero ,but later data will be OK 
 *  because of enough delay for later READ_ADCs
 */
// 	if (adc_get_interrupt_flag(&ADCA, ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3) == (ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3) &&
// 	adc_get_interrupt_flag(&ADCB, ADC_CH0 | ADC_CH1 ) == (ADC_CH0 | ADC_CH1 ))
// 	{
		adc_temperature  = adc_get_result(&ADCA, ADC_CH0);
		adc_bat          = adc_get_result(&ADCA, ADC_CH1) * 0.00924 ;// 0.924 = 3.3/2048.0*100*5.734;
		adc_m2           = adc_get_result(&ADCA, ADC_CH2);
		adc_m3           = adc_get_result(&ADCA, ADC_CH3);
		adc_m0           = adc_get_result(&ADCB, ADC_CH0);
		adc_m1           = adc_get_result(&ADCB, ADC_CH1);
		adc_clear_interrupt_flag(&ADCA, ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3);
		adc_clear_interrupt_flag(&ADCB, ADC_CH0 | ADC_CH1 );
//	}
}

inline void battery_voltage_update(void)
{
	//! Low pass filter for eliminating noise and impact of current  drawing of motors and boost circuit
	Robot.bat_v.full = (adc_bat - Robot.bat_v.full)*0.0001 + Robot.bat_v.full ;
	if (Robot.bat_v.full < 10)
	{
		ioport_set_value(BUZZER, high);
	}
	else
	{
		ioport_set_value(BUZZER, low);
	}
}

inline void every_1s(void)
{
	seconds++;  	
}

void boost_buck_manager(void)
{
  if (!bbs.chip_flag && !bbs.kick_flag && !ioport_get_pin_level(CHARGE_LIMIT))
  {
    CHARGE_PERIOD(0x77);
    CHARGE_DUTY_CYCLE(0x5D);
    CHARGE_START;
  }
  else
  {
    CHARGE_STOP;
    //! Kick    
    if ((Robot.KICK /*|| ioport_get_pin_level(BIG_BUTTON)*/) && !bbs.kick_flag && !bbs.chip_flag)
    {
      KICK_PERIOD(123);
      KICK_DUTY_CYCLE(123);
      KICK_START;
      BOOST_BUCK_TIMER = 0;
      bbs.kick_flag = true;
    }
    
    if (bbs.kick_flag && (BOOST_BUCK_TIMER > KICK_TIME_LIMIT))
    {
      KICK_STOP;
      bbs.kick_flag = false;
    }
    
    //! Chip
    if ((Robot.CHIP ||  ioport_get_pin_level(BIG_BUTTON)) && !bbs.kick_flag && !bbs.chip_flag)
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
      bbs.chip_flag = false;
    }
  }
  
}