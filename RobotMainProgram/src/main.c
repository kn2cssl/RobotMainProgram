/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "init.h"
#include "nrf24l01.h"
#include "controller.h"
#include "functions.h"

int main (void)
{
	sysclk_init();   //! Initializing system clock
	evsys_init();    //! Initializing Event system
	pmic_init();     //! Enabling all interrupt levels
 	port_init();     //! Initializing ports
	robot_id_set();  //! Setting robot id
 	spi_init();      //! Initializing spi
 	nrf_init();      //! Initializing NRF24l01+
	adc_init();      //! Initializing ADC module
	tc_init();    
	rtc_init();   

	sei();
	
	// complete run time : 23102 clk
	while(1)
	{

		if (WIRLESS_TIMEOUT_TIMER >= 10)
		{
			nrf_init () ;
			free_wheel.wireless_timeout = true ;
			WIRLESS_TIMEOUT_TIMER = 0;
			data = new_controller_loop ;//for sending free wheel order to fpga
			Robot.wrc ++;
		}

		// run time : about 19115 clk
		if (data == new_controller_loop)
		{
			Timer_show();
			Timer_on();
			
			d_time = timer_h/1000.0 + timer_l/500000.0 ;
			
			observer();
			
			state_generator();
			
			setpoint_generator() ;
			
			state_feed_back() ;

			ocr_change();
			
			Robot.W0_sp.full = u[0][0] /Robot.bat_v.full * max_ocr;
			Robot.W1_sp.full = u[1][0] /Robot.bat_v.full * max_ocr;
			Robot.W2_sp.full = u[2][0] /Robot.bat_v.full * max_ocr;
			Robot.W3_sp.full = u[3][0] /Robot.bat_v.full * max_ocr;
			data = packing_data ;
			
		}
		
		//run time : 536 clk
		if (data == packing_data)
		{
			free_wheel_function () ;
			data_packing () ;
			packet_counter = 0 ;
			summer=0;
			data = communication ;
		}
		
		//run time : 84 clk
		if (data == communication)
		{
			fpga_connection () ;
			packet_counter++;
			summer += packet_counter;
		}
		
		// run time : 425 clk
		if (data == unpacking_data)
		{
			data_unpacking () ;
			data = other_programs ;
		}
		
		// run time : 2 clk
		if (data == other_programs)
		{
			ioport_set_value(LED_RED,   low);
			ioport_set_value(LED_WHITE, low);
			
			read_all_adc();
			battery_voltage_update();
      
			boost_buck_manager();
			
			data_transmission();	

			data = new_controller_loop;
		}
	}
}


ISR(PORTD_INT0_vect)//PRX   IRQ Interrupt Pin
{
	wireless_connection();
	data = new_controller_loop;//communication;new_controller_loop ;	
}

