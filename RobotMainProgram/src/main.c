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
	pmic_init();     //! Enabling all interrupt levels
 	port_init();     //! Initializing ports
	robot_id_set();  //! Setting robot id
 	spi_init();      //! Initializing spi
 	nrf_init();      //! Initializing NRF24l01+

	sei();
	
	// complete run time : 23102 clk
	while(1)
	{

		if (wireless_time_out > 32000)
		{
			nrf_init () ;
			free_wheel = 1 ;
			wireless_time_out = 0 ;
			data = packing_data ;//for sending free wheel order to fpga
		}
		wireless_time_out ++ ;

		// run time : about 19115 clk
		if (data == new_controller_loop)
		{
			Timer_show();
			Timer_on();
			Vx = Robot.Vx_sp.full / 1000.0 ;
			Vy = Robot.Vy_sp.full / 1000.0 ;
			Wr = Robot.Wr_sp.full / 1000.0 ;
			x[0][0] = Robot.Vx.full/1000.0 ;
			x[1][0] = Robot.Vy.full/1000.0 ;
			x[2][0] = Robot.Wr.full/1000.0 ;
			x[3][0] = Robot.W0.full ;
			x[4][0] = Robot.W1.full ;
			x[5][0] = Robot.W2.full ;
			x[6][0] = Robot.W3.full ;
			
			setpoint_generator() ;
			
			state_feed_back() ;
			
			Robot.W0_sp.full = Robot.Vx_sp.full;//u[0][0] /battery_voltage * max_ocr;
			Robot.W1_sp.full = Robot.Vy_sp.full;//u[1][0] /battery_voltage * max_ocr;
			Robot.W2_sp.full = Robot.Wr_sp.full;//u[2][0] /battery_voltage * max_ocr;
			Robot.W3_sp.full = Robot.alpha.full;//u[3][0] /battery_voltage * max_ocr;
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
			
			// 			Timer_on();
			// 			adc_start_conversion(&ADCA, ADC_CH0);
			// 			adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
			// 			adc_m2 = adc_get_result(&ADCA, ADC_CH0);
			//
			// 			adc_start_conversion(&ADCA, ADC_CH1);
			// 			adc_wait_for_interrupt_flag(&ADCA, ADC_CH1);
			// 			adc_m3 = adc_get_result(&ADCA, ADC_CH1);
			//
			// 			adc_start_conversion(&ADCA, ADC_CH2);
			// 			adc_wait_for_interrupt_flag(&ADCA, ADC_CH2);
			// 			adc_bat = adc_get_result(&ADCA, ADC_CH2)*3.31/2048*100;
			//
			// 			adc_start_conversion(&ADCA, ADC_CH3);
			// 			adc_wait_for_interrupt_flag(&ADCA, ADC_CH3);
			// 			adc_temperature = adc_get_result(&ADCA, ADC_CH3);
			// 			Timer_show();

			if (adc_get_interrupt_flag(&ADCA, ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3) == (ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3))
			{
				adc_m2          = adc_get_result(&ADCA, ADC_CH0);
				adc_m3          = adc_get_result(&ADCA, ADC_CH1);
				adc_bat         = adc_get_result(&ADCA, ADC_CH0)*3.31/2048*100;
				adc_temperature = adc_get_result(&ADCA, ADC_CH1);
				adc_clear_interrupt_flag(&ADCA, ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3);
			}
		}
	}
}


ISR(PORTD_INT0_vect)//PRX   IRQ Interrupt Pin
{
	ioport_toggle_pin(LED_RED);
	wireless_connection();
	data = new_controller_loop;//communication;new_controller_loop ;
	wireless_time_out = 0 ;	
}

