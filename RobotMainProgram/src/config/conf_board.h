/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.kn2c.ir/">kn2c</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H


/**
 * Port masks
 */
#define MASK_PORTA 0b10000000
#define MASK_PORTB 0b00000110
#define MASK_PORTC 0b10111110
#define MASK_PORTD 0b00001010
#define MASK_PORTE 0b00000000
#define MASK_PORTF 0b00000000		// untill fpga ok then =>	//11111111
#define MASK_PORTR 0b00

/**
 * Definition of FPGA connection pins
 */
#define FPGA_DATA_IN0    IOPORT_CREATE_PIN(PORTE, 0)
#define FPGA_DATA_IN1    IOPORT_CREATE_PIN(PORTE, 1)
#define FPGA_DATA_IN2    IOPORT_CREATE_PIN(PORTE, 2)
#define FPGA_DATA_IN3    IOPORT_CREATE_PIN(PORTE, 3)
#define FPGA_DATA_IN4    IOPORT_CREATE_PIN(PORTE, 4)
#define FPGA_DATA_IN5    IOPORT_CREATE_PIN(PORTE, 5)
#define FPGA_DATA_IN6    IOPORT_CREATE_PIN(PORTE, 6)
#define FPGA_DATA_IN7    IOPORT_CREATE_PIN(PORTE, 7)		//new added
//#define FPGA_DATA_IN7  IOPORT_CREATE_PIN(PORTE, 0)
#define FPGA_IN_PORT    (FPGA_DATA_IN6 << 6) | (FPGA_DATA_IN5 << 5) | (FPGA_DATA_IN4 << 4) | \
		                (FPGA_DATA_IN3 << 3) | (FPGA_DATA_IN2 << 2) | (FPGA_DATA_IN1 << 1) | \
					    (FPGA_DATA_IN0 << 0)

//Q am i do it right ?						
#define PORTX_IN		( PORTE_IN & 0b01111111 )
//#define PORTX_IN ( ((PORTA.IN & 0b00000110) >> 1) | ((PORTE.IN & 0b11100000) >> 3 ) | ((PORTR.IN & 0b0000011) << 5 ) ) //PORTX=E6,E5,E4,E3,E2,E1,E0

#define FPGA_CLK    IOPORT_CREATE_PIN(PORTF, 7)

#define FPGA_DATA_OUT0    IOPORT_CREATE_PIN(PORTF, 0)
#define FPGA_DATA_OUT1    IOPORT_CREATE_PIN(PORTF, 1)
#define FPGA_DATA_OUT2    IOPORT_CREATE_PIN(PORTF, 2)
#define FPGA_DATA_OUT3    IOPORT_CREATE_PIN(PORTF, 3)
#define FPGA_DATA_OUT4    IOPORT_CREATE_PIN(PORTF, 4)
#define FPGA_DATA_OUT5    IOPORT_CREATE_PIN(PORTF, 5)
#define FPGA_DATA_OUT6    IOPORT_CREATE_PIN(PORTF, 6)
//#define FPGA_DATA_OUT7  IOPORT_CREATE_PIN(PORTA, 1)

/**
 * Ball detector sensor pin
 */
#define KICK_SENSOR    IOPORT_CREATE_PIN(PORTA, 3)

/**
 * Test button
 */
#define BIG_BUTTON     IOPORT_CREATE_PIN(PORTB, 6)			// only button on the board for micro
//#define TINY_BUTTON    IOPORT_CREATE_PIN(PORTC, 7)

/**
 * LEDs
 */
#define LED_GREEN   IOPORT_CREATE_PIN(PORTD, 1)		//LED1
// #define LED_RED     IOPORT_CREATE_PIN(PORTD, 2)		//LED2			//	+ RX
// #define LED_WHITE   IOPORT_CREATE_PIN(PORTD, 3)		//LED1			//	+ TX

/**
 * USART
 */
#define RX    IOPORT_CREATE_PIN(PORTD, 2)	// + LED_RED
#define TX    IOPORT_CREATE_PIN(PORTD, 3)	// + LED_WHITE

/**
 * Buzzer
 */
#define BUZZER    IOPORT_CREATE_PIN(PORTA, 7)

/**
 * SPI for wireless module
 */
#define NRF24L01_SPI        SPIC
#define NRF24L01_PORT       PORTC

#define NRF24L01_IRQ_LINE   IOPORT_CREATE_PIN(PORTD, 0)
#define NRF24L01_CE_LINE    IOPORT_CREATE_PIN(PORTC, 3)
#define NRF24L01_CS_LINE    IOPORT_CREATE_PIN(PORTC, 4)
#define NRF24L01_MOSI_LINE  IOPORT_CREATE_PIN(PORTC, 5)
#define NRF24L01_MISO_LINE  IOPORT_CREATE_PIN(PORTC, 6)
#define NRF24L01_SCK_LINE   IOPORT_CREATE_PIN(PORTC, 7)

/**
 * Buck & Boost
 */
#define CHARGE_PULSE    IOPORT_CREATE_PIN(PORTC, 2)
#define KICK_PULSE      IOPORT_CREATE_PIN(PORTB, 1)
#define CHIP_PULSE      IOPORT_CREATE_PIN(PORTB, 2)
#define CHARGE_AMOUNT	IOPORT_CREATE_PIN(PORTB, 3)		//CAPVOLTAGE
//#define DISCHARGE_LIMIT IOPORT_CREATE_PIN(PORTC, 5)
//#define CHARGE_LIMIT    IOPORT_CREATE_PIN(PORTC, 6)

/**
 * GYRO 
 */
#define GYRO_SDA    IOPORT_CREATE_PIN(PORTC, 0)
#define GYRO_SCL    IOPORT_CREATE_PIN(PORTC, 1)

/**
 * ADC
 */
//#define AREF_A                IOPORT_CREATE_PIN(PORTA, 0)
#define AREF_B                IOPORT_CREATE_PIN(PORTB, 0)

#define BATTERY_ADC           IOPORT_CREATE_PIN(PORTB, 5)
//#define MAIN_CURRENT_ADC      IOPORT_CREATE_PIN(PORTA, 4)
#define BOOST_CURRENT_ADC     IOPORT_CREATE_PIN(PORTA, 5)

#define MOTOR0_CURRENT_ADC    IOPORT_CREATE_PIN(PORTA, 2)
#define MOTOR1_CURRENT_ADC    IOPORT_CREATE_PIN(PORTA, 6)
#define MOTOR2_CURRENT_ADC    IOPORT_CREATE_PIN(PORTB, 7)
#define MOTOR3_CURRENT_ADC    IOPORT_CREATE_PIN(PORTB, 4)
#define SB_MOTOR_CURRENT_ADC  IOPORT_CREATE_PIN(PORTA, 4)

/**
 * ID pins
 */
#define ID0    IOPORT_CREATE_PIN(PORTD, 4)
#define ID1    IOPORT_CREATE_PIN(PORTD, 7)
#define ID2    IOPORT_CREATE_PIN(PORTD, 5)
#define ID3    IOPORT_CREATE_PIN(PORTD, 6)

#define RobotID		0
//#define RobotID ((((PORTB_IN & PIN5_bm) >> PIN5_bp) << 0)|(((PORTA_IN & PIN7_bm) >> PIN7_bp) << 1)|(((PORTB_IN & PIN6_bm) >> PIN6_bp) << 2)|(((PORTB_IN & PIN4_bm) >> PIN4_bp) << 3))


#endif // CONF_BOARD_H
