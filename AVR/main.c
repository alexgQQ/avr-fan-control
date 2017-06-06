#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define FCPU (16000000UL)
#define USART_BAUDRATE (38400)
#define BAUD_PRESCALE (8)
#define BAUD_VALUE ((FCPU/(BAUD_PRESCALE*USART_BAUDRATE))-1)

#define BUFF_SIZE 3								// RX and TX buffer size
#define FAN_START 1								// Fan speed at startup					

// Made gloabal variables volatile as they are mostly changed by an ISR

volatile int pulseCPUCount = 0;					// Global count for rotations on CPU fan.
volatile int pulseGPUCount = 0;					// Global count for rotations on GPU fan. 
volatile float hzCPU = 0;						// Fan #1 speed saved by an ISR
volatile float hzGPU = 0;						// Fan #2 speed saved by an ISR
volatile char readyRX = 0;						// Control flag denoting the Atmega is ready to receive serial data
volatile char inByte = 0;						// Temp variable for received byte in interrupt routine
volatile uint8_t buffRX[BUFF_SIZE] = {'\0'}; 	// Temporary serial RX buffer array to store received data
volatile uint8_t buffTX[BUFF_SIZE] = {'\0'};	// Temporary serial TX buffer array to stage sending data
volatile uint8_t buffRX_index = 0;				// Global index to track where the RX buffer is
volatile uint8_t buffTX_index = 0;				// Global index to track where the TX buffer is


/************************************************************************
*							FUNCTIONS
************************************************************************/


void pin_setup();

/*****************************************************
* NAME: 		pin_setup
*
* DESCRIPTION:	Configures pins for inputs and outputs.
*
* INPUTS:		None
*
* OUTPUTS:		None
*
* NOTES:		inputs: PORTD2 and PORTD3
*				outputs: PORTD6 and PORTB3
*/		

void interrupt_setup();

/*****************************************************
* NAME: 		interrupt_setup
*
* DESCRIPTION:	Interrupt setup for rising edge triggers on the two input pins.
*
* INPUTS:		None
*
* OUTPUTS:		None
*
* NOTES:		The interrupts are used to read the fan speeds. A rising edge input on 
*				PORTD2 and PORTD3 will trigger a count. These edges are generated from the 
*				fan's signal line. Another timer will trigger the count to be gathered and
*				reset.
*/		


void timer1_init();

/*****************************************************
* NAME: 		timer1_init
*
* DESCRIPTION:	Initializes Timer 1 on the Atmega328 for 100ms. It is configured in CTC modes with
*				a compare trigger interrupt against OCR0A. The timer is used to measure fan speed.
*
* INPUTS:		None
*
* OUTPUTS:		None
*
* NOTES:		This timer will  trigger an interrupt to process the pulse count read from the fans.
*				This count is used to measure RPMs by a reliable count.
*
*/		

void timer0_init();

/*****************************************************
* NAME: 		timer0_init
*
* DESCRIPTION:	Initializes Timer 0 on the Atmega328 for a 8kHz Fast PWM mode with a interrupt trigger
*				on OCR0A compare.
*
*
* INPUTS:		None
*
* OUTPUTS:		None
*
* NOTES:		This timer will count for 125us by counting to 0xFF and setting OC1A low when it matches
*				OCR0A. OC1A is tied to PORTD6 so the PWM is generated there. 
*
*/		

void timer2_init();	

/*****************************************************
* NAME: 		timer2_init
*
* DESCRIPTION:	Initializes Timer 2 on the Atmega328 for a 8kHz Fast PWM mode with a interrupt trigger
*				on OCR2A compare.
*
*
* INPUTS:		None
*
* OUTPUTS:		None
*
* NOTES:		This timer will count for 125us by counting to 0xFF and setting OC2A low when it matches
*				OCR2A. OC2A is tied to PORTB3 so the PWM is generated there. 
*
*/		

void set_usart();

/*****************************************************
* NAME: 		set_uart
*
* DESCRIPTION:	Initializes USART for asynchronous mode with an 8bit word length at the designated 
*				baudrate.
*
*
* INPUTS:		None
*
* OUTPUTS:		None
*
* NOTES:		Should run first to get communication with terminal going. 
*
*/	

unsigned char USART_Receive_Data();

/*****************************************************
* NAME: 		USART_Receive_Data
*
* DESCRIPTION:	Wazits for the serial data to be ready to read then returns the data held in UDR0.
*
*
* INPUTS:		None
*
* OUTPUTS:		8bits of data received via serial buffer.
*
* NOTES:		Use this function to receive chars from serial connection. 
*
*/	

void USART_Send_chars(uint8_t data);

/*****************************************************
* NAME: 		USART_Send_chars
*
* DESCRIPTION:	Waits for transmission to be ready and loads 8 bits into UDR0 to be sent via serial.
*
*
* INPUTS:		2 Two bytes of data to send.
*
* OUTPUTS:		None
*
* NOTES:		Use this function to send chars to serial connected devices. 
*
*/	

void USART_Send_string(uint8_t *string, uint8_t string_size);

/*****************************************************
* NAME: 		USART_Send_string
*
* DESCRIPTION:	Loads an array into the data buffer to be sent out sequentially. 
*
*
* INPUTS:		A pointer to an array and the size of the array. 
*
* OUTPUTS:		None
*
* NOTES:		Helpfull to send string messages back.
*
*/	

void clear_buffer(uint8_t *buffer, int bufferSize);

/*****************************************************
* NAME: 		clear_buffer
*
* DESCRIPTION:	Fills a given array address with null terminators. 
*
*
* INPUTS:		A pointer to an array and the size of the array. 
*
* OUTPUTS:		None
*
* NOTES:		This will clear the buffers for temp serial data in buffTX and buffRX.
*
*/	


/************************************************************************
*							INTERRUPT ROUTINES
************************************************************************/


ISR (INT0_vect)
{
	pulseCPUCount++;				
}

/*****************************************************
* NAME: 		INT0 Interrupt Routine
*
* DESCRIPTION:	Increments a global count on trigger.
*
* NOTES:		Rotation count for Fan #1. Disabling compromises RPM measurment.
*
*/	

ISR (INT1_vect)					
{
	pulseGPUCount++;				
}

/*****************************************************
* NAME: 		INT1 Interrupt Routine
*
* DESCRIPTION:	Increments a global count on trigger.
*
* NOTES:		Rotation count for Fan #2. Disabling compromises RPM measurment.
*
*/	

ISR (TIMER1_COMPA_vect)	
{           
	EIMSK &= ~(1 << INT0);		// Disable INT interrupts, no pulses should be read now
	EIMSK &= ~(1 << INT1);         

	hzCPU = pulseCPUCount;	
	hzGPU = pulseGPUCount;

	pulseGPUCount = 0;		
	pulseCPUCount = 0;

	EIMSK |= (1 << INT0);		// Enable INT interrupts
	EIMSK |= (1 << INT1);
}

/*****************************************************
* NAME: 		Timer 1 Compare Interrupt Routine
*
* DESCRIPTION:	Process pulse count and save as Hz measurment. Reset pulse count. 
*
* NOTES:		Handles the speed measurments. Disables INT interrupts for better timing.
*
*/	

ISR (USART_RX_vect)
{
	inByte = UDR0;
	buffRX[buffRX_index++] = inByte;
	if(buffRX_index > 1)
	{
		readyRX = 1;
		buffRX_index = 0;
	}
}

/*****************************************************
* NAME: 		USART Receive Interrupt Routine
*
* DESCRIPTION:	Loads received data into a buffer and triggers the main program to update PWM
*				for the specified fan. 
*
* NOTES:		
*
*/	


/************************************************************************
*							MAIN PROGRAM
************************************************************************/

int main()
{
	// Intitializations...
	set_uart();
	pin_setup();
	interrupt_setup();
	timer1_init();
	timer0_init();
	timer2_init();
	sei();

	// Ready to rock
	while (1)
	{
		if(readyRX)
		{
													// Ready to receive data

			if(buffRX[0] == 0x01) 					// Serial control updating Fan #1
			{
				OCR0A = buffRX[1]; 					// Next byte is PWM percentage

				clear_buffer(buffRX, BUFF_SIZE);
				clear_buffer(buffTX, BUFF_SIZE);	// clear buffers

				readyRX = 0;
			}
			else if(buffRX[0] == 0x02)				// Serial control updating Fan #2
			{
				OCR2A = buffRX[1];					// Next byte is PWM percentage

				clear_buffer(buffRX, BUFF_SIZE);
				clear_buffer(buffTX, BUFF_SIZE);	// clear buffers

				readyRX = 0;
			}
		}

		TIMSK1 &= ~(1 << OCIE1A);					// Disable RPM counter to avoid interruption

		USART_Send_chars((char)0x01);
		USART_Send_chars((char)hzCPU);				// Send data packet for Fan #1 speed

		_delay_ms(20);

		USART_Send_chars((char)0x02);
		USART_Send_chars((char)hzGPU);				// Send data packet for Fan #2 speed

		TIMSK1 |= (1 << OCIE1A);					// Enable RPM counter
	}

	return 0;
}

void pin_setup()
{
	DDRD &= ~(1 << DDD2);						// Clear the PD2 pin
	PORTD |= (1 << PORTD2);						// turn On the Pull-up, input now
	DDRD &= ~(1 << DDD3);						// Clear the PD3 pin
	PORTD |= (1 << PORTD3);						// turn On the Pull-up, input now
	DDRD |= (1 << DDD6);						// PD6 is now an output
	DDRB |= (1 << DDB3);						// PB3 is now an output		
}

void interrupt_setup()
{
	EICRA |= (1 << ISC00) | (1 << ISC01);		// Enable rising edge trigger for INT0
	EICRA |= (1 << ISC11) | (1 << ISC10);		// Enable riding edge trigger for INT1
	EIMSK |= (1 << INT0) | (1 << INT1);			// Enabke INT0, INT1 interrupt
}

void timer1_init()              
{					
	OCR1A = 0x0619;								// Compare value
	TCCR1B |= (1 << WGM12);						// Set CTC mode
	TCCR1B |= (1 << CS12) | (1 << CS10);		// Set OCR1A as compare value
	TIMSK1 |= (1 << OCIE1A);					// Enable overflow interrupt
}

void timer0_init()
{
    OCR0A = FAN_START;							// set PWM to predefined start value
    TCCR0A |= (1 << COM0A1);					// set none-inverting mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);		// set fast PWM Mode
    TCCR0B |= (1 << CS01);						// set prescaler to 8 and starts PWM
}

void timer2_init()	
{	
    OCR2A = FAN_START;							// set PWM to predefined start value
    TCCR2A |= (1 << COM2A1);					// set none-inverting mode
    TCCR2A |= (1 << WGM21) | (1 << WGM20);		// set fast PWM Mode
    TCCR2B |= (1 << CS21);						// set prescaler to 8 and starts PWM
}

void set_uart()
{
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);							// 8 bit character size and asynchronous mode
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);			// Enable the transmission rx/tX and rx interrupt
	UCSR0A |= (1 << U2X0);											// Double USART speed in asynch
	UBRR0H = (BAUD_VALUE >> 8);										// Shift Baudrate to fit 4 MSB
	UBRR0L = (uint8_t)BAUD_VALUE;									// Take the 8 LSB for this register
}

unsigned char USART_Receive_Data()
{
	while (!(UCSR0A & (1 << RXC0)))				// Wait for RX to ready
	{
		;
	}
	return UDR0;								// Receive byte
}

void USART_Send_chars( uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0)))				// Wait for TX to ready
		;
	UDR0 = data;								// Load byte

}

void USART_Send_string(uint8_t * string, uint8_t string_size)
{
	uint8_t index = 0;
	for(index = 0; index < string_size; index++)		// Iterate through sending buffer until a \0 is hit
	{
		if(*(string + index) == '\0')
			break;

		while(!(UCSR0A & (1<<UDRE0)))					// Send data until null terminator
			;
		UDR0 = *(string + index);
	}
}

void clear_buffer(uint8_t *buffer, int bufferSize)
{
	int i = 0;
	for(i = 0; i < bufferSize; i++)
	{
		*(buffer + i) = '\0';					// Iterate through a given buffer and set to \0
	}
}
