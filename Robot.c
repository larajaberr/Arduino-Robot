#include "librobot.h"

void init_button_sensor(void){
	

	DDRD = 0b00000000; // initialising portd as input
	PORTD = 0b10000000; // setting pin7 as input

	
	return;
}

int read_button_sensor(void){
	int pressed ;
	

	if(PIND&(1<<PD7)){ //if pin 7 is pressed

        pressed = 0; // return 0

	}
    else{

        pressed = 1; //otherwise return 1

    }
 

	
	return pressed;
}

float read_knob_sensor(int knob){
	float reading = 0; /* Sensor reading, normalised
						  (i.e., scaled) to be between 
						  zero and one. */

	

	DDRC = 0x00; // initialising portc as input 


	if(knob == 0){ //if knob 0 is moved, then..

		ADMUX |= (0<<MUX0); // ..ADC0 is activated
		

	}
	else if(knob == 1){ // if knob 1 is activated,then..

		ADMUX |= (1<<MUX0); //.. ADC1 is activated
		

	}


	ADCSRA |= (1<<ADSC); //sinlge cnversion starts

	while( ADCSRA & (1<<ADSC)); //create a busy wait until the value is measured

	reading = (float)ADCH/(155); //mapping ADCH value between 0 and 1

	ADMUX &=~ (1<<MUX0); //clear the control registers so we can store ADMUX values


	
	return reading;
}


void init_arm_motors(void) {


	int dutycycle;

	//dutycycle = 2000; //here you can adjust the duty cycle by uncommenting/commenting as you please
	//dutycycle = 3000;
	dutycycle = 4000;

	DDRB = 0b00000110; // initialising pins 1 and 2 as outputs
	PORTB = 0b00000000; 
	DDRD = 0xFF; // setting pin d as inputs


	TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0); //timer control registers which i configured the operation mode of PWM to clear OC1A and OC1B on compare match

	TCCR1A |= (1<<WGM11); // this line and the next sets the wave generation mode to Fast PWM where the top value is ICR1 which i have set to 40,000
	TCCR1B = (1<<WGM12) | (1<<WGM13);

	TCCR1B |=  (1<<CS11); // this sets the clock and prescaler to 8 
	TCNT1 = 0;

	ICR1 = 40000; // i have chosen the value 40,000 because i calulated it, calculations shown right at the bottom
	OCR1A = dutycycle;
	OCR1B = dutycycle;

	//duty cycle calculations
	//F_CPU/8 = 1600000/8 = 2x10^6
	//TOP = 2x10^6 x 20*10^(-3) = 40000 
	//T = 20ms/40000=0.5u
	//angle of 90 degrees = 1.5ms/0.5us = 3000
	//angle of 180 degrees = 2ms/0.5us = 4000
	//angle of 0 degrees = 1ms/0.5us = 2000



	return;
}
