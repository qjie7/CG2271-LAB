#include "MKL25Z4.h"                    // Device header

#define RED_LED  										18 // PortB Pin 18 
#define GREEN_LED 									19 // PortB Pin 19 
#define BLUE_LED 										1 // PortD Pin 1 
#define MASK(x) 										(1 << (x))
#define OFFMASK(x)									(0 << (x))

typedef enum {
	
	Red = RED_LED, Green = GREEN_LED, Blue = BLUE_LED
	
}colour_t;

void led_show(colour_t colour);
void reset_counter (void);
void led_control (int led_counter);


void InitGPIO(void) {  
	// Enable Clock to PORTB and PORTD  
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));    
	
	// Configure MUX settings to make all 3 pins GPIO    
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);    
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);    
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);    
	// Set Data Direction Registers for PortB and PortD  
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));  
	PTD->PDDR |= MASK(BLUE_LED);   
}

unsigned int counter = 0;
unsigned int led_counter = 0;

/* MAIN function */

int main(void) {
	
	SystemCoreClockUpdate();
	InitGPIO();
		
	// Initialise Red LED to display first to prevent all three colour display at the same time at the beginning
	led_show(Red);

	while(1) {
		
		counter++;
		
		if(counter > 0x80000) {		
			reset_counter();
			led_control(led_counter);
		}	
	}
	
}

	void led_show(colour_t colour) {
			if (colour == Red) {  
				PTB->PDOR = MASK(GREEN_LED); // Set Green LED bit to 1 to off it since the LED are active low
				PTD->PDOR = MASK(BLUE_LED);  // Set Blue LED bit to 1 to off it 
				
			} else if (colour == Green) { 
				PTB->PDOR = MASK(RED_LED);
			} else {
				PTB->PDOR |= MASK(GREEN_LED); //0011
				PTD->PDOR = OFFMASK(BLUE_LED); // On Blue LED
			}
	}
	
	void reset_counter (void) {
		counter = 0;
		led_counter++;
		if(led_counter > 2) {
			led_counter = 0;
		}
	}
	
	void led_control (int led_counter) {
		
		switch (led_counter) {
				
				case 0:
					led_show(Red);
					break;
				
				case 1:
					led_show(Green);
					break;
				
				case 2:
					led_show(Blue);
					break;
		}
	}


 
