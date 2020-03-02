#include "MKL25Z4.h"                    // Device header

#define RED_LED  										18 // PortB Pin 18 
#define GREEN_LED 									19 // PortB Pin 19 
#define BLUE_LED 										1  // PortD Pin 1 
#define SW_POS											6  // PORT D Pin 6
#define DEBOUNCE										4

#define MASK(x) 										(1 << (x))
#define OFFMASK(x)									(0 << (x))

#define THRESHOLD 									90000

typedef enum {
	
	Red = RED_LED, Green = GREEN_LED, Blue = BLUE_LED
	
}colour_t;

void initSwitch(void);
void PORTD_IRQHandler (void);
void initLED (void);
void led_show(colour_t colour);
void reset_counter (void);
void led_control (int led_counter);

unsigned int counter = 0;
volatile unsigned int led_counter = 0;
unsigned long lastTime = 0, currTime;

int main(void) {
	SystemCoreClockUpdate();
	initSwitch();
	initLED();
	
	led_show(Red);
	
	while (1) {
		currTime = counter++;
		if (currTime - lastTime > THRESHOLD) {
			lastTime = currTime;
			led_control(led_counter);
		}
	}
	
	
}

void initSwitch(void) {
	
	// enable clock for PortD
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

  /* Select GPIO and enable pull-up resistors and interrupts on falling edges of pin connected to switch*/  
	PORTD->PCR[SW_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0a);
	
	//PORTD->PCR[4] |= 1;
	
	// Set PORT D Switch bit to input  
	PTD->PDDR &= ~MASK(SW_POS);
	
	//Enable Interrupts
	NVIC_SetPriority(PORTD_IRQn, 128);
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
	
}

void initLED(void) {  
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
	
void counter_check (void) {
	led_counter++;
	if(led_counter > 2) {
		led_counter = 0;
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
	

void PORTD_IRQHandler(void) {
	// Clear Pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	// Updating some variable / flag
	if ((PORTD->ISFR & MASK(SW_POS))) {
		counter_check();
	}
	
	//Clear INT Flag
	PORTD->ISFR |= MASK(SW_POS);
}
