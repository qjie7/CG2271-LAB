/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED  										18 		// PortB Pin 18 
#define GREEN_LED 									19 		// PortB Pin 19
#define MASK(x) 										(1 << (x))
#define led_on 											1
#define led_off 										0
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 static void delay(volatile uint32_t nof) {
	while(nof != 0) {
		__ASM("NOP");
		nof--;
	}
}
 
 void InitGPIO(void) {  
	// Enable Clock to PORTB and PORTD  
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));    
	
	// Configure MUX settings to make all 3 pins GPIO    
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);    
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);    
   
	// Set Data Direction Registers for PortB and PortD  
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));  
  
}
 
void ledControl(int led, int led_switch) {
	if (led_switch == 1) {
		PTB->PDOR &= ~MASK(led);
	} else {
		PTB->PDOR |= MASK(led);
	}
}

void offRGB(void) {
	ledControl(RED_LED, 0);
	ledControl(GREEN_LED, 0);
}
 

void led_red_thread(void *argument) {
 
  // ...
  for (;;) {
		ledControl(RED_LED, led_on);	
		delay(0x80000);
		ledControl(RED_LED, led_off);
		delay(0x80000);
	}
}

void led_green_thread(void *argument) {
 
  // ...
  for (;;) {
		ledControl(GREEN_LED, led_on);
		delay(0x80000);
		ledControl(GREEN_LED, led_off);
		delay(0x80000);
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	InitGPIO();
	offRGB();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	osThreadNew(led_green_thread, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
