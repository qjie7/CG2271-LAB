/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED  										18 		// PortB Pin 18 
#define GREEN_LED 									19 		// PortB Pin 19
#define MASK(x) 										(1 << (x))
#define SW_POS											6  // PORT D Pin 6
#define led_on 											1
#define led_off 										0
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 volatile osSemaphoreId_t mySem;
 volatile osSemaphoreId_t mySemGreen;
 
 static void delay(volatile uint32_t nof) {
	while(nof != 0) {
		__ASM("NOP");
		nof--;
	}
}
 
void initUART2(uint32_t baud_rate){
	
	uint32_t divisor, bus_clock;
	/*
	 * Enable clock gating for serial communication, specifically for UART2 
	 * and enable clock gating for PORT E as well
	 */
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	/*
	 * Ensure Tx and Rx are disabled before configuration.
	 * This is to ensure there are no other codes that can 
	 * accidentally turn on the Tx and Rx before configuration is done
	 */
	UART2->C2 &= ~ UART_C2_RE_MASK;
	
	/*
	 * UART2 has fixed 16x oversampling. This is to improve noise immunity ,
	 * hence better synchronization to incoming data.
	 */
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= (UART_C2_RE_MASK);

	NVIC_SetPriority(UART2_IRQn, 128);  // Initialise of the interrupt must put at last part of the init
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_RIE_MASK;
	UART2->C2 |= UART_C2_RIE_MASK;
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
	  osSemaphoreAcquire(mySem, osWaitForever);
		ledControl(RED_LED, led_on);	
		osDelay(1000);
		ledControl(RED_LED, led_off);
		osDelay(1000);
	
	}
}

void led_green_thread(void *argument) {
 
  // ...
  for (;;) {
		osSemaphoreAcquire(mySemGreen, osWaitForever);
		ledControl(GREEN_LED, led_on);
		osDelay(1000);
		ledControl(GREEN_LED, led_off);
		osDelay(1000);
		
	}
}


void UART2_Receive_Poll(void) {
	if((UART2->S1 & UART_S1_RDRF_MASK)) {
		if (UART2->D == 3) {
			osSemaphoreRelease(mySem);
		} else if(UART2->D == 5) {
			osSemaphoreRelease(mySemGreen);
		}
	}
}

void UART2_IRQHandler(void){
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	UART2_Receive_Poll();
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	
	InitGPIO();
	offRGB();
	initUART2(BAUD_RATE);
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	mySem = osSemaphoreNew(1,0, NULL);
	mySemGreen = osSemaphoreNew(1,0, NULL);
  osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
	osThreadNew(led_green_thread, NULL, NULL);
  osKernelStart();                      // Start thread execution
  for (;;) {}
}

