#include <MKL25Z4.h>

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define LED_MASK(x) (x & 0x06) //0b0110 & 0b0011  = 0010
#define BIT0_MASK(x)(x & 0x01)

#define RED_LED  										18 // PortB Pin 18 
#define GREEN_LED 										19 // PortB Pin 19 
#define BLUE_LED 										1  // PortD Pin 1 
#define LED_RED											2  // 0b00000010

#define MASK(x) 										(1 << (x))
#define OFFMASK(x)										(0 << (x)) 

typedef enum {
	
	Red = RED_LED, Green = GREEN_LED, Blue = BLUE_LED
	
}colour_t;

volatile unsigned int counter = 0;
volatile uint8_t rx_data = 3;

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
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
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
	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	NVIC_SetPriority(UART2_IRQn, 128);  // Initialise of the interrupt must put at last part of the init
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
	UART2->C2 |= UART_C2_RIE_MASK;
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
	PTB->PDOR |= MASK(RED_LED); // Set Red LED bit to 1 to off it since the LED are active low
	PTB->PDOR |= MASK(GREEN_LED); // Set Green LED bit to 1 to off it since the LED are active low
	PTD->PDOR |= MASK(BLUE_LED);  // Set Blue LED bit to 1 to off it 
}

void red_show(int on) {
	if (on == 1) {  
		PTB->PDOR = 0;
		PTB->PDOR = MASK(GREEN_LED);
	} else {
		PTB->PDOR |= MASK(RED_LED);
	}
}

/* UART2 Transmit Poll*/
void UART2_Transmit_Poll(uint8_t data){
	while(!(UART2->S1 & UART_S1_TDRE_MASK));
	UART2->D = data;
}

/* UART2 Receive Poll*/
void UART2_Receive_Poll(void) {
	if((UART2->S1 & UART_S1_RDRF_MASK)) {
		rx_data = UART2->D;
		if(LED_MASK(rx_data) == LED_RED) {
			if (BIT0_MASK(rx_data)){
				red_show(1);
			}else {
				red_show(0);
			}
		}
	}
}

void UART2_IRQHandler(void){
	counter++;
	NVIC_ClearPendingIRQ(UART2_IRQn);
	UART2_Receive_Poll();
}

int main (void){
	
	SystemCoreClockUpdate();
	initLED();
	initUART2(BAUD_RATE);
	
}

