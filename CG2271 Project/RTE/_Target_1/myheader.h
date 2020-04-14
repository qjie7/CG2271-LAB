#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"  
#include <stdbool.h>

// For DRV8833 #1 
#define PTE20_PIN 										20    // tpm1 chn0
#define PTE21_PIN 										21 	  // tpm1 chn1
#define PTB0_PIN											0	    // tpm1 chn0
#define PTB1_PIN											1 	  // tpm1 chn1

// For DRV8833 #2
#define PTA1_PIN 											1     // tpm2 chn0
#define PTA2_PIN 											2		  // tpm2 chn1
#define PTB2_PIN											2     // tpm2 chn0
#define PTB3_PIN											3     // tpm2 chn1

// For Green LED
#define PTA13_PIN 										13
#define PTC12_PIN 										12
#define PTC13_PIN 										13
#define PTC16_PIN 										16
#define PTC17_PIN 										17
#define PTD5_PIN 											5
#define PTD0_PIN 											0
#define PTD2_PIN 											2

// For Red LED
#define PTB8_PIN											8

// For Buzzer 
#define PTA4_PIN											4			 // tpm0 chn1


// For BT06/UART
#define BAUD_RATE 										9600
#define UART_TX_PORTE22 							22
#define UART_RX_PORTE23 							23
#define UART2_INT_PRIO 								128

// Other Varaible Constants
#define LOWERCOMMANDLIMIT							0
#define UPPERCOMMANDLIMIT							15
#define CONNECTED											100
#define FINISHED											200
#define STARTED												255
#define MSG_COUNT											1


// Shifting Operation
#define MASK(x) 											(1 << (x))

// Set up Thread Priority 
const osThreadAttr_t thread_attr_realtime7 = {
	.priority = osPriorityRealtime7
};

typedef enum {
	
	 MoveForward , MoveBackward, TurnRight, TurnLeft, Stop
	
}Movement;

// Type definition 
typedef enum {
	
	C6 = 1047,
	C6U = 1109,
	D6D = C6U,
	D6 = 1175,
	D6U = 1245,
	E6D = D6U,
	E6 = 1319,
	F6 = 1397,
	F6U = 1480,
	G6D = F6U,
	G6 = 1568,
	G6U = 1661,
	A6D = G6U,
	A6 = 1760,
	A6U = 1865,
	B6D = A6U,
	B6 = 1976,
	
	SILENCE = 0
	
}Note;

typedef enum {
	Whole = 64,
	Half = 32,
	Quarter = 16,
	Eight = 8,
	Sixteenth = 4,
	ThirtySecond = 2,
	SixtyFourth = 1,
	DoubleWhole = 128
}StandardNoteLength;

typedef enum {
	VERYSHORT = Sixteenth,
	SHORT = Eight,
	MEDIUM = Quarter,
	LONG = Half,
	VERYLONG = Whole
}NoteLength;

Note song[] = {F6U, G6U, B6, B6, B6, B6, B6, B6, B6, F6U, G6U, B6, B6, B6, B6, B6, B6, B6, F6U, G6U, B6, B6, B6, B6, B6, B6, B6, B6, B6, B6D};
	
NoteLength noteLength[] = {	 	MEDIUM, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT,
															MEDIUM, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT,
															MEDIUM, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, MEDIUM,
															SHORT, SHORT, LONG};

NoteLength silenceLength[] ={ LONG, LONG, MEDIUM, MEDIUM, MEDIUM, MEDIUM, VERYSHORT, VERYSHORT, SHORT,
															LONG, LONG, MEDIUM, MEDIUM, MEDIUM, MEDIUM, VERYSHORT, VERYSHORT, SHORT,
															LONG, LONG, MEDIUM, MEDIUM, MEDIUM, MEDIUM, VERYSHORT, VERYSHORT, SHORT,
															MEDIUM, MEDIUM, MEDIUM};

Note connectTones[] = {C6, D6, E6, F6, G6, A6, B6};
NoteLength connectNoteLength[] = {SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT};
NoteLength connectSilenceLength[] = {SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT};

Note finishTones[] = {B6, A6, G6, F6, E6, D6, C6};
NoteLength finishNoteLength[] = {SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT};
NoteLength finishSilenceLength[] = {SHORT, SHORT, SHORT, SHORT, SHORT, SHORT, SHORT};

// Functions declaration
void initUART2(uint32_t baud_rate);
void initPWM(void);
void initGPIO(void);

void initMotorGPIO(int pin, char port, bool state);
void initMotorPWM(int pin, char port);

void set_speed(float duty_cycle, bool update);
void all_green_led_on(void);
void all_green_led_off(void);
void double_flash_green_led(void);
void red_led_500ms(void);
void red_led_250ms(void);


void tMotorControl(Movement movement, float duty_cycle, bool update);
void tBrain(int command);
void Serial_ISR(void);
void tLED(void *argument);
void tAudio(void *argument);

// Global Variables
volatile float current_duty_cycle = 0.5f;
volatile Movement current_movement = Stop;
volatile int CnV_value = 0;
volatile int led_on_id = 0;
volatile bool isConnected = false;
volatile int command;
volatile bool isStarted = false;
volatile bool finishedChallenge = false;


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

void initPWM(void) {
	
	// Enable clock to supply power to Port A,Port B,Port C anbd Port E
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
		
	// Configure MUX settings to make PTA4_Pin, PTB1_Pin, PTB3_PIN, PTC8_PIN, PTE30_PIN as PWM mode
	PORTA->PCR[PTA4_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA4_PIN] |= PORT_PCR_MUX(3);
		
	PORTE->PCR[PTE21_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE21_PIN] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_PIN] |= PORT_PCR_MUX(3);
	
	PORTA->PCR[PTA1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA1_PIN] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB2_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_PIN] |= PORT_PCR_MUX(3);
	
	
	// Enable clock to supply power to TPM0, TPM1 and TPM2
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
  SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	

	// SIM->SOPT2 --> System Options Register
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; 	//Ensure TPMSRC bit in SOPT2 is cleared before assigning a new value
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);	//Assign 1 to select s MCGFLLCLK clock or MCGPLLCLK/2 as clock source for the TPM clock

	// Set all TPM values to 0 to prevent the motor from running at the very beginning after running the program
	TPM0->MOD = 0;
	TPM0_C1V = 0;

	TPM1->MOD = 7500;
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	
	TPM2->MOD = 7500;
	TPM2_C0V = 0;
	TPM2_C1V = 0;

	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 1 before assign
	
	//CMOD:LPTPM counter increments on every LPTPM counter clock
	//PS:111(7) Divide by 128
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 0 before assign
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 1 before assign
	
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 0 before assign
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 1 before assign
	

	// ELSB:ELSA = 10 , MSB:MSA = 10  --> Edge-aligned PWM, High-true pulses (clear Output on match, set Output on reload)
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM0 Channel 1  

	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM1 Channel 0  
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM1 Channel 1
	
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM2 Channel 0  
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM2 Channel 1
}

void initGPIO(void) {  
	// Enable Clock to Port A, Port B, Port C and Port D  
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));    
	
	PORTE->PCR[PTE20_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE20_PIN] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB0_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB0_PIN] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA2_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA2_PIN] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA13_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA13_PIN] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB3_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB3_PIN] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB8_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[PTB8_PIN] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC12_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTC->PCR[PTC12_PIN] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC13_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTC->PCR[PTC13_PIN] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC16_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTC->PCR[PTC16_PIN] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC17_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTC->PCR[PTC17_PIN] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[PTD0_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTD->PCR[PTD0_PIN] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[PTD2_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTD->PCR[PTD2_PIN] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[PTD5_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTD->PCR[PTD5_PIN] |= PORT_PCR_MUX(1);
	

	PTA->PDDR |= (MASK(PTA2_PIN) | MASK(PTA13_PIN));
	PTB->PDDR |= (MASK(PTB1_PIN) | MASK(PTB3_PIN) | MASK(PTB8_PIN));    
	PTC->PDDR |= (MASK(PTC12_PIN) | MASK(PTC13_PIN) | MASK(PTC16_PIN) | MASK(PTC17_PIN));
	PTD->PDDR |= (MASK(PTD0_PIN) | MASK(PTD2_PIN) | MASK(PTD5_PIN));

	PTA->PDOR &= ~MASK(PTA2_PIN);
	PTA->PDOR &= ~MASK(PTA13_PIN);
	PTB->PDOR &= ~MASK(PTB1_PIN);
	PTB->PDOR &= ~MASK(PTB3_PIN);
	PTB->PDOR &= ~MASK(PTB8_PIN);
	PTC->PDOR &= ~MASK(PTC12_PIN);	
	PTC->PDOR &= ~MASK(PTC13_PIN);
	PTC->PDOR &= ~MASK(PTC16_PIN);
	PTC->PDOR &= ~MASK(PTC17_PIN);
	PTD->PDOR &= ~MASK(PTD0_PIN);
	PTD->PDOR &= ~MASK(PTD2_PIN);
	PTD->PDOR &= ~MASK(PTD5_PIN);

}
