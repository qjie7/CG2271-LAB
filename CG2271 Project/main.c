/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"  
#include <stdbool.h>

// For DRV8833 #1 
#define PTB0_PIN 										0
#define PTB1_PIN 										1
#define PTB2_PIN										2
#define PTB3_PIN										3

// For DRV8833 #2
#define PTA4_PIN 										4
#define PTA5_PIN 										5
#define PTC8_PIN										8
#define PTC9_PIN										9

//For Green LED
#define PTA13_PIN 									13
#define PTC12_PIN 									12
#define PTC13_PIN 									13
#define PTC16_PIN 									16
#define PTC17_PIN 									17
#define PTD5_PIN 										5
#define PTD0_PIN 										0
#define PTD2_PIN 										2

// For Red LED
#define PTB8_PIN										8

// For Buzzer
#define PTE30_PIN										30

// For BT06/UART
#define BAUD_RATE 									9600
#define UART_TX_PORTE22 						22
#define UART_RX_PORTE23 						23
#define UART2_INT_PRIO 							128

// Other Varaible Constants
#define LOWERCOMMANDLIMIT						0
#define UPPERCOMMANDLIMIT						15
#define DEFAULT_duty_cycle						0.5f

// Shifting Operation
#define MASK(x) 										(1 << (x))



//const osThreadAttr_t thread_attr = {
//	.priority = osPriorityNormal1 
//};

// Type definition 
typedef enum {
	
	RED_LED, GREEN_LED
	
}LED;

typedef enum {
	
	 FrontLeftWheel = PTA4_PIN , FrontRightWheel = PTB3_PIN, BackLeftWheel = PTC9_PIN, BackRightWheel = PTB0_PIN
	
}Wheel;

typedef enum {
	
	 MoveForward , MoveBackward, TurnRight, TurnLeft, Stop
	
}Movement;

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
	SHORT = Eight,
	MEDIUM = Quarter,
	LONG = Half
}NoteLength;

typedef enum {
	C4 = 262,
	C4U = 277,
	D4D = C4U,
	D4 = 294,
	D4U = 311,
	E4D = C4U,
	E4 = 330,
	F4 = 349,
	F4U = 370,
	G4D = C4U,
	G4 = 392,
	G4U = 415,
	A4D = C4U,
	A4 = 440,
	A4U = 466,
	B4D = C4U,
	B4 = 494,
	C5 = 523
}Note;

// Functions declaration
void initMotorGPIO(int pin, char port, bool state);
void initMotorPWM(int pin, char port);
void initUART2(uint32_t baud_rate);
void initPWM(void);
void initGPIO(void);
void set_speed(float duty_cycle, bool update);
void running_green_led(void);
void all_green_led_on(void);
void all_green_led_off (void);
void double_flash_green_led(void);
void red_led_500ms(void);
void red_led_250ms(void);

void tMotorControl(Movement movement, float duty_cycle, bool update);
void tBrain(int command);
void tLED (void);
void tAudio (void);

// Global Variables
osMutexId_t myMutex;

volatile float current_duty_cycle = 0.5f;
volatile Movement current_movement = Stop;
volatile int CnV_value = 0;
volatile int counter = 0;
volatile int cnv = 0;
volatile int led_on_id = 0;


 static void delay(volatile uint32_t nof) {
	while(nof != 0) {
		__ASM("NOP");
		nof--;
	}
}


void initMotorGPIO(int pin, char port, bool state) {
	
	switch (port) {
		case 'A':
	
			PORTA->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTA->PCR[pin] |= PORT_PCR_MUX(1);
		
			PTA->PDDR |=  MASK(pin);
		
			if (state) {
				PTA->PDOR |= MASK(pin);
			}else {
				PTA->PDOR &= ~MASK(pin);
			}
			break;
			
		case 'B':
		
			PORTB->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTB->PCR[pin] |= PORT_PCR_MUX(1);
		
			PTB->PDDR |=  MASK(pin);
		
			if (state) {
				PTB->PDOR |= MASK(pin);
			}else {
				PTB->PDOR &= ~MASK(pin);
			}
			break;
			
		case 'C':
			
			PORTC->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTC->PCR[pin] |= PORT_PCR_MUX(1);
		
			PTC->PDDR |=  MASK(pin);
		
			if (state) {
				PTC->PDOR |= MASK(pin);
			}else {
				PTC->PDOR &= ~MASK(pin);
			}
			
			break;
	}
}

void initMotorPWM(int pin, char port) {
	
		switch (port) {
			
		case 'A':
		
			PORTA->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTA->PCR[pin] |= PORT_PCR_MUX(3);
		
			TPM0->MOD = 7500;
			TPM0_C1V = 2250;
			TPM0_C2V = 2250;

			break;
		
		case 'B':
		
			PORTB->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTB->PCR[pin] |= PORT_PCR_MUX(3);
		
			TPM1->MOD = 7500;
			TPM1_C0V = 2250;
			TPM1_C1V = 2250;
		
			TPM2->MOD = 7500;
			TPM2_C0V = 2250;
			TPM2_C1V = 2250;
		
			break;
		
		case 'C':
		
			PORTC->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTC->PCR[pin] |= PORT_PCR_MUX(3);
		
			TPM0->MOD = 7500;
			TPM0_C4V = 2250;
			TPM0_C5V = 2250;
			
			break;
	}
}

void set_speed(float duty_cycle, bool update) {
	
	if (update) {
		current_duty_cycle = duty_cycle;
	} 
	
	CnV_value = (int)(TPM1->MOD * duty_cycle);
	
	TPM1_C0V = CnV_value;
	TPM1_C1V = CnV_value;
	TPM0_C4V = CnV_value;
	TPM0_C5V = CnV_value;
	TPM0_C1V = CnV_value;
	TPM0_C2V = CnV_value;
	TPM2_C0V = CnV_value;
	TPM2_C1V = CnV_value;
}

void tMotorControl(Movement movement, float duty_cycle, bool update) {
	
	current_movement = movement;
	set_speed(duty_cycle, update);
	
	switch (movement) {
		
		case MoveForward:
			// FrontLeftWheel (Move Forward)
			initMotorGPIO(PTA4_PIN, 'A', false);
			initMotorPWM(PTA5_PIN, 'A');
		
			// FrontRightWheel (Move Forward)
			initMotorGPIO(PTB3_PIN, 'B', false);
			initMotorPWM(PTB2_PIN, 'B');
		
			// BackLeftWheel (Move Forward)
			initMotorGPIO(PTC8_PIN, 'C', false);
			initMotorPWM(PTC9_PIN, 'C');
		
			// BackRightWheel (Move Forward)
			initMotorGPIO(PTB1_PIN, 'B', false);
			initMotorPWM(PTB0_PIN, 'B');
		
			break;
		
		case MoveBackward:
			
			// FrontLeftWheel (Move Backward)
			initMotorGPIO(PTA5_PIN, 'A', false);
			initMotorPWM(PTA4_PIN, 'A');
		
			// FrontRightWheel (Move Backward)
			initMotorGPIO(PTB2_PIN, 'B', false);
			initMotorPWM(PTB3_PIN, 'B');
		
			// BackLeftWheel (Move Backward)
			initMotorGPIO(PTC9_PIN, 'C', false);
			initMotorPWM(PTC8_PIN, 'C');
		
			// BackRightWheel (Move Backward)
			initMotorGPIO(PTB0_PIN, 'B', false);
			initMotorPWM(PTB1_PIN, 'B');	
		
			break;
		
		case TurnLeft:
			
			// FrontRightWheel (Move Forward)
			initMotorGPIO(PTB3_PIN, 'B', false);
			initMotorPWM(PTB2_PIN, 'B');
		
			// BackRightWheel (Move Forward)
			initMotorGPIO(PTB1_PIN, 'B', false);
			initMotorPWM(PTB0_PIN, 'B');
		
			// FrontLeftWheel (Move Backward)
			initMotorGPIO(PTA5_PIN, 'A', false);
			initMotorPWM(PTA4_PIN, 'A');
		
			// BackLeftWheel (Move Backward)
			initMotorGPIO(PTC9_PIN, 'C', false);
			initMotorPWM(PTC8_PIN, 'C');
			
			break;
		
		case TurnRight:
			
			// FrontLeftWheel (Move Forward)
			initMotorGPIO(PTA4_PIN, 'A', false);
			initMotorPWM(PTA5_PIN, 'A');
		
			// BackLeftWheel (Move Forward)
			initMotorGPIO(PTC8_PIN, 'C', false);
			initMotorPWM(PTC9_PIN, 'C');
		
			// FrontRightWheel (Move Backward)
			initMotorGPIO(PTB2_PIN, 'B', false);
			initMotorPWM(PTB3_PIN, 'B');
		
			// BackRightWheel (Move Backward)
			initMotorGPIO(PTB0_PIN, 'B', false);
			initMotorPWM(PTB1_PIN, 'B');	
		
			break;
		
		default:
			
			// Stop ALL wheels from rotating
			initMotorGPIO(PTA4_PIN, 'A', true);
			initMotorGPIO(PTA5_PIN, 'A', true);
			
			initMotorGPIO(PTB0_PIN, 'B', true);
			initMotorGPIO(PTB1_PIN, 'B', true);
			
			initMotorGPIO(PTB2_PIN, 'B', true);
			initMotorGPIO(PTB3_PIN, 'B', true);
			
			initMotorGPIO(PTC8_PIN, 'C', true);
			initMotorGPIO(PTC9_PIN, 'C', true);
		
			break;
	}
}

void initUART2(uint32_t baud_rate) {
	
	uint32_t divisor, bus_clock;
	
	//Enable clock gating for serial communication, specifically for UART2 and enable clock gating for PORT E as well
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	//PORTE22 is not in use for this project
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	
	// Ensure Tx and Rx are disabled before configuration. This is to ensure there are no other codes that can accidentally turn on the Tx and Rx before configuration is done.
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	
	// UART2 has fixed 16x oversampling. This is to improve noise immunity, hence better synchronization to incoming data. 
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);  // Initialise of the interrupt must put at last part of the init
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
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
	
	PORTB->PCR[PTB1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_PIN] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB3_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_PIN] |= PORT_PCR_MUX(3);
	
	PORTC->PCR[PTC8_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC8_PIN] |= PORT_PCR_MUX(3);
	
	PORTE->PCR[PTE30_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE30_PIN] |= PORT_PCR_MUX(3);
	
	
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
	TPM0_C2V = 0;
	TPM0_C3V = 0;
	TPM0_C4V = 0;
	TPM0_C5V = 0;

	TPM1->MOD = 0;
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	
	TPM2->MOD = 0;
	TPM2_C0V = 0;
	TPM2_C1V = 0;

	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 1 before assign
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 2 before assign
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 3 before assign
	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 4 before assign
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 5 before assign
	
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
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM0 Channel 2
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM0 Channel 3
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM0 Channel 4  
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM0 Channel 5

	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM1 Channel 0  
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM1 Channel 1
	
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM2 Channel 0  
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM2 Channel 1
}

void initGPIO(void) {  
	// Enable Clock to Port A, Port B, Port C and Port D  
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTD_MASK));    
	
	// Configure MUX settings to GPIO    
	PORTA->PCR[PTA5_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTA->PCR[PTA5_PIN] |= PORT_PCR_MUX(1); 	
	
	PORTA->PCR[PTA13_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTA->PCR[PTA13_PIN] |= PORT_PCR_MUX(1); 
	
	PORTB->PCR[PTB0_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[PTB0_PIN] |= PORT_PCR_MUX(1);
 
	PORTB->PCR[PTB2_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[PTB2_PIN] |= PORT_PCR_MUX(1);
	
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
	
	// Set Data Direction Registers for Port A, Port B, Port C and Port D  
	PTA->PDDR |= (MASK(PTA5_PIN) | MASK(PTA13_PIN));
	PTB->PDDR |= (MASK(PTB0_PIN) | MASK(PTB2_PIN) | MASK(PTB8_PIN));    
	PTC->PDDR |= (MASK(PTC9_PIN) | MASK(PTC12_PIN) | MASK(PTC13_PIN) | MASK(PTC16_PIN) | MASK(PTC17_PIN));
	PTD->PDDR |= (MASK(PTD0_PIN) | MASK(PTD2_PIN) | MASK(PTD5_PIN));
	
	PTA->PDOR &= ~MASK(PTA5_PIN);
	PTA->PDOR &= ~MASK(PTA13_PIN);
	PTB->PDOR &= ~MASK(PTB0_PIN);
	PTB->PDOR &= ~MASK(PTB2_PIN);
	PTB->PDOR &= ~MASK(PTB8_PIN);
	PTC->PDOR &= ~MASK(PTC9_PIN);	
	PTC->PDOR &= ~MASK(PTC12_PIN);	
	PTC->PDOR &= ~MASK(PTC13_PIN);
	PTC->PDOR &= ~MASK(PTC16_PIN);
	PTC->PDOR &= ~MASK(PTC17_PIN);
	PTD->PDOR &= ~MASK(PTD0_PIN);
	PTD->PDOR &= ~MASK(PTD2_PIN);
	PTD->PDOR &= ~MASK(PTD5_PIN);
}

 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
//void motor_control_thread(void *argument) {
// 
//  // ...
//  for (;;) {
//		osMutexAcquire(myMutex, osWaitForever);
//		
//		tMotorControl(MoveForward, 0.1, true);
//		osDelay(1000);
//		tMotorControl(MoveForward, 0.3, true);
//		osDelay(1000);
//		tMotorControl(MoveForward, 0.5, true);
//		osDelay(1000);
//		tMotorControl(MoveForward, 0.7, true);
//		osDelay(1000);
//		tMotorControl(MoveForward, 0.9, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.1, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.3, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.5, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.7, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.9, true);
//		osDelay(1000);
//		tMotorControl(TurnRight, 0.1, true);
//		osDelay(1000);
//		tMotorControl(TurnRight, 0.3, true);
//		osDelay(1000);
//		tMotorControl(TurnRight, 0.5, true);
//		osDelay(1000);
//		tMotorControl(TurnRight, 0.7, true);
//		osDelay(1000);
//		tMotorControl(TurnRight, 0.9, true);
//		osDelay(1000);
//		tMotorControl(Stop, 0.9, false);
//			osDelay(1000);
//		tMotorControl(Stop, 0.9, false);
//		
//		
//		osMutexRelease(myMutex);		
//	}
//}



void tBrain(int command) {
	if (command < LOWERCOMMANDLIMIT || command > UPPERCOMMANDLIMIT) {
		//tLED();
		return;
	}
	
	if (command >= LOWERCOMMANDLIMIT && command < 11) { // duty_cycle control
		set_speed(command / 10.0f, true);
		//tLED();

	} else if (command == 11) {
			tMotorControl(MoveForward, current_duty_cycle, true);
			//tLED();

	} else if (command == 12) { 
			tMotorControl(MoveBackward, current_duty_cycle, true);
			//tLED();

	} else if (command == 13) { 
			tMotorControl(TurnRight, current_duty_cycle, true);
			//tLED();
		
	} else if (command == 14) {
			tMotorControl(TurnLeft, current_duty_cycle, true);
			
			//tLED();

	} 
	else {
		tMotorControl(Stop, current_duty_cycle, false);
		//tLED();
	}
}

/*----------------------------------------------------------------------------
 * Red LED and Green LED Code 
 *---------------------------------------------------------------------------*/

void running_green_led(void) {
	if (led_on_id > 7) {
			led_on_id = 0;
	}
	
	switch (led_on_id) {
		case 0:
			PTC->PDOR &= ~MASK(PTC17_PIN);
			PTA->PDOR |= MASK(PTA13_PIN);
			delay(1000000);
			
			break;
		case 1:
			PTA->PDOR &= ~MASK(PTA13_PIN);
			PTC->PDOR |= MASK(PTC12_PIN);
			delay(1000000);
			
			break;
		case 2:
			PTC->PDOR &= ~MASK(PTC12_PIN);
			PTD->PDOR |= MASK(PTD5_PIN);
			delay(1000000);
			break;
		
		case 3:
			PTD->PDOR &= ~MASK(PTD5_PIN);
			PTC->PDOR |= MASK(PTC13_PIN);
			delay(1000000);
			break;
		case 4:
			PTC->PDOR &= ~MASK(PTC13_PIN);
			PTD->PDOR |= MASK(PTD0_PIN);
			delay(1000000);
			break;
		case 5:
			PTD->PDOR &= ~MASK(PTD0_PIN);
			PTC->PDOR |= MASK(PTC16_PIN);
			delay(1000000);
			break;
		case 6:
			PTC->PDOR &= ~MASK(PTC16_PIN);
			PTD->PDOR |= MASK(PTD2_PIN);
			delay(1000000);
			break;
		case 7:
			PTD->PDOR &= ~MASK(PTD2_PIN);
			PTC->PDOR |= MASK(PTC17_PIN);
			delay(1000000);
			break;	
	}
	
	led_on_id++;
	
}

void all_green_led_on (void) {
	
	PTD->PDOR |= MASK(PTD5_PIN);
	PTD->PDOR |= MASK(PTD0_PIN);
	PTD->PDOR |= MASK(PTD2_PIN);
	PTA->PDOR |= MASK(PTA13_PIN);
	PTC->PDOR |= MASK(PTC12_PIN);
	PTC->PDOR |= MASK(PTC13_PIN);
	PTC->PDOR |= MASK(PTC16_PIN);
	PTC->PDOR |= MASK(PTC17_PIN);
}

void all_green_led_off (void) {
	
	PTD->PDOR &= ~MASK(PTD0_PIN);
	PTD->PDOR &= ~MASK(PTD2_PIN);
	PTD->PDOR &= ~MASK(PTD5_PIN);
	PTA->PDOR &= ~MASK(PTA13_PIN);
	PTC->PDOR &= ~MASK(PTC12_PIN);
	PTC->PDOR &= ~MASK(PTC13_PIN);
	PTC->PDOR &= ~MASK(PTC16_PIN);
	PTC->PDOR &= ~MASK(PTC17_PIN);
}



void double_flash_green_led(void) {
	
	all_green_led_on();
	delay(1000000);
	all_green_led_off();
	delay(1000000);
	all_green_led_on();
	delay(1000000);
	all_green_led_off();
}


void red_led_500ms(void) {
	
	PTB->PTOR |= MASK(PTB8_PIN);
	delay(1000000);
}

void red_led_250ms(void) {
	
	PTB->PTOR |= MASK(PTB8_PIN);
	delay(100000000);
}

void tLED (void) {
	
	if (current_movement == Stop) {
		all_green_led_on();
		red_led_250ms();
	} else {
		running_green_led();
		red_led_500ms();
	}
	
}

/*----------------------------------------------------------------------------
 * Buzzer Code 
 *---------------------------------------------------------------------------*/
void set_buzzer_frequency(int frequency) {
	TPM0->MOD = (int)(48000000.0 / (128.0 * frequency));
	TPM0_C3V = TPM0->MOD - 1;
	cnv = TPM0_C3V;
}



void play_note(Note note, NoteLength length, int bpm) {
	set_buzzer_frequency(note);
	osDelay((int)((float)length / MEDIUM * 60000 / bpm)); //250
	//delay(0x100000);
	set_buzzer_frequency(0);
	osDelay(8);
	//delay(0x8000);
}

void play_song(void *argument, Note* notes) {
	
	play_note(C4, MEDIUM, 240);
	counter++;
	play_note(D4, MEDIUM, 240);
	counter++;
	play_note(E4, MEDIUM, 240); 
	counter++;
	play_note(F4, MEDIUM, 240);
	counter++;
	play_note(G4, MEDIUM, 240);
	counter++;
	play_note(A4, MEDIUM, 240);
	counter++;
	play_note(B4, MEDIUM, 240);
}

void tAudio (void) {
}

/* UART2 Receive Poll*/
void UART2_Receive_Poll(void) {
		
	if((UART2->S1 & UART_S1_RDRF_MASK)) {
		tBrain(UART2->D);
	}
}
 
void UART2_IRQHandler(void){
	NVIC_ClearPendingIRQ(UART2_IRQn);
	UART2_Receive_Poll();
}



 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	
	initPWM();
	initGPIO();
//	while(1){
//		running_green_led();
//		red_led_500ms();
//	}

	
//  osKernelInitialize();                 // Initialize CMSIS-RTOS
//	myMutex = osMutexNew(NULL);
//  osThreadNew(play_song, NULL, NULL);    // Create application main thread
//  osKernelStart();                      // Start thread execution
	initUART2(BAUD_RATE);
	tMotorControl(Stop, current_duty_cycle, false);
  for (;;) {}
}
