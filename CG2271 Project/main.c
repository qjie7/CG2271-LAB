/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include <stdbool.h>

//For DRV8833 #1 
#define PTB0_PIN 										0
#define PTB1_PIN 										1

#define PTB2_PIN										2
#define PTB3_PIN										3

//For DRV8833 #2
#define PTA4_PIN 										4
#define PTA5_PIN 										5

#define PTC8_PIN										8
#define PTC9_PIN										9

#define BAUD_RATE 									9600
#define UART_TX_PORTE22 						22
#define UART_RX_PORTE23 						23
#define UART2_INT_PRIO 							128

#define LOWERCOMMANDLIMIT						0
#define UPPERCOMMANDLIMIT						15
#define DEFAULT_DUTYCYCLE						0.5f

#define MASK(x) 										(1 << (x))
#define UNMASK(x) 										(0 << (x))

//const osThreadAttr_t thread_attr = {
//	.priority = osPriorityNormal1 
//};

void set_speed(float dutycycle, bool update);
void decode_command(int command);

typedef enum {
	
	 FrontLeftWheel = PTA4_PIN , FrontRightWheel = PTB3_PIN, BackLeftWheel = PTC9_PIN, BackRightWheel = PTB0_PIN
	
}wheel_t;

typedef enum {
	
	 MoveForward , MoveBackward, TurnRight, TurnLeft, Stop
	
}movement_t;

osMutexId_t myMutex;

volatile float current_dutycycle = 0.5f;
volatile movement_t current_movement = Stop;
volatile int CnV_value = 0;
volatile int signal = 0;


void initMotorGPIO(int pin, char port, bool state) {
	
	switch (port) {
		case 'A':
	
			PORTA->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTA->PCR[pin] |= PORT_PCR_MUX(1); 
			// Set Data Direction Registers for PortE
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
			// Set Data Direction Registers for PortE
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
			// Set Data Direction Registers for PortE
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



void tMotorControl(movement_t movement, float dutycycle, bool update) {
	
	current_movement = movement;
	set_speed(dutycycle, update);
	
	switch (movement) {
		
		case MoveForward:
			// ForwardLeftWheel
			initMotorGPIO(PTA4_PIN, 'A', false);
			initMotorPWM(PTA5_PIN, 'A');
		
			// ForwardRightWheel
			initMotorGPIO(PTB3_PIN, 'B', false);
			initMotorPWM(PTB2_PIN, 'B');
		
			// BackwardLeftWheel
			initMotorGPIO(PTC8_PIN, 'C', false);
			initMotorPWM(PTC9_PIN, 'C');
		
			// BackwardRightWheel
			initMotorGPIO(PTB1_PIN, 'B', false);
			initMotorPWM(PTB0_PIN, 'B');
			break;
		
		case MoveBackward:
			
			// ForwardLeftWheel
			initMotorGPIO(PTA5_PIN, 'A', false);
			initMotorPWM(PTA4_PIN, 'A');
		
			// ForwardRightWheel
			initMotorGPIO(PTB2_PIN, 'B', false);
			initMotorPWM(PTB3_PIN, 'B');
		
			// BackwardLeftWheel
			initMotorGPIO(PTC9_PIN, 'C', false);
			initMotorPWM(PTC8_PIN, 'C');
		
			// BackwardRightWheel
			initMotorGPIO(PTB0_PIN, 'B', false);
			initMotorPWM(PTB1_PIN, 'B');	
			break;
		
		case TurnLeft:
			
			// ForwardRightWheel
			initMotorGPIO(PTB3_PIN, 'B', false);
			initMotorPWM(PTB2_PIN, 'B');
		
			// BackwardRightWheel
			initMotorGPIO(PTB1_PIN, 'B', false);
			initMotorPWM(PTB0_PIN, 'B');
		
			// ForwardLeftWheel
			initMotorGPIO(PTA5_PIN, 'A', false);
			initMotorPWM(PTA4_PIN, 'A');
		
			// BackwardLeftWheel
			initMotorGPIO(PTC9_PIN, 'C', false);
			initMotorPWM(PTC8_PIN, 'C');
			
			break;
		
		case TurnRight:
			
			// ForwardLeftWheel
			initMotorGPIO(PTA4_PIN, 'A', false);
			initMotorPWM(PTA5_PIN, 'A');
		
					// BackwardLeftWheel
			initMotorGPIO(PTC8_PIN, 'C', false);
			initMotorPWM(PTC9_PIN, 'C');
		
					// ForwardRightWheel
			initMotorGPIO(PTB2_PIN, 'B', false);
			initMotorPWM(PTB3_PIN, 'B');
		
					// BackwardRightWheel
			initMotorGPIO(PTB0_PIN, 'B', false);
			initMotorPWM(PTB1_PIN, 'B');	
			break;
		
		
		
		default:
			//STOP
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

	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);  // Initialise of the interrupt must put at last part of the init
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
	UART2->C2 |= UART_C2_RIE_MASK;
}

void initPWM(void)
{
	//Enable clock to supply power to Port A and Port B and Port C
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
		
	//Configure MUX settings to make PTB0_Pin, PTB1_Pin, PTC8_PIN, PTC9_PIN as PWM


	PORTB->PCR[PTB1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_PIN] |= PORT_PCR_MUX(3);
	


	PORTB->PCR[PTB3_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_PIN] |= PORT_PCR_MUX(3);
	
	PORTA->PCR[PTA4_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA4_PIN] |= PORT_PCR_MUX(3);
	
	PORTC->PCR[PTC8_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC8_PIN] |= PORT_PCR_MUX(3);
	
	
	//Enable clock to supply power to TPM1 and TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
  SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	


	//SIM->SOPT2 --> System Options Register
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; 	//Ensure TPMSRC bit in SOPT2 is cleared before assigning a new value
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);	//Assign 1 to select s MCGFLLCLK clock or MCGPLLCLK/2 as clock source for the TPM clock
	
	TPM0->MOD = 7500;
	TPM0_C4V = 2250;
	TPM0_C5V = 2250;

	TPM1->MOD = 7500;
	TPM1_C0V = 2250;
	TPM1_C1V = 2250;
	
	TPM2->MOD = 7500;
	TPM2_C0V = 2250;
	TPM2_C1V = 2250;

	TPM0->MOD = 7500;
	TPM0_C1V = 2250;
	TPM0_C2V = 2250;
	
	
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	
	
	//CMOD:LPTPM counter increments on every LPTPM counter clock
	//PS:111(7) Divide by 128
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 4 before assign
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 5 before assign
	
	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 4 before assign
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 5 before assign
	
	
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 0 before assign
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 1 before assign
	
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 0 before assign
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 1 before assign
	

	// ELSB:ELSA = 10 , MSB:MSA = 10  --> Edge-aligned PWM, High-true pulses (clear Output on match, set Output on reload)
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM0 Channel 4  
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM0 Channel 5
	
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM0 Channel 4  
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM0 Channel 5

	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM1 Channel 0  
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM1 Channel 1
	
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM2 Channel 0  
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 	// For TPM2 Channel 1
}

void initGPIO(void) {  
	// Enable Clock to PORTB and PORTD  
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK));    
	
	// Configure MUX settings to make all 3 pins GPIO    
	PORTA->PCR[PTA5_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTA->PCR[PTA5_PIN] |= PORT_PCR_MUX(1); 	
	
	PORTB->PCR[PTB0_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[PTB0_PIN] |= PORT_PCR_MUX(1);
 
	PORTB->PCR[PTB2_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTB->PCR[PTB2_PIN] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC9_PIN] &= ~PORT_PCR_MUX_MASK;  
	PORTC->PCR[PTC9_PIN] |= PORT_PCR_MUX(1);   
	
	// Set Data Direction Registers for PortB and PortD  
	PTA->PDDR |= MASK(PTA5_PIN);
	PTB->PDDR |= (MASK(PTB0_PIN) | MASK(PTB2_PIN));  
	PTC->PDDR |= MASK(PTC9_PIN);
	
	PTA->PDOR &= ~MASK(PTA5_PIN);	// Set Port to logic 0
	PTB->PDOR &= ~MASK(PTB0_PIN);
	PTB->PDOR &= ~MASK(PTB2_PIN);
	PTC->PDOR &= ~MASK(PTC9_PIN);
}

 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void motor_control_thread(void *argument) {
 
  // ...
  for (;;) {
		osMutexAcquire(myMutex, osWaitForever);
		
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
//		tMotorControl(MoveBackward, 0.1, true);
//		osDelay(1000);
//		tMotorControl(MoveBackward, 0.3, true);
//		osDelay(1000);
//		tMotorControl(MoveBackward, 0.5, true);
//		osDelay(1000);
//		tMotorControl(MoveBackward, 0.7, true);
//		osDelay(1000);
//		tMotorControl(MoveBackward, 0.9, true);
//		osDelay(1000);
//				tMotorControl(TurnLeft, 0.1, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.3, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.5, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.7, true);
//		osDelay(1000);
//		tMotorControl(TurnLeft, 0.9, true);
//		osDelay(1000);
				tMotorControl(TurnRight, 0.1, true);
		osDelay(1000);
		tMotorControl(TurnRight, 0.3, true);
		osDelay(1000);
		tMotorControl(TurnRight, 0.5, true);
		osDelay(1000);
		tMotorControl(TurnRight, 0.7, true);
		osDelay(1000);
		tMotorControl(TurnRight, 0.9, true);
		osDelay(1000);
		tMotorControl(Stop, 0.9, false);
			osDelay(1000);
		tMotorControl(Stop, 0.9, false);
	
//		decode_command(11);
//		osDelay(1000);
//		osDelay(1000);
//		decode_command(12);
//		osDelay(1000);
//		osDelay(1000);
//		decode_command(13);
//		osDelay(1000);
//		osDelay(1000);
//		decode_command(14);
//		osDelay(1000);
//		osDelay(1000);
//		decode_command(15);
//		osDelay(1000);
//		osDelay(1000);
		
		
		osMutexRelease(myMutex);		
	}
}

void set_speed(float dutycycle, bool update) {
	
	if (update) {
		current_dutycycle = dutycycle;
	} 
	
	CnV_value = (int)(TPM1->MOD * dutycycle);
	
	TPM1_C0V = CnV_value;
	TPM1_C1V = CnV_value;
	TPM0_C4V = CnV_value;
	TPM0_C5V = CnV_value;
	TPM0_C1V = CnV_value;
	TPM0_C2V = CnV_value;
	TPM2_C0V = CnV_value;
	TPM2_C1V = CnV_value;
	
	
}

void decode_command(int command) {
	if (command < LOWERCOMMANDLIMIT || command > UPPERCOMMANDLIMIT) {
		return;
	}
	if (command >= LOWERCOMMANDLIMIT && command < 11) { // dutycycle control
		set_speed(command / 10.0f, true);
	} else if (command == 11) { // move forward
		tMotorControl(MoveForward, current_dutycycle, true);
	} else if (command == 12){ 
		tMotorControl(MoveBackward, current_dutycycle, true);		
	} else if (command == 13){ 
		tMotorControl(TurnLeft, current_dutycycle, true);		
	} else if (command == 14){ 
		tMotorControl(TurnRight, current_dutycycle, true);		
	} else {
		tMotorControl(Stop, current_dutycycle, false);
	}
}

/* UART2 Receive Poll*/
void UART2_Receive_Poll(void) {
		
	if((UART2->S1 & UART_S1_RDRF_MASK)) {
		signal = UART2->D;
		decode_command(UART2->D);
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
	
	
  //osKernelInitialize();                 // Initialize CMSIS-RTOS
	//myMutex = osMutexNew(NULL);
  //osThreadNew(motor_control_thread, NULL, NULL);    // Create application main thread
	
  //osKernelStart();                      // Start thread execution
	initUART2(BAUD_RATE);
	tMotorControl(Stop, current_dutycycle, false);
  for (;;) {}
}
