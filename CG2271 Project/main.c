#include "myheader.h"

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

void motor_control(Movement movement, float duty_cycle, bool update) {
	
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



void serial_decoder(int command) {
	if (command < LOWERCOMMANDLIMIT || command > UPPERCOMMANDLIMIT) {
		return;
	}
	
	if (command >= LOWERCOMMANDLIMIT && command < 11) { // duty_cycle control
		set_speed(command / 10.0f, true);


	} else if (command == 11) {
			motor_control(MoveForward, current_duty_cycle, true);


	} else if (command == 12) { 
			motor_control(MoveBackward, current_duty_cycle, true);


	} else if (command == 13) { 
			motor_control(TurnRight, current_duty_cycle, true);

		
	} else if (command == 14) {
			motor_control(TurnLeft, current_duty_cycle, true);


	} 
	else {
		motor_control(Stop, current_duty_cycle, false);
	
	}
}

/* UART2 Receive Poll*/
void Serial_ISR(void) {
		
	if((UART2->S1 & UART_S1_RDRF_MASK)) {
		serial_decoder(UART2->D);
	}
}
 
void UART2_IRQHandler(void){
	NVIC_ClearPendingIRQ(UART2_IRQn);
	Serial_ISR();
}



 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	
	initPWM();
	initGPIO();
	initUART2(BAUD_RATE);
	motor_control(Stop, current_duty_cycle, false);
	
//  osKernelInitialize();                 // Initialize CMSIS-RTOS
//  osThreadNew(play_song, NULL, NULL);    // Create application main thread
//  osKernelStart();                      // Start thread execution

 // for (;;) {}
}

