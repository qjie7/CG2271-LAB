#include "myheader.h"

/*----------------------------------------------------------------------------
 * Motor Control Code
 *---------------------------------------------------------------------------*/
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
			
		case 'E':
	
			PORTE->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTE->PCR[pin] |= PORT_PCR_MUX(1);
		
			PTE->PDDR |=  MASK(pin);
		
			if (state) {
				PTE->PDOR |= MASK(pin);
			}else {
				PTE->PDOR &= ~MASK(pin);
			}
			break;
	}
}

void initMotorPWM(int pin, char port) {
	
		switch (port) {
			
		case 'A':
		
			PORTA->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTA->PCR[pin] |= PORT_PCR_MUX(3);
		
			TPM2_C0V = (int)(TPM2->MOD * current_duty_cycle);
			TPM2_C1V = (int)(TPM2->MOD * current_duty_cycle);

			break;
		
		case 'B':
		
			PORTB->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTB->PCR[pin] |= PORT_PCR_MUX(3);
		
			TPM1_C0V = (int)(TPM1->MOD * current_duty_cycle);
			TPM1_C1V = (int)(TPM1->MOD * current_duty_cycle);
		
			TPM2_C0V = (int)(TPM2->MOD * current_duty_cycle);
			TPM2_C1V = (int)(TPM2->MOD * current_duty_cycle);
		
			break;
		
		case 'E':
		
			PORTE->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTE->PCR[pin] |= PORT_PCR_MUX(3);
		
			TPM1_C0V = (int)(TPM1->MOD * current_duty_cycle);
			TPM1_C1V = (int)(TPM1->MOD * current_duty_cycle);
			
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
	TPM2_C0V = CnV_value;
	TPM2_C1V = CnV_value;

}



void tMotorControl(Movement movement, float duty_cycle, bool update) {
	
	current_movement = movement;
	
	
	switch (movement) {
		
		case MoveForward:
			set_speed(duty_cycle, update);
			// FrontLeftWheel (Move Forward)
			initMotorGPIO(PTA1_PIN, 'A', false);
			initMotorPWM(PTA2_PIN, 'A');
		
			// FrontRightWheel (Move Forward)
			initMotorGPIO(PTB1_PIN, 'B', false);
			initMotorPWM(PTB0_PIN, 'B');
		
			// BackLeftWheel (Move Forward)
			initMotorGPIO(PTB2_PIN, 'B', false);
			initMotorPWM(PTB3_PIN, 'B');
		
			// BackRightWheel (Move Forward)
			initMotorGPIO(PTE21_PIN, 'E', false);
			initMotorPWM(PTE20_PIN, 'E');
		
			break;
		
		case MoveBackward:
			set_speed(duty_cycle, update);
			// FrontLeftWheel (Move Backward)
			initMotorGPIO(PTA2_PIN, 'A', false);
			initMotorPWM(PTA1_PIN, 'A');
		
			// FrontRightWheel (Move Backward)
			initMotorGPIO(PTB0_PIN, 'B', false);
			initMotorPWM(PTB1_PIN, 'B');
		
			// BackLeftWheel (Move Backward)
			initMotorGPIO(PTB3_PIN, 'B', false);
			initMotorPWM(PTB2_PIN, 'B');
		
			// BackRightWheel (Move Backward)
			initMotorGPIO(PTE20_PIN, 'E', false);
			initMotorPWM(PTE21_PIN, 'E');	
		
			break;
		
		case TurnLeft:
			 set_speed(duty_cycle, update);
			// FrontRightWheel (Move Forward)
			initMotorGPIO(PTB1_PIN, 'B', false);
			initMotorPWM(PTB0_PIN, 'B');
		
			// BackRightWheel (Move Forward)
			initMotorGPIO(PTE21_PIN, 'E', false);
			initMotorPWM(PTE20_PIN, 'E');
		
			// FrontLeftWheel (Move Backward)
			initMotorGPIO(PTA2_PIN, 'A', false);
			initMotorPWM(PTA1_PIN, 'A');
		
			// BackLeftWheel (Move Backward)
			initMotorGPIO(PTB3_PIN, 'B', false);
			initMotorPWM(PTB2_PIN, 'B');
		
			
			
			break;
		
		case TurnRight:
			
			set_speed(duty_cycle, update);
			// FrontLeftWheel (Move Forward)
			initMotorGPIO(PTA1_PIN, 'A', false);
			initMotorPWM(PTA2_PIN, 'A');
		
			// BackLeftWheel (Move Forward)
			initMotorGPIO(PTB2_PIN, 'B', false);
			initMotorPWM(PTB3_PIN, 'B');
		
			// FrontRightWheel (Move Backward)
			initMotorGPIO(PTB0_PIN, 'B', false);
			initMotorPWM(PTB1_PIN, 'B');
		
			// BackRightWheel (Move Backward)
			initMotorGPIO(PTE20_PIN, 'E', false);
			initMotorPWM(PTE21_PIN, 'E');	
			
			
		
			break;
		
		default:
			
			// Stop ALL wheels from rotating
			initMotorGPIO(PTA1_PIN, 'A', true);
			initMotorGPIO(PTA2_PIN, 'A', true);
			
			initMotorGPIO(PTE20_PIN, 'E', true);
			initMotorGPIO(PTE21_PIN, 'E', true);
			
			initMotorGPIO(PTB0_PIN, 'B', true);
			initMotorGPIO(PTB1_PIN, 'B', true);
			
			initMotorGPIO(PTB2_PIN, 'B', true);
			initMotorGPIO(PTB3_PIN, 'B', true);
		
			break;
	}
}



void tBrain(int command) {
	
	if (command < LOWERCOMMANDLIMIT || command > UPPERCOMMANDLIMIT) {
		
		if (command == CONNECTED) {
			
			isConnected = true;
			
		} else if (command == FINISHED) {
			
			finishedChallenge = true;
			
		} else if (command == STARTED) {
			
			isStarted = true;
		}
		return;
	}
	
	if (command < 11) { // duty_cycle control
		set_speed(command / 10.0f, true);

	} else if (command == 11) {
		
			tMotorControl(MoveForward, current_duty_cycle, true);

	} else if (command == 12) { 
		
			tMotorControl(MoveBackward, current_duty_cycle, true);

	} else if (command == 13) { 
		
			tMotorControl(TurnRight, current_duty_cycle, true);
		
	} else if (command == 14) {
		
			tMotorControl(TurnLeft, current_duty_cycle, true);

	} else {
		
			tMotorControl(Stop, current_duty_cycle, false);
	}
}





/*----------------------------------------------------------------------------
 * Red LED and Green LED Code 
 *---------------------------------------------------------------------------*/

void tLED(void *argument) {
	
	for (;;) {
	
		if (current_movement == Stop) {
			red_led_250ms();
			if (!isConnected) {
				all_green_led_on();
			} else {
				double_flash_green_led();
				isConnected = false;
			}
		} else {
				red_led_500ms();
				all_green_led_off();
			
				if (led_on_id > 7) {
					led_on_id = 0;
				}
		
				switch (led_on_id) {
					
					case 0:
						PTC->PDOR &= ~MASK(PTC17_PIN);
						PTA->PDOR |= MASK(PTA13_PIN);
					
						
						break;
					case 1:
						PTA->PDOR &= ~MASK(PTA13_PIN);
						PTC->PDOR |= MASK(PTC12_PIN);
					
						
						break;
					case 2:
						PTC->PDOR &= ~MASK(PTC12_PIN);
						PTD->PDOR |= MASK(PTD5_PIN);
						
						break;
					
					case 3:
						PTD->PDOR &= ~MASK(PTD5_PIN);
						PTC->PDOR |= MASK(PTC13_PIN);
						
						break;
					case 4:
						PTC->PDOR &= ~MASK(PTC13_PIN);
						PTD->PDOR |= MASK(PTD0_PIN);
						
						break;
					case 5:
						PTD->PDOR &= ~MASK(PTD0_PIN);
						PTC->PDOR |= MASK(PTC16_PIN);
					
						break;
					case 6:
						PTC->PDOR &= ~MASK(PTC16_PIN);
						PTD->PDOR |= MASK(PTD2_PIN);
					
						break;
					case 7:
						PTD->PDOR &= ~MASK(PTD2_PIN);
						PTC->PDOR |= MASK(PTC17_PIN);
				
						break;	
			}		
		
			led_on_id++;		
		}
		osDelay(100);
		
	}
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
	
	all_green_led_off();
	osDelay(200);
	all_green_led_on();
	osDelay(200);
	all_green_led_off();
	osDelay(200);
	all_green_led_on();
	osDelay(200);
}
	
void red_led_500ms(void) {
	
	PTB->PTOR |= MASK(PTB8_PIN);
	osDelay(500);
}

void red_led_250ms(void) {
	
	PTB->PTOR |= MASK(PTB8_PIN);
	osDelay(250);
}

/*----------------------------------------------------------------------------
 * Buzzer Code 
 *---------------------------------------------------------------------------*/
void playnote(Note note, NoteLength length, int bpm) {
	
	if (note != 0) {
		
		TPM0->MOD = (int)(48000000 / (128 * note));
		TPM0_C1V = TPM0->MOD >> 1;
	} else {
		
		TPM0_C1V = 0;
	}
	osDelay((int)((float)length / MEDIUM * 60000 / bpm));
}

void tAudio(void *argument) {
	
	int nofNotes = sizeof(song) / sizeof(song[0]);
	int nofNotes1 = sizeof(finishTones) / sizeof(finishTones[0]);
	int nofNotes2 = sizeof(connectTones) / sizeof(connectTones[0]);
	
	for (;;) {
		
		if (isStarted) {
			for (int i = 0; i < nofNotes && !isConnected && !finishedChallenge; ++i) {
				playnote(song[i], noteLength[i], 240);
				playnote(SILENCE, silenceLength[i], 800);
			}
		}
		if (finishedChallenge) {
			
			for (int i = 0; i < nofNotes1; ++i) {
				playnote(finishTones[i], finishNoteLength[i], 240);
				playnote(SILENCE, finishSilenceLength[i], 240);
			}
			break;
		}
		if (isConnected) {
			
			for (int i = 0; i < nofNotes2; ++i) {
				playnote(connectTones[i], connectNoteLength[i], 240);
				playnote(SILENCE, finishNoteLength[i], 240);
			}
			osDelay(500);
			isConnected = false;
			continue;
		}
		osDelay(500);
	}
	
	
}

/*----------------------------------------------------------------------------
 * ISR and UART Controller
 *---------------------------------------------------------------------------*/
void Serial_ISR(void) {
	
		if((UART2->S1 & UART_S1_RDRF_MASK)) {	
	
			command = UART2->D;
			tBrain(command);
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
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	
	osThreadNew(tLED, NULL, &thread_attr_realtime7);
	
	osThreadNew(tAudio, NULL, &thread_attr_realtime7);
  
  osKernelStart();                      // Start thread execution
	
  for (;;) {}
}

