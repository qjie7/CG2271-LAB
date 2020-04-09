#include "myheader.h"

//volatile osSemaphoreId_t mySemMoveForward;
//volatile osSemaphoreId_t mySemMoveBackward;
//osMutexId_t myMutex;

osMessageQueueId_t runningLedMsg, allGreenLedOnMsg, doubleFlashMsg;

//osThreadId_t tMotorControl_Id;

myDataPkt myData;

volatile bool isConnected = false;

volatile int command;

volatile int debug = 0;

volatile int debug2 = 0;
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
		
			TPM0->MOD = 7500;
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
		
		case 'E':
		
			PORTE->PCR[pin] &= ~PORT_PCR_MUX_MASK;  
			PORTE->PCR[pin] |= PORT_PCR_MUX(3);
		
			TPM0->MOD = 7500;
			TPM0_C3V = 2250;
			
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
			initMotorGPIO(PTE30_PIN, 'E', false);
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
			initMotorPWM(PTE30_PIN, 'E');
		
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
			initMotorPWM(PTE30_PIN, 'E');
		
			// BackLeftWheel (Move Backward)
			initMotorGPIO(PTC9_PIN, 'C', false);
			initMotorPWM(PTC8_PIN, 'C');
			
			break;
		
		case TurnRight:
			
			// FrontLeftWheel (Move Forward)
			initMotorGPIO(PTE30_PIN, 'E', false);
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
			initMotorGPIO(PTE30_PIN, 'E', true);
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
		
		if (command == 100) {
			myData.cmd = 0x03;
			myData.data = 0x03;
		} else {
			myData.cmd = 0x02;
			myData.data = 0x02;
		}
		return;
	}
	
	if (command >= LOWERCOMMANDLIMIT && command < 11) { // duty_cycle control
		myData.cmd = 0x02;
		myData.data = 0x02;
		set_speed(command / 10.0f, true);

	} else if (command == 11) {
			debug++;
			myData.cmd = 0x01;
			myData.data = 0x01;
			tMotorControl(MoveForward, current_duty_cycle, true);


	} else if (command == 12) { 
			myData.cmd = 0x01;
			myData.data = 0x01;
			tMotorControl(MoveBackward, current_duty_cycle, true);

	} else if (command == 13) { 
			myData.cmd = 0x01;
			myData.data = 0x01;
			tMotorControl(TurnRight, current_duty_cycle, true);

		
	} else if (command == 14) {
			myData.cmd = 0x01;
			myData.data = 0x01;
			tMotorControl(TurnLeft, current_duty_cycle, true);


	} else {
			myData.cmd = 0x02;
			myData.data = 0x02;
			tMotorControl(Stop, current_duty_cycle, false);
	
	}
}

/* UART2 Receive Poll*/
void Serial_ISR(void) {
	
		if((UART2->S1 & UART_S1_RDRF_MASK)) {	
	
			command = UART2->D;			
		}
		
}
 
void UART2_IRQHandler(void){
	NVIC_ClearPendingIRQ(UART2_IRQn);
	Serial_ISR();
}


void tBrain (void *argument) {
	
	for(;;) {
		serial_decoder(command);
	}

}

/*----------------------------------------------------------------------------
 * Red LED and Green LED Code 
 *---------------------------------------------------------------------------*/

void running_green_led(void *argument) {
	
	myDataPkt myRxData;
	
	for (;;) {
		
		osMessageQueueGet(runningLedMsg, &myRxData, NULL, osWaitForever);
		debug2++;
		
		if (myRxData.cmd == 0x01 && myRxData.data == 0x01) {
			
			red_led_500ms();
			all_green_led_off();
			
			if (led_on_id > 7) {
				led_on_id = 0;
			}
	
			switch (led_on_id) {
				case 0:
					PTC->PDOR &= ~MASK(PTC17_PIN);
					PTA->PDOR |= MASK(PTA13_PIN);
					//osDelay(1);
					
					break;
				case 1:
					PTA->PDOR &= ~MASK(PTA13_PIN);
					PTC->PDOR |= MASK(PTC12_PIN);
					//osDelay(1);
					
					break;
				case 2:
					PTC->PDOR &= ~MASK(PTC12_PIN);
					PTD->PDOR |= MASK(PTD5_PIN);
					//osDelay(1);
					break;
				
				case 3:
					PTD->PDOR &= ~MASK(PTD5_PIN);
					PTC->PDOR |= MASK(PTC13_PIN);
					//osDelay(1);
					break;
				case 4:
					PTC->PDOR &= ~MASK(PTC13_PIN);
					PTD->PDOR |= MASK(PTD0_PIN);
					//osDelay(1);
					break;
				case 5:
					PTD->PDOR &= ~MASK(PTD0_PIN);
					PTC->PDOR |= MASK(PTC16_PIN);
					//osDelay(1);
					break;
				case 6:
					PTC->PDOR &= ~MASK(PTC16_PIN);
					PTD->PDOR |= MASK(PTD2_PIN);
					//osDelay(1);
					break;
				case 7:
					PTD->PDOR &= ~MASK(PTD2_PIN);
					PTC->PDOR |= MASK(PTC17_PIN);
					//osDelay(1);
					break;	
			}
			
		
			led_on_id++;
			
		}
		
	}

}

void green_led_control (void *arguement) {
	
	myDataPkt myRxData;
	
	for (;;) {
		
		osMessageQueueGet(runningLedMsg, &myRxData, NULL, osWaitForever);
		debug2++;
		if (myRxData.cmd == 0x02 && myRxData.data == 0x02) {
			red_led_250ms();
			all_green_led_on();
		} 
		else if (myRxData.cmd == 0x03 && myRxData.data == 0x03 && isConnected == false ) {
			
			isConnected = true;
			double_flash_green_led();
			
		}
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

void tLED (void *argument) {
	
	
	for(;;) {
		
		osMessageQueuePut(runningLedMsg, &myData, NULL, 0);
		osDelay(100);
		osMessageQueuePut(allGreenLedOnMsg, &myData, NULL, 0);
		osDelay(100);
	
	}
	
}

 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	
	
	initPWM();
	initGPIO();
	initUART2(BAUD_RATE);
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	
	runningLedMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	allGreenLedOnMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	
	osThreadNew(tBrain, NULL, &thread_attr_realtime7);
	
	osThreadNew(tLED, NULL, &thread_attr_realtime7);
	
	osThreadNew(green_led_control, NULL, &thread_attr_realtime7);
	
	osThreadNew(running_green_led, NULL, &thread_attr_realtime7);
  
  osKernelStart();                      // Start thread execution
	
  for (;;) {}
}

