#include "MKL25Z4.h"                    // Device header

#define PTB0_PIN 										0
#define PTB1_PIN 										1

#define C_NOTE	1431
#define D_NOTE  1275
#define E_NOTE	1136
#define F_NOTE  1074
#define G_NOTE	956
#define	A_NOTE	852
#define	B_NOTE  759

void initPWM(void)
{
	//Enable clock to supply power to Port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//Configure MUX settings to make PTB0_Pin and PTB1_Pin as PWM
	PORTB->PCR[PTB0_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_PIN] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_PIN] |= PORT_PCR_MUX(3);
	
	//Enable the clock gate to the TPM1 module
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	//SIM->SOPT2 --> System Options Register
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; 	//Ensure TPMSRC bit in SOPT2 is cleared before assigning a new value
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);	//Assign 1 to selects MCGFLLCLK clock or MCGPLLCLK/2 as clock source for the TPM clock
	
	TPM1->MOD = 7500;
	TPM1_C0V = 3750;
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value
	
	//CMOD:LPTPM counter increments on every LPTPM counter clock
	//PS:111(7) Divide by 128
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear before assign
	
	// ELSB:ELSA = 10 , MSB:MSA = 10  --> Edge-aligned PWM, High-true pulses (clear Output on match, set Output on reload)
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 
}

void delay (int num) {
	for (int i = 0; i < num; i++);
}

void play(int note){
	TPM1->MOD = note;
	TPM1_C0V = note / 2;
}

void stop() {
	TPM1_C0V = 0;
}

int main(void) {
	
 	SystemCoreClockUpdate();
	initPWM();
	
	while(1) {
		play(C_NOTE);
		delay(0xAFFFF);
		play(C_NOTE);
		delay(0xAFFFF);
		play(G_NOTE);
		delay(0xAFFFF);
		play(G_NOTE);
		delay(0xAFFFF);
		play(A_NOTE);
		delay(0xAFFFF);
		play(A_NOTE);
		delay(0xAFFFF);
		play(G_NOTE);
		delay(0xAFFFF);
		stop();
		
		play(F_NOTE);
		delay(0xAFFFF);
		play(F_NOTE);
		delay(0xAFFFF);
		play(E_NOTE);
		delay(0xAFFFF);
		play(E_NOTE);
		delay(0xAFFFF);
		play(D_NOTE);
		delay(0xAFFFF);
		play(D_NOTE);
		delay(0xAFFFF);
		play(C_NOTE);
		delay(0xAFFFF);	
		stop();
	}

}


