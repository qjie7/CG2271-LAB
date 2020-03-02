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
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_PIN] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_PIN] |= PORT_PCR_MUX(3);
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->MOD = 7500;
	TPM1_C0V = 3750;
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
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


