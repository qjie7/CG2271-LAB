#include "MKL25Z4.h"                    // Device header

#define PTB0_PIN 										0
#define PTB1_PIN 										1
#define PTA4_PIN										4

volatile int cnv = 0;
volatile int counter =0;

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

void initPWM(void) {
	
	// Enable clock to supply power to Port A,Port B,Port C anbd Port E
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

		
	// Configure MUX settings to make PTA4_Pin, PTB1_Pin, PTB3_PIN, PTC8_PIN, PTE30_PIN as PWM mode
	PORTA->PCR[PTA4_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA4_PIN] |= PORT_PCR_MUX(3);
	
	
	
	// Enable clock to supply power to TPM0, TPM1 and TPM2
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	


	// SIM->SOPT2 --> System Options Register
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; 	//Ensure TPMSRC bit in SOPT2 is cleared before assigning a new value
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);	//Assign 1 to select s MCGFLLCLK clock or MCGPLLCLK/2 as clock source for the TPM clock

	// Set all TPM values to 0 to prevent the motor from running at the very beginning after running the program
	TPM0->MOD = 7500;
	TPM0_C1V = 3750;


	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 	//Clear before assign value

	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear Channel 1 before assign

	//CMOD:LPTPM counter increments on every LPTPM counter clock
	//PS:111(7) Divide by 128
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));		
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //Clear CPWMS field:LPTPM counter operates in up counting mode.
	

	// ELSB:ELSA = 10 , MSB:MSA = 10  --> Edge-aligned PWM, High-true pulses (clear Output on match, set Output on reload)
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));  // For TPM0 Channel 1  

}

 static void delay(volatile uint32_t nof) {
	while(nof != 0) {
		__ASM("NOP");
		nof--;
	}
}


void set_buzzer_frequency(int frequency) {
	TPM0->MOD = (int)(48000000.0 / (128.0 * frequency));
	TPM0_C1V = TPM0->MOD - 1;
	cnv = TPM0_C1V;
}

void play_note(Note note, NoteLength length, int bpm) {
	counter++;
	set_buzzer_frequency(note);
	delay((int)((float)length / MEDIUM * 60000 / bpm)); //250
	//delay(0x100000);
	//set_buzzer_frequency(0);

	
}



void play_song(void) {
	
	play_note(C4, MEDIUM, 240);
	

	play_note(D4, MEDIUM, 240);
	

	play_note(E4, MEDIUM, 240); 


	play_note(F4, MEDIUM, 240);


	play_note(G4, MEDIUM, 240);


	play_note(A4, MEDIUM, 240);
	

	play_note(B4, MEDIUM, 240);

}

int main(void) {
	
 	SystemCoreClockUpdate();
	initPWM();
	

play_song();


	

	

}


