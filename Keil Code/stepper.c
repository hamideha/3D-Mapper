#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "stepper.h"


void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;									// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};					// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;        													// Port H is configured as output for stepper motor
  GPIO_PORTH_AFSEL_R &= ~0xFF;     													// Disable alternate funct on Port H
  GPIO_PORTH_DEN_R |= 0xFF;        													// Enable digital I/O on Port H
  GPIO_PORTH_AMSEL_R &= ~0xFF;     													// Disable analog functionality on Port H		
	return;
}

void RotateCCW(void) {                        // Set bits on Port H opposite of Rotate CW in order to spin the motor in the counterclockwise 
		GPIO_PORTH_DATA_R = 0b00001100;               
		SysTick_Wait(90000);                     		
		GPIO_PORTH_DATA_R = 0b00000110;               
		SysTick_Wait(90000);  
		GPIO_PORTH_DATA_R = 0b0000011;                
		SysTick_Wait(90000); 
		GPIO_PORTH_DATA_R = 0b00001001; 							
		SysTick_Wait(90000); 
}

void RotateCW(void) {                       	// Function that sets the bits PH0-PH3 that drive the motor clockwise according to fullstep driving
		GPIO_PORTH_DATA_R = 0b00001001;       // First step has a sequence of bits of 1001
		SysTick_Wait(90000);                 // Time waited between each step. This is required as time between steps cannot be 0
		GPIO_PORTH_DATA_R = 0b00000011;       // Second step has a sequence of bits of 0011
		SysTick_Wait(90000);                 
		GPIO_PORTH_DATA_R = 0b00000110;       // Third step has a sequence of bits of 0110
		SysTick_Wait(90000);                   
		GPIO_PORTH_DATA_R = 0b00001100;       // Last step has a sequence of bits of 1100
		SysTick_Wait(90000); 
}

void spin(int direction) {                // Call function to spin the motor
		direction == 1? RotateCW(): RotateCCW();       									                          // Depending on direction parameter spin motor
}