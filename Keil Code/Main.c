#include <stdint.h>
#include <math.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "stepper.h"

#define X_INCREMENT 200 																		// This is the manual displacement you move your sensor after one full 360 degree vertival scan
																														// You can define it to be whaever you would like. Between 10cm to 50cm is recommended.

void PortL_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;		              // Initialize the RCGCGPIO clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	      // Do nothing to allow RCGCGPIO clock to stabilize  
	GPIO_PORTL_DEN_R |= 0xFF;			
	GPIO_PORTL_DIR_R = 0xFF;																	// Port L pins are used as digital outputs (rows) 																																
	return;
}


void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Initialize the RCGCGPIO clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};				// Do nothing to allow RCGCGPIO clock to stabilize
	GPIO_PORTM_DEN_R = 0b00001111;		
	GPIO_PORTM_DIR_R = 0b00000000;       								    	// Port M pins are initialized as inputs to capture button press (columns)
	return;
}

// The VL53L1X uses a slightly different way to define the default address of 0x29
// The I2C protocol defintion states that a 7-bit address is used for the device
// The 7-bit address is stored in bit 7:1 of the address register.  Bit 0 is a binary
// value that indicates if a write or read is to occur.  The manufacturer lists the 
// default address as 0x52 (0101 0010).  This is 0x29 (010 1001) with the read/write bit
// alread set to 0.
//uint16_t	dev = 0x29;
uint16_t	dev=0x52;

int status=0;

volatile int IntCount;

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t debugArray[100];

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  // uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	float angle = 0, x = 0, y = 0, z = 0;							// x,y,z data that will be sent to PySerial

	//initialize
	PortL_Init();																			// Initialize all the ports			
	PortM_Init();
	PortH_Init();
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"ESMS Program: %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n");
 	UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	Status_Check("StartRanging", status);

	GPIO_PORTL_DATA_R = 0b00001110;      														// Output to Port L that the first row is active low
	while(1) {
		while((GPIO_PORTM_DATA_R & 0b00000001)==0){										// Check if the first column (push button) is active
			for(int stepsCounter = 0; stepsCounter < 512; stepsCounter++) {
			
				spin(1);																									// Spin the motor only when the button is pressed
				
				if(stepsCounter%8 == 0) {																	// Take a measurement every 8 steps or equivalently every 5.625 degrees
					FlashLED2(1);
					while (dataReady == 0){
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
					}
					
					dataReady = 0;
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);			// Get status of ToF sensor
					status = VL53L1X_GetDistance(dev, &Distance);						// Get distance measurement from ToF sensor
		
					debugArray[i] = Distance;
					angle = stepsCounter/8 * 5.625 * 0.0174532;            	// The
					y = Distance * sin(angle);														 	// y-direction is Distance * sin(angle the motor is currently at)
					z = Distance * cos(angle);															// z-direction is Distance * cos(angle the motor is currently at)
					
					status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
					sprintf(printf_buffer,"%f %f %f\r\n",x,y,z);						// Using serial UART communicate with PySerial the x,y,z data
					UART_printf(printf_buffer);
				}
			}
			x += X_INCREMENT;																						// Everytime the button is pressed again we will manually increment the x-direction
																																	// On the user side, they must manually move the sensor the displacement the defined 
																																	// on line 12
			for(int stepsCounter = 0; stepsCounter < 512; stepsCounter++) { // Spin the motor in the reverse direction to untangle the wires
				spin(0);
			}
			UART_printf("Press button for next measurement\r\n"); 			// Message displayed to user as prompt if they ant to take next measurement
		}
	}	
}


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock 
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


