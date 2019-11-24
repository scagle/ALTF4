/* Jesus Luciano, Steven Cagle, Alex Jong
 * 
 *
 * Final Implementation for turret control
 * 
 * Pinout:
 *
 * SERV: PA7 PWM Signal controls servo direction
 * 
 * STEP: PC6 Stepping control of motor
 * DIR : PC7 Direction of motor, cw & ccw
 * 
 * MS0 : PB7 Controls step resolution of motor
 * MS1 : PB6 
 * MS2 : PB5
 *
 * LASR: PB4 Controls status of LASER
 * GUN:  PB1 Electronic Control of gun firing mechanism
 *
 * PF4 : sw2 - changes motor resolution
 * PF0 : sw1 - changes motor direction
 * PF1-3: status LEDs
 *
 * PC0-3 is JTAG, do NOT USER
 */

#include "tm4c123gh6pm.h"
#include "UART.h"
#include "stdlib.h"
#include "bluetooth.h"
/***********************************************************************************/
#define FULL 				 0x00
#define HALF 				 0x80
#define QUARTER 		 0x40
#define EIGHTH			 0xC0
#define SIXTEENTH		 0x20
#define THIRTYSECOND 0xA0
#define LEFT 				 0x80
#define RIGHT 			 0x7F
#define SERVO_MIN    9000
#define SERVO_MAX    15000
/***********************************************************************************/
int en = 419;

// UART coordinates
unsigned int xGreen, yGreen, xRed, yRed;

// Stepper Motor variables
unsigned int STEP_COUNT = 0;
char step_en = 0x01;

// UART variables
char STRT[5];
char STOP[5];
char test[5];
char NewDataFlag = 0;		// Flag set when new data is received over UART0

// Servo basic feedback val
unsigned int servo_basic = 12000;
double Servo_pwm = 8000;

// Bluetooth flags
char FiringFlag = 0;		// Flag set when glove is sending fire signal 
char GreenLaserOnFlag = 0;	// Flag set when glove's green laser is on
/***********************************************************************************/
void OutUART(void);
void LaserOn(void);
void LaserOff(void);
void LaserToggle(void);
void ActivateGun(void);
void DeactivateGun(void);
int  strcmp(const char *, const char *);
/***********************************************************************************/
void PortA_Init(){ unsigned long delay;
	SYSCTL_RCGCPWM_R  |= 0x02;					//activate PWM module 1
	SYSCTL_RCGC2_R 		|= 0x00000001;		//activate Port A
	delay = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTA_CR_R	  |= 0x80;					//allow changes to PA7
	GPIO_PORTA_AMSEL_R&= 0xEF;					//disable analog for PA7
	GPIO_PORTA_PCTL_R &= 0x0FFFFFFF; 		//set PA7 as PWM output
	GPIO_PORTA_PCTL_R |= 0x50000000;
	GPIO_PORTA_DIR_R  |= 0x80;					//set PA7 as output
	GPIO_PORTA_AFSEL_R|= 0x80;					//enable alt func PA7
	GPIO_PORTA_DEN_R  |= 0x80;					//digital enable PA7
	
	SYSCTL_RCGCPWM_R  |= 0x02;					//activate PWM M1
	SYSCTL_RCGCGPIO_R |= 0x01; 					//clock for Port A
	SYSCTL_RCC_R 			&=~0x00100000; 		//disable divider
	
	PWM1_1_CTL_R 			 = 0x00;					//disable PWM for initializations
	PWM1_1_GENB_R     |= 0x00000C08; 		//drive PWM b high, invert pwm b
	PWM1_1_LOAD_R 		 = 320000-1; 			  //needed for 20ms period
	PWM1_1_CMPB_R 		 = 8000; 				//0.5ms duty cycle
	PWM1_1_CTL_R 			&=~0x00000010; 	//set to countdown mode
	PWM1_1_CTL_R 			|= 0x00000001; 		//enable generator
	
	PWM1_ENABLE_R 		|= 0x08; //enable output 3 of module 1
	
}

void PortB_Init(){ unsigned long delay;
	SYSCTL_RCGC2_R    |= 0x00000002;		//activate Port B
	delay = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTB_CR_R   |= 0xF2;				//allow changes PB7,6,5,4,1
	GPIO_PORTB_AMSEL_R&= 0x0D;				//disable analog for PB7,6,5,4,1
	GPIO_PORTB_PCTL_R &= 0x0000FF0F;		//set PB7,6,5,4,1 as GPIO
	GPIO_PORTB_DIR_R  |= 0xF2;				//set PB7,6,5,4,1 as output
	GPIO_PORTB_AFSEL_R&= 0x0D;				//disable alt func PB7,6,5,4,1
	GPIO_PORTB_DEN_R  |= 0xF2;				//digital enable PB7,6,5,4,1
}
void PortC_Init(){ unsigned long delay;
	SYSCTL_RCGC2_R 		 |= 0x00000004; //intialize port C
	delay = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTC_LOCK_R   = 0x4C4F434B; //unlock Port C
	GPIO_PORTC_CR_R    |= 0xC0;  		  //allow changes to PC7, 6
	GPIO_PORTC_AMSEL_R &= 0x3F;       //disable analog for PC7, 6
	GPIO_PORTC_PCTL_R  &= 0x00FFFFFF; //set PC7, 6 to GPIO
	GPIO_PORTC_DIR_R   |= 0xC0;				//set PC7, 6 as output
	GPIO_PORTC_AFSEL_R &= 0x3E;				//disable alt func PC7, 6
	GPIO_PORTC_DEN_R   |= 0xC0;				//digital enable PC7, 6
}	
 
void PortF_Init(){ unsigned long delay;//for LED debugging
	SYSCTL_RCGC2_R 		|= 0x00000020;		//activate Port F
	delay = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;			//unlock Port F
	GPIO_PORTF_CR_R	  |= 0x1F;					//allow changes to PF4,3,2,1,0
	GPIO_PORTF_AMSEL_R&= 0xE0;					//disable analog for PF4,3,2,1,0
	GPIO_PORTF_PCTL_R &= 0xFFF00000;		//set PF4,3,2,1,0 as GPIO
	GPIO_PORTF_PCTL_R |= 0x00000050; // configure PF1 as M1_PWM5

	GPIO_PORTF_DIR_R  &= 0xEE;					//set PF4,0 as input
	GPIO_PORTF_DIR_R  |= 0x0E;					//set PF3,2,1 as output
	GPIO_PORTF_AFSEL_R&= 0xE0;					//disable alt func PF4,3,2,1,0
	GPIO_PORTF_AFSEL_R |= 0x02;       // enable alt function on PF1
	GPIO_PORTF_PUR_R  |= 0x13;					//pull up resistors PF4,0
	GPIO_PORTF_DEN_R  |= 0x1F;					//digital enable PF4,3,2,1,0
	
	//Interrupt
	GPIO_PORTF_IS_R		 &= ~0x11; //PF4 edge sensitive
	GPIO_PORTF_IBE_R   &= ~0x11; //PF4 not both edges
	GPIO_PORTF_IEV_R   &=  0x11; //PF4 falling edge
	GPIO_PORTF_ICR_R	 |=  0x11; //PF4 clear flags
	GPIO_PORTF_IM_R    |=  0x11; //arm interrupt
	
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00000000; 	// priority 0 interrupt for switches				 
	NVIC_EN0_R  = 0x40000000;      			// enable interrupt 30 in NVIC
	
	//PWM control - for M1PWM5 on pin PF5
	SYSCTL_RCGCPWM_R |= 0x02; 		// enable PWM M1
	SYSCTL_RCGCGPIO_R |= 0x20;    // activate Port F
																// AFSEL already taken car of
	SYSCTL_RCC_R |=  0x00100000;  // enable PWM divider	 - bit 20		
	SYSCTL_RCC_R &= ~0x000E0000;  // clear divider bits 19-17
	SYSCTL_RCC_R |=  0x00000000;  // set bits 19-17 to 0, for divider of 2
	
	//using M1 and generator 3 for output 5
	//PWM1_2_LOAD_R = period -1;	// set period duration with period variable
	PWM1_2_LOAD_R = 800000;			// set period to 40,000

	//using M1, generator 3, output 2, so CMPB is used instead of CMPA
	//PWM1_2_CMPB_R = duty cycle -1;
	PWM1_2_CMPB_R = 400000;     // set duty cycle to 50%, for initial half brightness
	
	PWM1_2_CTL_R |= 0x00000001;   // enable PWM signal, count down mode
	PWM1_2_GENB_R |= 0x0000080C; 				// llowo n LOAD, high on CMPB down
	
	PWM1_ENABLE_R |= 0x20; 				// enable M1 PWM5 output
	
 }

void SysTick_Init(){
	 // 1 / 16,000,000 * val = sec
	 // output: 
	 NVIC_ST_CTRL_R = 0; //disable systick
	 NVIC_ST_RELOAD_R = 1600; // 0.5 second interval
	 NVIC_ST_CURRENT_R = 0; //reset current couter value
	 NVIC_ST_CTRL_R |= 0x00000007;
 }
/***********************************************************************************/
void SysTick_Handler(void){
	// steps enabled and pulse output count
	if( step_en && STEP_COUNT > 0){
		// check value of step output
		if( GPIO_PORTC_DATA_R & 0x40){// high
			//set low and decrease counter
			GPIO_PORTC_DATA_R &= 0xBF;
			GPIO_PORTF_DATA_R &= ~0x04;
			STEP_COUNT = STEP_COUNT - 1;
		}
		else{// low
			// set high
			GPIO_PORTC_DATA_R |= 0x40; 
			GPIO_PORTF_DATA_R |= 0x04;
		}
	}
	// Turn off output if count is not high
	else{
		GPIO_PORTC_DATA_R &= 0xBF;
	}
}
 
void GPIOPortF_Handler(void){
	if(GPIO_PORTF_DATA_R & 0x01){//left button
		step_en = 0x01;
		STEP_COUNT = 100;
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R ^ 0x08;
	 }
	if(GPIO_PORTF_DATA_R & 0x10){//right button
		// Do nothing for now
	}
	GPIO_PORTF_ICR_R = 0x11; //acknowledge interrupt
}
/***********************************************************************************/
// Functions to change step direction
void step_left(void) { GPIO_PORTC_DATA_R |=  LEFT; }
void step_right(void){ GPIO_PORTC_DATA_R &= RIGHT; }
// Functions to change step resolution
void step_full(void)			{ GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | FULL;			}
void step_half(void)			{ GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | HALF;			}
void step_quarter(void) 		{ GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | QUARTER;		}
void step_eighth(void)			{ GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | EIGHTH;		}
void step_sixteenth(void)		{ GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | SIXTEENTH;		}
void step_thirtysecond(void)	{ GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | THIRTYSECOND;	}
//Change 3 bit outputs to stepper motor driver
void StepOut(){
	// Have an output enable & an output counter to send x number of pulses
	int diff = xRed - xGreen;
	int val_to_step = 0;
	
	// Step count range values
	if(abs(diff) > 400)
		val_to_step = 40;
	else if(abs(diff) > 300)
		val_to_step = 30;
	else if(abs(diff) > 200)
		val_to_step = 20;
	else if(abs(diff) > 100)
		val_to_step = 15;
	else 
		val_to_step = 10;
	
	STEP_COUNT = val_to_step;

	// Check xGreen & xRed
	
	// xRed is left of xGreen
	if( diff < 0 ){ step_right();}
	// xRed is right of xGreen
	else{ 					step_left(); }
	
}

/***********************************************************************************/
// Manually set servo values
void UpdateServo(unsigned int pwm){ PWM1_1_CMPB_R = pwm; }
// Basic movement up and down
void ServoFeedback(){
	// Assume yGreen and yRed variables already read from UART
	// Max pwm range of servo [9000, 15000]
	int diff = yRed - yGreen;
	int val = 0;
	
	// change pwm change value based on range
	if(abs(diff) > 200)
		val = 200;
	else if(abs(diff) > 150)
		val = 100;
	else if(abs(diff) > 100)
		val = 50;
	else if(abs(diff) > 50)
		val = 25;
	else
		val = 10;
	
	// check for negative movement
	if(diff > 0)
		val *= -1;
	
	servo_basic += val;
	
	// check max and min servo values
	if(servo_basic < 9000) { servo_basic =  9000; }
	if(servo_basic > 15000){ servo_basic = 15000; }
	
	// Update servo position
	UpdateServo(servo_basic);
}
/***********************************************************************************/
void LaserOn() { GPIO_PORTB_DATA_R |=  0x10; NewDataFlag = 1; }
void LaserOff(){ GPIO_PORTB_DATA_R &= ~0x10; NewDataFlag = 0; }
void LaserToggle(){ GPIO_PORTB_DATA_R ^= 0x10; NewDataFlag ^= 1; }
void ActivateGun(){ GPIO_PORTB_DATA_R |= 0x02; FiringFlag = 1; }
void DeactivateGun(){ GPIO_PORTB_DATA_R &= ~0x02; FiringFlag = 0; }
/***********************************************************************************/
// Send carriage return, line feed
void OutCRLF(void){ UART_OutChar(CR); UART_OutChar(LF); }
// Bluetooth reading and output
void GetBluetooth(){
	unsigned char character = UART2_NonBlockingInChar();
	switch ( character )
	{
		case 'f':	// Start (F)iring
			ActivateGun();
			break;
		case 's':	// (S)top Firing
			DeactivateGun();
			break;
		case 'h':	// Laser on ( (H)igh )
			GreenLaserOnFlag = 1;
			break;
		case 'l':	// Laser off ( (L)ow )
			GreenLaserOnFlag = 0;
			break;
		default:
			break;
	}
}

// UART reading and output
void GetUART(){
	// Change color to red before receiving 4 characters
	//GPIO_PORTF_DATA_R  = 0x02;
	
	// Get first "strt" string and check if it correct
	char temp_check = 0x00; 
	int  temp = 9000;
	UART_InString(STRT, 5);

	temp_check = strcmp(STRT, "strt");
	if(temp_check != 0){
		GPIO_PORTF_DATA_R |= 0x02; // Red == 1st input not start
		temp = UART_InUDec();
		servo_basic = temp;
		UpdateServo(temp);
		return;
	}
	
	// Read either none or number
	UART_InString(test, 5);
	temp_check = strcmp(test, "none");
	// Did not get a none
	if(strcmp(test, "none") != 0){
		LaserOn();
		// Convert string to int
		xGreen = atoi(test);

		// Get 3 remaining numbers through UART			
		yGreen = UART_InUDec();
		xRed   = UART_InUDec();
		yRed   = UART_InUDec();	
		//GPIO_PORTF_DATA_R |= 0x08;
	}
	// None
	else{ LaserOff(); }
	
	// Get stop
	UART_InString(STOP, 5);
	
	if(strcmp(STOP, "stop") != 0){
		//GPIO_PORTF_DATA_R |= 0x04;
		return;
	}
	//GPIO_PORTF_DATA_R = 0x0E; //white if data good

	// Change color to sky blue after receiving 4 characters
	//GPIO_PORTF_DATA_R  = 0x0C;
}

//debug code
int main(void){
	PortA_Init();
	PortB_Init();
	PortC_Init();
	PortF_Init();
	SysTick_Init();
	UART_Init();
	BT_Init();		// Bluetooth UART from the glove
	
	UpdateServo(servo_basic);
	//step_thirtysecond();
	step_full();

	while(1){
		GetBluetooth();		// Handle Glove UART input
		GetUART();			// Handle Computer UART input

		// Update Turret Servo / Stepper values
		// (Only updates if glove's green laser is on, and if we got new data)
		if(GreenLaserOnFlag && NewDataFlag){
			ServoFeedback();
			StepOut();
		}
	}
}


// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06

