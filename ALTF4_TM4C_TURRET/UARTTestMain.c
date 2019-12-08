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
#include "stdio.h"
#include "string.h"
#include "bluetooth.h"
#include "pid.h"
/***********************************************************************************/
#define FULL 				 0x00
#define HALF 				 0x80
#define QUARTER 		 0x40
#define EIGHTH			 0xC0
#define SIXTEENTH		 0x20
#define THIRTYSECOND 0xA0
#define LEFT 				 0x80
#define RIGHT 			 0x7F
#define SERVO_MIN    22000
#define SERVO_MAX    32000
#define SERVO_RANGE  (SERVO_MAX - SERVO_MIN)
#define SERVO_MIDDLE (SERVO_MIN + SERVO_RANGE / 2)

unsigned char step_sizes[] = { THIRTYSECOND, SIXTEENTH, EIGHTH, QUARTER, HALF, FULL };

/***********************************************************************************/
int en = 419;

// UART coordinates
unsigned int xGreen = 320; 	// Center X
unsigned int yGreen = 240;	// Center Y
unsigned int xRed = 320;		// Center X
unsigned int yRed = 240;		// Center Y

// PID Variables
unsigned int *servo_command = &yGreen;
unsigned int *servo_position = &yRed;
unsigned int *stepper_command = &xGreen;
unsigned int *stepper_position = &xRed; 	// TODO: Maybe calibrate the X on camera, and use that instead
int servo_drive = 0;
int stepper_drive = 0;
PID servo_pid = { 
	.Kp = 3,              // Proportional Constant                
	.Ki = 10,             // Integral Constant
	.Kd = 1,              // Derivative Constant
	.time_constant = 8,   // For integral
	.error = 0,           // Expected output - Actual Output       
	.reset_register = 0,  // Accumulated error of integral
	.last_error = 0       // Track previous error for Derivative
};

PID stepper_pid = { 
	.Kp = 5,              // Proportional Constant                
	.Ki = 1,              // Integral Constant
	.Kd = 1,              // Derivative Constant
	.time_constant = 1,   // For integral
	.error = 0,           // Expected output - Actual Output       
	.reset_register = 0,  // Accumulated error of integral
	.last_error = 0       // Track previous error for Derivative
};
	

// Stepper Motor variables
unsigned char step_size = THIRTYSECOND;	
unsigned char STEP_COUNT = 0;	
char step_en = 0x01; 		// Flag set when stepper is enabled

// UART Capture Data variables
char UART_IN[10];
char NewDataFlag = 0;		// Flag set when new data is received over UART0
char state = 0;
int delimit_index = 0;

// Servo basic feedback val
unsigned int servo_pwm = SERVO_MIDDLE;
unsigned int servo_basic = SERVO_MIDDLE;
unsigned int range = SERVO_RANGE;
unsigned int middle = SERVO_MIDDLE;

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
	SYSCTL_RCGCPWM_R   |= 0x02;        // activate PWM module 1
	SYSCTL_RCGC2_R     |= 0x00000001;  // activate Port A
	delay = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTA_CR_R    |= 0x80;        // allow changes to PA7
	GPIO_PORTA_AMSEL_R &= 0xEF;        // disable analog for PA7
	GPIO_PORTA_PCTL_R  &= 0x0FFFFFFF;  // set PA7 as PWM output
	GPIO_PORTA_PCTL_R  |= 0x50000000;
	GPIO_PORTA_DIR_R   |= 0x80;        // set PA7 as output
	GPIO_PORTA_AFSEL_R |= 0x80;        // enable alt func PA7
	GPIO_PORTA_DEN_R   |= 0x80;        // digital enable PA7
	
	SYSCTL_RCGCPWM_R   |= 0x02;        // activate PWM M1
	SYSCTL_RCGCGPIO_R  |= 0x01;        // clock for Port A
	SYSCTL_RCC_R       &= ~0x00100000; // disable divider
	
	PWM1_1_CTL_R        = 0x00;        // disable PWM for initializations
	PWM1_1_GENB_R      |= 0x00000C08;  // drive PWM b high, invert pwm b
	PWM1_1_LOAD_R       = 320000-1;    // needed for 20ms period
	PWM1_1_CMPB_R       = servo_pwm;   // 0.5ms duty cycle
	PWM1_1_CTL_R       &= ~0x00000010; // set to countdown mode
	PWM1_1_CTL_R       |= 0x00000001;  // enable generator
	
	PWM1_ENABLE_R      |= 0x08;        // enable output 3 of module 1
	
}

void PortB_Init(){ unsigned long delay;
	SYSCTL_RCGC2_R     |= 0x00000002; // activate Port B
	delay               = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTB_CR_R    |= 0xF2;       // allow changes PB7,6,5,4,1
	GPIO_PORTB_AMSEL_R &= 0x0D;       // disable analog for PB7,6,5,4,1
	GPIO_PORTB_PCTL_R  &= 0x0000FF0F; // set PB7,6,5,4,1 as GPIO
	GPIO_PORTB_DIR_R   |= 0xF2;       // set PB7,6,5,4,1 as output
	GPIO_PORTB_AFSEL_R &= 0x0D;       // disable alt func PB7,6,5,4,1
	GPIO_PORTB_DEN_R   |= 0xF2;       // digital enable PB7,6,5,4,1
}
void PortC_Init(){ unsigned long delay;
	SYSCTL_RCGC2_R     |= 0x00000004; // intialize port C
	delay               = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTC_LOCK_R   = 0x4C4F434B; // unlock Port C
	GPIO_PORTC_CR_R    |= 0xC0;       // allow changes to PC7, 6
	GPIO_PORTC_AMSEL_R &= 0x3F;       // disable analog for PC7, 6
	GPIO_PORTC_PCTL_R  &= 0x00FFFFFF; // set PC7, 6 to GPIO
	GPIO_PORTC_DIR_R   |= 0xC0;       // set PC7, 6 as output
	GPIO_PORTC_AFSEL_R &= 0x3E;       // disable alt func PC7, 6
	GPIO_PORTC_DEN_R   |= 0xC0;       // digital enable PC7, 6
}

void PortF_Init(){ unsigned long delay; // for LED debugging
	SYSCTL_RCGC2_R     |= 0x00000020;   // activate Port F
	delay               = SYSCTL_RCGC2_R; delay++;
	GPIO_PORTF_LOCK_R   = 0x4C4F434B;   // unlock Port F
	GPIO_PORTF_CR_R    |= 0x1F;         // allow changes to PF4,3,2,1,0
	GPIO_PORTF_AMSEL_R &= 0xE0;         // disable analog for PF4,3,2,1,0
	GPIO_PORTF_PCTL_R  &= 0xFFF00000;   // set PF4,3,2,1,0 as GPIO
	GPIO_PORTF_PCTL_R  |= 0x00000050;   // configure PF1 as M1_PWM5
	
	GPIO_PORTF_DIR_R   &= 0xEE;         // set PF4,0 as input
	GPIO_PORTF_DIR_R   |= 0x0E;         // set PF3,2,1 as output
	GPIO_PORTF_AFSEL_R &= 0xE0;         // disable alt func PF4,3,2,1,0
	GPIO_PORTF_AFSEL_R |= 0x02;         // enable alt function on PF1
	GPIO_PORTF_PUR_R   |= 0x13;         // pull up resistors PF4,0
	GPIO_PORTF_DEN_R   |= 0x1F;         // digital enable PF4,3,2,1,0
	
	//Interrupt
	GPIO_PORTF_IS_R  &= ~0x11;          // PF4 edge sensitive
	GPIO_PORTF_IBE_R &= ~0x11;          // PF4 not both edges
	GPIO_PORTF_IEV_R &= 0x11;           // PF4 falling edge
	GPIO_PORTF_ICR_R |= 0x11;           // PF4 clear flags
	GPIO_PORTF_IM_R  |= 0x11;           // arm interrupt
	
	NVIC_PRI7_R       = (NVIC_PRI7_R&0xFF00FFFF)|0x00000000; // priority 0 interrupt for switches
	NVIC_EN0_R        = 0x40000000;     // enable interrupt 30 in NVIC
	
	//PWM control - for M1PWM5 on pin PF5
	SYSCTL_RCGCPWM_R  |= 0x02; // enable PWM M1
	SYSCTL_RCGCGPIO_R |= 0x20; // activate Port F
	// AFSEL already taken car of
	SYSCTL_RCC_R |=  0x00100000;  // enable PWM divider  - bit 20
	SYSCTL_RCC_R &= ~0x000E0000;  // clear divider bits 19-17
	SYSCTL_RCC_R |=  0x00000000;  // set bits 19-17 to 0, for divider of 2
	
	//using M1 and generator 3 for output 5
	//PWM1_2_LOAD_R = period -1;    // set period duration with period variable
	PWM1_2_LOAD_R = 800000;         // set period to 40,000
	
	//using M1, generator 3, output 2, so CMPB is used instead of CMPA
	//PWM1_2_CMPB_R = duty cycle -1;
	PWM1_2_CMPB_R  = 400000;     // set duty cycle to 50%, for initial half brightness
	
	PWM1_2_CTL_R  |= 0x00000001; // enable PWM signal, count down mode
	PWM1_2_GENB_R |= 0x0000080C; // llowo n LOAD, high on CMPB down
	
	PWM1_ENABLE_R |= 0x20;       // enable M1 PWM5 output
	
}

void SysTick_Init(){
	 // 1 / 16,000,000 * val = sec
	 // output: 
	 NVIC_ST_CTRL_R = 0; //disable systick
	 NVIC_ST_RELOAD_R = 8000; // 0.5 second interval
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
			//GPIO_PORTF_DATA_R &= ~0x04;
			STEP_COUNT = STEP_COUNT - 1;
		}
		else{// low
			// set high
			GPIO_PORTC_DATA_R |= 0x40; 
			//GPIO_PORTF_DATA_R |= 0x04;
		}
	}
	// Turn off output if count is not high
	else{
		GPIO_PORTC_DATA_R &= 0xBF;
	}
}
// Functions to change step direction
void step_left(void) { GPIO_PORTC_DATA_R |=  LEFT; }
void step_right(void){ GPIO_PORTC_DATA_R &= RIGHT; }

void GPIOPortF_Handler(void){
	if(GPIO_PORTF_DATA_R & 0x01){//left button
		LaserOn();
		//step_right();
		//step_en = 0x01;
		//STEP_COUNT = 2000;
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R ^ 0x08;
	 }
	if(GPIO_PORTF_DATA_R & 0x10){//right button
		// Do nothing for now
		LaserOff();
		//		step_left();
		//step_en = 0x01;
		//STEP_COUNT = 2000;
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R ^ 0x08;
	}
	GPIO_PORTF_ICR_R = 0x11; //acknowledge interrupt
}
/***********************************************************************************/
// Functions to change step direction
// Functions to change step resolution
void step_full(void)            { GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | FULL;          }
void step_half(void)            { GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | HALF;          }
void step_quarter(void)         { GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | QUARTER;       }
void step_eighth(void)          { GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | EIGHTH;        }
void step_sixteenth(void)       { GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | SIXTEENTH;     }
void step_thirtysecond(void)    { GPIO_PORTB_DATA_R = (GPIO_PORTB_DATA_R&0x1F) | THIRTYSECOND;  }
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
	else{ step_left(); }
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
void LaserOn() { GPIO_PORTB_DATA_R |=  0x10;}
void LaserOff(){ GPIO_PORTB_DATA_R &= ~0x10;}
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

int findDelimiterIndex(char *string, int strlen, unsigned char delimit){
	int index;
	for (index = 0; index < strlen; index++){
		if (string[index] == delimit){
			return index;
		}
	}
	return -1;
}
// UART reading and output
void GetUART(){
	// Get first "strt" string and check if it correct
	int temp = 9000;
	state = 0;

	// Change color to yellow before receiving 4 characters
	// GPIO_PORTF_DATA_R  = 0x0A;

	state = UART_InString( UART_IN, 10 );

	if( strcmp( UART_IN, "" ) == 0 ){
		return;
	}

	if( strcmp( UART_IN, "none" ) == 0 ){
		UART_OutString("Error: '");
		UART_OutString(UART_IN);
		UART_OutString("'\r\n'");
	}

	delimit_index = findDelimiterIndex( UART_IN, 10, ':' );
	if ( delimit_index != -1 ){
		if ( delimit_index != 2 ){
			UART_OutString("Error: '");
			UART_OutString(UART_IN);
			UART_OutString("'\r\n'");
		}
		else{
			UART_OutString("Got: '");
			UART_OutString(UART_IN);
			UART_OutString("'\r\n");

			char header[10] = "";
			char value_string[10] = "";
			int value = 0;
			strncpy( header, UART_IN, delimit_index);
			header[2] = '\0';
			strncpy( value_string, UART_IN+delimit_index+1, 10);
			value_string[4] = '\0';
			
			unsigned int place = 1; // one's place, ten's place, hundred's place, etc
			int i = 0;
			for ( i = strlen(value_string) - 1; i >= 0; i-- ){
				if ( value_string[i] >= 0x30 && value_string[i] <= 0x39 )
				{
					value += ( value_string[i] - 0x30 ) * place;
					place *= 10;
				}
			}

			switch( header[0] )
			{
				case 'g':
					if( header[1] == 'x' )
						xGreen = value;
					if( header[1] == 'y' )
						yGreen = value;
					break;
				case 'r':
					if( header[1] == 'x' )
						xRed = value;
					if( header[1] == 'y' )
					{
						yRed = value;
						NewDataFlag = 1; // End of UART packet
						LaserOn();
					}
					break;
			}
		}
	}


	// Did not get a none
	//if(strcmp(test, "none") != 0){
	//	LaserOn();
	//	// Convert string to int
	//	xGreen = atoi(test);

	//	// Get 3 remaining numbers through UART			
	//	yGreen = UART_InUDec();
	//	OutCRLF();
	//	//GPIO_PORTF_DATA_R  = 0x0C;

	//	xRed   = UART_InUDec();
	//	OutCRLF();
	//	//GPIO_PORTF_DATA_R  = 0x06;

	//	yRed   = UART_InUDec();
	//	OutCRLF();		
	//	//GPIO_PORTF_DATA_R = 0x08; // Green = 4 valid coordinates
	//}
	//// None
	//else{ LaserOff(); }
	//GPIO_PORTF_DATA_R  = 0x0C;
	
	//GPIO_PORTF_DATA_R = 0x0E; // White = valid data packet

	// Change color to sky blue after receiving 4 characters
}
void GetData(void){
	// Red before receiving data
	//GPIO_PORTF_DATA_R  = 0x02;
	
	UART_InString(UART_IN, 5);
	
	// Blue after receiving data
	//GPIO_PORTF_DATA_R  = 0x04;
	
	UART_OutString("Data: ");
	UART_OutString(UART_IN);
	OutCRLF();
	
	// Green after sending data back
	//GPIO_PORTF_DATA_R  = 0x08;
}

void OutputServo( PID *pid, int drive ){
	// Saturation limit clamping 
	int before_clamp_drive = drive;
	if( drive > ( SERVO_RANGE / 2) )
		drive = ( SERVO_RANGE / 2 );
	else if( drive < -( SERVO_RANGE / 2) )
		drive = -( SERVO_RANGE / 2 );

	// Prevent Integral Wind-Up after Clamping
	if( drive != before_clamp_drive ){ 	// If clamping has occurred
		// Check if Integral is causing wind up
		if( ( pid->error > 0 ) && ( drive > 0 ) ) // If both positive 
			pid->reset_register = ( SERVO_RANGE / 2 );
		if( ( pid->error < 0 ) && ( drive < 0 ) ) // if both negative
			pid->reset_register = -( SERVO_RANGE / 2 );
	}
	// Actuator Command
	servo_pwm = SERVO_MIDDLE + drive;

	// Actuator Output clamping
	if( servo_pwm < SERVO_MIN )
		servo_pwm = SERVO_MIN;
	else if( servo_pwm > SERVO_MAX )
		servo_pwm = SERVO_MAX;

    UpdateServo(servo_pwm); // Set PWM1_1 to servo_pwm
}

void OutputStepper( PID *pid, int drive ){
	//UART_OutUDec((unsigned int)drive);
}

//debug code
int main(void){
	PortA_Init();
	PortB_Init();
	PortC_Init();
	//PortF_Init();
	SysTick_Init();
	UART_Init();
	BT_Init();		// Bluetooth UART from the glove

	UpdateServo(servo_pwm); // Set PWM1_1 to servo_pwm
	//UpdateServo(servo_basic);
	//step_thirtysecond();
	//step_full();

	while(1){
		// Get Data
		//GetBluetooth();  // Handle Glove UART input
		GetUART();       // Handle Computer UART input
		//GetData();     // Echo Data

		// Update PIDs
		servo_pid.error = *servo_command - *servo_position;
		stepper_pid.error = *stepper_command - *stepper_position;
		servo_drive   = UpdatePID( &servo_pid,   *servo_position   );
		stepper_drive = UpdatePID( &stepper_pid, *stepper_position );

		GreenLaserOnFlag = 1; // TODO: Replace this in release
		// Output to motors
		if(GreenLaserOnFlag && NewDataFlag){
			OutputServo( &servo_pid, servo_drive );
			//OutputStepper( &stepper_pid, stepper_drive );
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

