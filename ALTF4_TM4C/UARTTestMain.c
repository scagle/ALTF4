/* Jesus Luciano, Steven Cagle, Alex Jong
 * 
 *
 * Test file for stepper motor control
 * 
 * Pinout:
 *
 * STEP: PD0 - M0PWM6
 *
 *-SERV: PA7 PWM Signal controls servo direction
 *-DIR : PC7 Direction of motor, cw & ccw
 * 
 * MS1 : PB5 Controls stepping of motor
 * MS2 : PB6 
 * MS3 : PB7
 * 
 *-PF4 : sw2 - changes motor resolution
 *-PF0 : sw1 - changes motor direction
 *
 *-PF1-3: status LEDs
 *
 * TODO: Add ADC Input for 3 inputs
 * 1st Input: Val for servo. 
 * 2nd Input: Val for stepper motor
 * 3rd Input: Val for speed for stepper motor
 *
 * ADC Inputs
 *
 * PE1 - AIN2
 * PE4 - AIN9
 * PE5 - AIN8
 *
 * STEPPER_MOTOR_OUTPUT
 * PC6
 *
 *
 * PC0-3 is JTAG, do NOT USER
 *
 ***************************************************
 *
 * PID notes
 *
 * Proportional Controller
 * -Error = Set Point - Process Variable
 * Control Variable = Control Variable + (Kp * Error ) 
 * 
 * Integral Controller - for steady state error, accumulated error
 * Integral = Sum(Error)
 * -Error = Set Point - Process Variable
 * -Integral = Integral + Error
 * Control Variable = (Kp * Error) + (Ki * Integral)
 * // Some applications stop accumulating error when CV is saturated
 * 
 * Derivative Controller - rate of change in an error (slows down system when approaching target)
 * Derivative = Error - Last Error
 * -Last Error = Error
 * -Error = Set Point - Process Variable
 * Derivative = Error - Last Error 
 * Control Variable = (Kp* Error) + (Kd * Derivative)
 *
 * Final
 * Control Variable = (Kp * P_error) + (Ki * I_sum) + (Kd * D_error)
 */

#include "tm4c123gh6pm.h"
#include "UART.h"
#include "stdlib.h"
/***********************************************************************************/
int en = 419;
volatile float         L_SENSvolts,  R_SENSvolts ; // Sensor volt for left/right ADC
volatile unsigned long L_ADCvalue,  R_ADCvalue   ; // Sensor for forward left/right IR ADC
volatile unsigned long IRleft, IRright, POTmeter ; // ADC input variables

// Servos
const unsigned int LEFT_MOST_VALUE = 8000;   // 5%  Duty
const unsigned int RIGHT_MOST_VALUE = 40000; // 10% Duty
const unsigned int RANGE = RIGHT_MOST_VALUE - LEFT_MOST_VALUE;
const unsigned int MAX_ADC = 4095;

unsigned int getServoPosition(volatile unsigned long *ADC_Servo_Value);

// UART coordinates
unsigned int xGreen, yGreen, xRed, yRed;

// Servo PID constants, adjust as needed
double  Servo_PID = 0;
double  Servo_pwm = 8000;// Min servo value
double  Servo_error = 0;

// Proportional
const double Kp = 1.0;

// Integral
const double Ki = 0.5;
double Servo_error_sum = 0;

// Derivative
const double Kd = 0.2;
double Servo_derivative = 0;
double Servo_last_error = 0;

// Stepper Motor vars
unsigned int STEP_COUNT = 0;
char step_en = 0x01;

// UART vars
char STRT[5];
char STOP[5];
char test[5];

// Servo basic feedback val
unsigned int servo_basic = 12000;


void ServoPID(void);
void OutUART();

/***********************************************************************************/
void PortA_Init(){ unsigned long delay;
	SYSCTL_RCGCPWM_R  |= 0x02;					//activate PWM module 1
	SYSCTL_RCGC2_R 		|= 0x00000001;		//activate Port A
	delay = SYSCTL_RCGC2_R;
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

void PortC_Init(){ unsigned long delay;
	SYSCTL_RCGC2_R 		 |= 0x00000004; //intialize port C
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTC_LOCK_R   = 0x4C4F434B; //unlock Port C
	GPIO_PORTC_CR_R    |= 0xC0;  		  //allow changes to PC7, 6
	GPIO_PORTC_AMSEL_R &= 0x3F;       //disable analog for PC7, 6
	GPIO_PORTC_PCTL_R  &= 0x00FFFFFF; //set PC7, 6 to GPIO
	GPIO_PORTC_DIR_R   |= 0xC0;				//set PC7, 6 as output
	GPIO_PORTC_AFSEL_R &= 0x3E;				//disable alt func PC7, 6
	GPIO_PORTC_DEN_R   |= 0xC0;				//digital enable PC7, 6
}	
 
void PortB_Init(){ unsigned long delay;
	SYSCTL_RCGC2_R    |= 0x00000002;		//activate Port B
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTB_CR_R   |= 0xE0;  		    //allow changes PB7,6,5
	GPIO_PORTB_AMSEL_R&= 0x1F;  			  //disable analog for PB7,6,5
	GPIO_PORTB_PCTL_R &= 0x000FFFFF; 		//set PB7,6,5 as GPIO
	GPIO_PORTB_DIR_R  |= 0xE0;					//set PB7,6,5 as output
	GPIO_PORTB_AFSEL_R&= 0x1F;					//disable alt func PB7,6,5
	GPIO_PORTB_DEN_R  |= 0xE0;					//digital enable PB7,6,5
}
void PortF_Init(){ unsigned long delay;//for LED debugging
	SYSCTL_RCGC2_R 		|= 0x00000020;		//activate Port F
	delay = SYSCTL_RCGC2_R; 
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
	 NVIC_ST_RELOAD_R = 16000000; // 0.5 second interval
	 NVIC_ST_CURRENT_R = 0; //reset current couter value
	 NVIC_ST_CTRL_R |= 0x00000007;
 }
/***********************************************************************************/
void SysTick_Handler(void){
	// steps enabled and pulse output count
	if(	step_en && STEP_COUNT > 0){
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
}
 
void GPIOPortF_Handler(void){
	 if(GPIO_PORTF_DATA_R & 0x01){//left button
			step_en = 0x01;
		  STEP_COUNT = 5;
	 }
	 if(GPIO_PORTF_DATA_R & 0x10){//right button
			step_en = 0x01;
		  STEP_COUNT = 2;		 
	 }
	 
	 GPIO_PORTF_ICR_R = 0x11; //acknowledge interrupt
	 
 }
 

/***********************************************************************************/
// Functions to change step direction
void step_left(void){
	GPIO_PORTC_DATA_R |= 0x80;
}
void step_right(void){
	GPIO_PORTC_DATA_R &= 0x7F;;
}
// Functions to change step resolution
/*

 * MS1 : PB5 Controls stepping of motor
 * MS2 : PB6 
 * MS3 : PB7
 * 
*/
void step_full(void){
	GPIO_PORTB_DATA_R &= 0x1F;
	GPIO_PORTB_DATA_R |= 0x00;
}
void step_half(void){
	GPIO_PORTB_DATA_R &= 0x1F;
	GPIO_PORTB_DATA_R |= 0x20;
}
void step_quarter(void){
	GPIO_PORTB_DATA_R &= 0x1F;
	GPIO_PORTB_DATA_R |= 0x40;
}
void step_eighth(void){
	GPIO_PORTB_DATA_R &= 0x1F;
	GPIO_PORTB_DATA_R |= 0xD0;
}
void step_sixteenth(void){
	GPIO_PORTB_DATA_R &= 0x1F;
	GPIO_PORTB_DATA_R |= 0xE0;
}

//Change 3 bit outputs to stepper motor driver
void StepOut(){
	// Send x number of pulses using systick
	// Have an output enable & an output counter to send x number of pulses
	int diff = xRed - xGreen;
	step_full();
	// Check xGreen & xRed
	if( diff < 0 ){ // xRed is left of xGreen
		step_right();
		STEP_COUNT = 100;
	}
	else{ // xRed is right of xGreen
		step_left();
		STEP_COUNT = 100;
	}
	// 
}

// Returns a clock cycle count requirement in order to acheive PWM duty cycles 
// Converts (0-4095) to 5% - 10% duty cycle (or 8000-40000 clock cycles)
unsigned int getServoPosition(volatile unsigned long *ADC_Servo_Value){
    double Add_Value = 2.44;
    return ( Add_Value * (*ADC_Servo_Value)  + LEFT_MOST_VALUE);
}

// Set servo pwm value using paramter
void servo_pos(unsigned long val){
	//unsigned long normalVal;
	//normalVal = getServoPosition(&val);
	PWM1_1_CMPB_R = val;//normalVal;
}


/***********************************************************************************/
// Manually set servo values
void UpdateServo(unsigned int pwm){
	PWM1_1_CMPB_R = pwm;
}
// Servo PID loop. Adjust servo pwm output based on a closed feedback loop
void ServoPID(){
		// yGreen - yRed = target - current
		// range [0, 1079]
		// reduced resolution to 640 x 480
		
		// Derivative Last Error
		Servo_last_error = Servo_error;
	  
		// Error
		Servo_error = yGreen - yRed;
		
		// Integral Error Sum
		Servo_error_sum = Servo_error_sum + Servo_error;
	
		// Derivative 
		Servo_derivative = Servo_error - Servo_last_error;

		// Set servo value
		Servo_PID = (Kp * Servo_error) + (Ki * Servo_error_sum) + (Kd * Servo_derivative);
		
		Servo_pwm = Servo_pwm + Servo_PID;
		// Servo_pwm value range [9000, 15000], min & maxes
		if(Servo_pwm < 8000){
			Servo_pwm = 8000;
			Servo_PID = 0;
			Servo_error_sum = 0;
		}
		else if(Servo_pwm > 16000){
			Servo_pwm = 40000;
			Servo_PID = 0;
			Servo_derivative = 0;
		}
		
		//Set servo value
		UpdateServo(Servo_pwm);
}



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
	if(servo_basic < 9000)
		servo_basic = 9000;
	if(servo_basic > 15000)
		servo_basic = 15000;
	
	UpdateServo(servo_basic);
}

/***********************************************************************************/
// Send carriage return, line feed
void OutCRLF(void){
    UART_OutChar(CR);
    UART_OutChar(LF);
}
/*
unsigned long UART_InData(void){
	char character;
	unsigned long num = 0, len = 0;
	character = UART_InChar();
  while(character != CR){
		 if((character>='0') && (character<='9')) {// character is a number

		 }
	}
	
	
	return character;
}*/
// UART reading and output
void GetUART(){
	
		// Print line to send coordinates
		//UART_OutString("Send Coordinates. PID Control Variable: ");
		//UART_OutUDec(Servo_pwm);
		//OutCRLF();
		
		// Change color to red before receiving 4 characters
		//GPIO_PORTF_DATA_R  = 0x02;
		
		// Get first "strt" string and check if it correct
		int temp = 9000;

		UART_InString(STRT, 5);
		GPIO_PORTF_DATA_R = 0x00;
	
		if(strcmp(STRT, "strt") != 0){
			GPIO_PORTF_DATA_R |= 0x02; // Red == 1st input not start
			temp = UART_InUDec();
			servo_basic = temp;
			UpdateServo(temp);
			return;
		}
				// Read either none or number
		UART_InString(test, 5);
		
		// Did not get a none
		if(strcmp(test, "none") != 0){
			
			// Convert string to int
			xGreen = atoi(test);

			// Get 3 remaining numbers through UART			
			yGreen = UART_InUDec();
			xRed   = UART_InUDec();
			yRed   = UART_InUDec();	
			GPIO_PORTF_DATA_R |= 0x08;
		}
		
		// Get stop
		UART_InString(STOP, 5);
		
		if(strcmp(STOP, "stop") != 0){
			GPIO_PORTF_DATA_R |= 0x04;
			return;
		}
		
		OutUART();
		GPIO_PORTF_DATA_R = 0x0E; //white if data good
		
		// Change color to sky blue after receiving 4 characters
		//GPIO_PORTF_DATA_R  = 0x0C;

	
}
void OutUART(){
		OutCRLF();
		// Print pwm value to terminal, check changes
		GPIO_PORTF_DATA_R  = 0x08; // Green
		UART_OutString("Coordinates Received. PID Control Variable: ");
		UART_OutUDec(Servo_pwm);
		OutCRLF();
		// Send Coordinates back
	
		UART_OutString("xGreen: ");
		UART_OutUDec(xGreen);
		OutCRLF();
		UART_OutString("yGreen: ");
		UART_OutUDec(yGreen);
		OutCRLF();
		UART_OutString("xRed: ");
		UART_OutUDec(xRed);
		OutCRLF();
		UART_OutString("yRed: ");
		UART_OutUDec(yRed);
		OutCRLF();
	
		UART_OutString("servo_basic: ");
		UART_OutUDec(servo_basic);
		OutCRLF();

}

//debug code
int main(void){

  PortA_Init();
	PortB_Init();
	PortC_Init();
	PortF_Init();
	SysTick_Init();
	UART_Init();
	UpdateServo(servo_basic);
	
  while(1){
		// Start, get UART character
		GetUART();
		ServoFeedback();

		// (x, y) green, (x, y) red coordinates
		// servo PWM [2000 , 18000]
		//ServoPID();
		//StepOut();
		//OutUART();
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

