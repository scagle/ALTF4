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
 */

#include "tm4c123gh6pm.h"
#include "UART.h"
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
	GPIO_PORTC_CR_R    |= 0x80;  		  //allow changes to PC7
	GPIO_PORTC_AMSEL_R &= 0xEF;       //disable analog for PC7
	GPIO_PORTC_PCTL_R  &= 0x0FFFFFFF; //set PC7 to GPIO
	GPIO_PORTC_DIR_R   |= 0x80;				//set PC7 as output
	GPIO_PORTC_AFSEL_R &= 0xEF;				//disable alt func PC7
	GPIO_PORTC_DEN_R   |= 0x80;				//digital enable PC7
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
	 NVIC_ST_CTRL_R = 0; //disable systick
	 NVIC_ST_RELOAD_R = 0x001F4240; // period
	 NVIC_ST_CURRENT_R = 0; //reset current couter value
	 NVIC_ST_CTRL_R |= 0x00000007;
 }

/***********************************************************************************/
void ADC_Init(void){   
    // AIN2 -> (PE1)  POTENTIOMETER
    // AIN9 -> (PE4)  RIGHT IR SENSOR
    // AIN8 -> (PE5)  LEFT IR SENSOR 
    volatile unsigned long delay;
    SYSCTL_RCGCADC_R     |=  0x00000001                   ; // 1) activate ADC0
    SYSCTL_RCGCGPIO_R    |=  SYSCTL_RCGCGPIO_R4           ; // 1) activate clock for Port E
    //WaitForPorts(0x10)                                    ; // small delay for GPIO
    GPIO_PORTE_DIR_R     &= ~0x32                         ; // 3) make PE1, PE4, and PE5 input
    GPIO_PORTE_AFSEL_R   |=  0x32                         ; // 4) enable alternate function on PE1, PE4, and PE5
    GPIO_PORTE_DEN_R     &= ~0x32                         ; // 5) disable digital I/O on PE1, PE4, and PE5
    GPIO_PORTE_PCTL_R     =  GPIO_PORTE_PCTL_R&0xFF00FF0F ;
    GPIO_PORTE_AMSEL_R   |=  0x32                         ; // 6) enable analog functionality on PE1, PE4, and PE5
    ADC0_PC_R            &= ~0xF                          ; // 8) clear max sample rate field
    ADC0_PC_R            |=  0x1                          ; //    configure for 125K samples/sec
    ADC0_SSPRI_R          =  0x3210                       ; // 9) Sequencer 3 is lowest priority
    ADC0_ACTSS_R         &= ~0x0004                       ; // 10) disable sample sequencer 2
    ADC0_EMUX_R          &= ~0x0F00                       ; // 11) seq2 is software trigger
    ADC0_SSMUX2_R         =  0x0892                       ; // 12) set channels for SS2
    ADC0_SSCTL2_R         =  0x0600                       ; // 13) no D0 END0 IE0 TS0 D1 END1 IE1 TS1 D2 TS2, yes EN    D2 IE2
    ADC0_IM_R            &= ~0x0004                       ; // 14) disable SS2 interrupts
    ADC0_ACTSS_R         |=  0x0004                       ; // 15) enable sample sequencer 2
}

void ADC_In(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8){   // ain2 (PE1) [0-4095]
    // ain9 (PE4) [0-4095]
    // ain8 (PE5) [0-4095]
    ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
    while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
    *ain2 = ADC0_SSFIFO2_R&0xFFF;    // 3A) read first result
    *ain9 = ADC0_SSFIFO2_R&0xFFF;    // 3B) read second result
    *ain8 = ADC0_SSFIFO2_R&0xFFF;    // 3C) read third result
    ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}

unsigned long median(unsigned long u1, unsigned long u2, unsigned long u3){
    // Median function
    unsigned long result;
    if(u1>u2)
        if(u2>u3)     result=u2; // u1>u2, u2>u3       u1>u2>u3
        else
            if(u1>u3) result=u3; // u1>u2, u3>u2,u1>u3 u1>u3>u2
            else      result=u1; // u1>u2, u3>u2,u3>u1 u3>u1>u2
    else
        if(u3>u2)     result=u2; // u2>u1, u3>u2       u3>u2>u1
        else
            if(u1>u3) result=u1; // u2>u1, u2>u3,u1>u3 u2>u1>u3
            else      result=u3; // u2>u1, u2>u3,u3>u1 u2>u3>u1
    return(result);
}

 
 

void ReadADCMedianFilter(volatile unsigned long *ain2, volatile unsigned long *ain9, volatile unsigned long *ain8){
    // This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
    // returns the results in the corresponding variables.  Some
    // kind of filtering is required because the IR distance sensors
    // output occasional erroneous spikes.  This is a median filter:
    // y(n) = median(x(n), x(n-1), x(n-2))
    // Assumes: ADC initialized by previously calling ADC_Init()
    //                   x(n-2)        x(n-1)
    static unsigned long ain2oldest=0, ain2middle=0;
    static unsigned long ain9oldest=0, ain9middle=0;
    static unsigned long ain8oldest=0, ain8middle=0;
    // save some memory; these do not need to be 'static'
    //            x(n)
    unsigned long ain2newest;
    unsigned long ain9newest;
    unsigned long ain8newest;
    ADC_In(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
    *ain2 = median(ain2newest, ain2middle, ain2oldest);
    *ain9 = median(ain9newest, ain9middle, ain9oldest);
    *ain8 = median(ain8newest, ain8middle, ain8oldest);
    ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
    ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}


/***********************************************************************************/ 
void LED_on(){
	PWM1_1_CMPB_R = 40000;

}
void LED_off(){
	PWM1_1_CMPB_R = 0;
}
void LED_max(){
	PWM1_1_CMPB_R = 8000;//min

}

void LED(int x){
	PWM1_1_CMPB_R = x;
}


// 0^    -   180^
// 8000  -  40000
/***********************************************************************************/
void SysTick_Handler(void){
	
	//GPIO_PORTA_DATA_R = GPIO_PORTA_DATA_R ^ 0x80;	
	
}
 
void GPIOPortF_Handler(void){
	 if(GPIO_PORTF_DATA_R & 0x01){//left button
		 GPIO_PORTC_DATA_R  = 0x7F;
		 
	 }
	 if(GPIO_PORTF_DATA_R & 0x10){//right button
		 GPIO_PORTC_DATA_R  = 0x80;
		 
	 }
	 
	 GPIO_PORTF_ICR_R = 0x11; //acknowledge interrupt
	 
 }
 

/***********************************************************************************/
//Change 3 bit outputs to stepper motor driver
void step_speed(int val){//PB 7,6,5
	if(val >=  0 & val <800){// 1/16
		//GPIO_PORTF_DATA_R = 0;//off
		GPIO_PORTB_DATA_R &= 0x1F;
		GPIO_PORTB_DATA_R |= 0xE0;		
	}
	
	if(val >=800 & val <1600){// 1/8
		//GPIO_PORTF_DATA_R = 0x02;//red
		GPIO_PORTB_DATA_R &= 0x1F;
		GPIO_PORTB_DATA_R |= 0x60;
	}
	
	if(val >=1600 & val <2400){// 1/4
		//GPIO_PORTF_DATA_R = 0x04;//blue
		GPIO_PORTB_DATA_R &= 0x1F;
		GPIO_PORTB_DATA_R |= 0x40;		
	}
	if(val >=2400 & val <3200){// 1/2
		//GPIO_PORTF_DATA_R = 0x08;//green
		GPIO_PORTB_DATA_R &= 0x1F;
		GPIO_PORTB_DATA_R |= 0x20;	
	}
	
	if(val >=3200){// 1/1
		//GPIO_PORTF_DATA_R = 0x0A;//yellow
		GPIO_PORTB_DATA_R &= 0x1F;
	}
} 

void step_dir(int val){//PF1
	if(val >= 0 & val < 1850){//Rotate Left
		//Clear
		GPIO_PORTC_DATA_R  = 0x7F;//Rotate Left
		PWM1_2_CMPB_R  		 = 800000-1;
	}
	if(val >= 1850 & val < 2250){//don't rotate
		PWM1_2_CMPB_R  		 = 0; //off
	}
	if(val >=2250){//led off, right
		//Set
		GPIO_PORTC_DATA_R  = 0x80;//Rotate Right
		PWM1_2_CMPB_R 		 = 800000-1;
	}
}


// Returns a clock cycle count requirement in order to acheive PWM duty cycles 
// Converts (0-4095) to 5% - 10% duty cycle (or 8000-40000 clock cycles)
unsigned int getServoPosition(volatile unsigned long *ADC_Servo_Value){
    //double Add_Value = 2.44;
    double Add_Value = 1.83;		
    return ( Add_Value * (*ADC_Servo_Value)  + 7000);//LEFT_MOST_VALUE);
}

void servo_pos(unsigned long val){
	unsigned long normalVal;
	normalVal = getServoPosition(&val);
	PWM1_1_CMPB_R = normalVal;
}


//debug code
int main(void){
	unsigned long speed, dir, pos;

  PortA_Init();
	PortB_Init();
	PortC_Init();
	PortF_Init();
	//SysTick_Init();
	ADC_Init();
	
  while(1){
		ReadADCMedianFilter(&speed, &dir, &pos);
		step_speed(speed);
		step_dir(dir);
		servo_pos(pos);
		
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

