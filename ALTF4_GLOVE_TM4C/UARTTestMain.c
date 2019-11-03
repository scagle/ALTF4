/* Jesus Luciano, Steven Cagle, Alex Jong
 * 
 *
 * Test file glove
 *
 * PB0 : Transistor Control to LASER
 * PB1 : Input for
 *
 *
 * ADC Inputs
 *
 * PE1 - AIN2 - Flex Sensor Input
 * PE4 - AIN9
 * PE5 - AIN8
 *
 */

#include "tm4c123gh6pm.h"
#include "UART.h"
/***********************************************************************************/
const unsigned int MAX_ADC = 4095;

/***********************************************************************************/
 
void PortB_Init(){ unsigned long delay;
	SYSCTL_RCGC2_R    |= 0x00000002;		//activate Port B
	delay = SYSCTL_RCGC2_R;
	delay++;
	GPIO_PORTB_CR_R   |= 0x03;  		    //allow changes PB1,0
	GPIO_PORTB_AMSEL_R&= 0xFC;  			  //disable analog for PB1,0
	GPIO_PORTB_PCTL_R &= 0xFFFFFF00; 		//set PB1, 0 as GPIO
	GPIO_PORTB_DIR_R  |= 0x01;					//set PB0 as output
	GPIO_PORTB_DIR_R  &= 0xFD;					//set PB1 as  input
	GPIO_PORTB_AFSEL_R&= 0xFC;					//disable alt func PB1,0
	GPIO_PORTB_DEN_R  |= 0x03;					//digital enable PB1, 0
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

void Timer1_Init(void){
    SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
    //PeriodicTask = task;          // user function
    TIMER1_CTL_R &= ~0x00000001;  // 1) disable TIMER1A during setup
    TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
    TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
    TIMER1_TAILR_R = 1600000;    // 4) reload value
    TIMER1_TAPR_R = 0;            // 5) bus clock resolution
    TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
    TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
    NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00008000; // 8) priority 4
    // interrupts enabled in the main program after all devices initialized
    // vector number 37, interrupt number 21
    NVIC_EN0_R = 1<<21;           // 9) enable IRQ 21 in NVIC
    TIMER1_CTL_R |= 0x00000001;    // 10) enable TIMER1A
}

void BT_Init(void){ unsigned long delay;
    SYSCTL_RCGC1_R     |= SYSCTL_RCGC1_UART2; // activate UART2
    SYSCTL_RCGC2_R     |= SYSCTL_RCGC2_GPIOD; // activate port D
		delay = SYSCTL_RCGC2_R; 
		delay++;
		UART2_CTL_R        &= ~UART_CTL_UARTEN;   // disable UART
    UART2_IBRD_R        = 81;                 // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
    UART2_FBRD_R        = 24;                 // FBRD = int(0.1267 * 64 + 0.5) = 8
    // 8 bit word length (no parity bits, one stop bit, FIFOs)
    UART2_LCRH_R        = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
    UART2_CTL_R        |= UART_CTL_UARTEN;    // enable UART
    GPIO_PORTD_AFSEL_R |= 0xC0;               // enable alt funct on PD7-6
    GPIO_PORTD_DEN_R   |= 0xC0;               // enable digital I/O on PD7-6
    // configure PD7-6 as UART
    GPIO_PORTD_PCTL_R   = (GPIO_PORTD_PCTL_R&0x00FFFFFF)+0x11000000;
    GPIO_PORTD_AMSEL_R &= ~0xC0;              // disable analog functionality on PD
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
void SysTick_Handler(void){
	
	//GPIO_PORTA_DATA_R = GPIO_PORTA_DATA_R ^ 0x80;	
	
}
 
void GPIOPortF_Handler(void){
	 if(GPIO_PORTF_DATA_R & 0x01){//left button
			
		 
	 }
	 if(GPIO_PORTF_DATA_R & 0x10){//right button
		 
	 }
	 
	 GPIO_PORTF_ICR_R = 0x11; //acknowledge interrupt
 }
 

void Timer1A_Handler(void){ // 10Hz   
    if( GPIO_PORTB_DATA_R & 0x02){ // Pressure sensor has been pressed
			GPIO_PORTF_DATA_R = 0x04;
		}
		else{
			GPIO_PORTF_DATA_R = 0x02;
		}

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;  // acknowledge TIMER1A timeout
}

 /***********************************************************************************/
//debug code
int main(void){
	unsigned long flex_input=0, extra1, extra2;
	Timer1_Init();
	PortB_Init();
	PortF_Init();
	//SysTick_Init();
	ADC_Init();
	
  while(1){
		ReadADCMedianFilter(&flex_input, &extra1, &extra2);
		
		// test range of values
		// range 0 to 4095
		// no flex: ~1280
		// 		flex: ~1024
		if(flex_input > 1100){
			GPIO_PORTB_DATA_R &= 0xFE; // Turn laser off
		}
		else{
			GPIO_PORTB_DATA_R |= 0x01; // Turn laser on
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

