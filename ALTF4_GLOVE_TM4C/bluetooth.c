#include "bluetooth.h"
#include "tm4c123gh6pm.h"     // Register Definitions

#define CR              0x0D
#define LF              0x0A
#define BS              0x08

void BT_Init(void)
{
    SYSCTL_RCGC1_R     |= SYSCTL_RCGC1_UART2; // activate UART2
    SYSCTL_RCGC2_R     |= SYSCTL_RCGC2_GPIOD; // activate port D
    Delay();

    // Special Considerations since PD7 is a locked register
    GPIO_PORTD_LOCK_R  = 0x4C4F434B; // Unlock Port D registers
    GPIO_PORTD_CR_R    |= 0x80;          // Commit bit 7 to be unlocked (before its 0x7F, now its 0xFF)
    //GPIO_PORTD_LOCK_R  |= 0x00;          // Lock Port D registers

    UART2_CTL_R        &= ~UART_CTL_UARTEN;
    UART2_IBRD_R        = 8;                 
    UART2_FBRD_R        = 44;                  
    UART2_LCRH_R        = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
    UART2_CTL_R        |= UART_CTL_UARTEN;
    GPIO_PORTD_AFSEL_R |= 0xC0;               
    GPIO_PORTD_DEN_R   |= 0xC0;               
    
    GPIO_PORTD_PCTL_R   = ( GPIO_PORTD_PCTL_R & 0x00FFFFFF ) + 0x11000000;
    GPIO_PORTD_AMSEL_R &= ~0xC0;              
}

void UART2_OutCRLF(void)
{
    UART2_OutChar(CR);
    UART2_OutChar(LF);
}

void UART2_OutChar(unsigned char data)
{
    while ((UART2_FR_R & UART_FR_TXFF) != 0);
    UART2_DR_R = data;
}

void UART2_OutString(char *pt)
{
    while (*pt)
    {
        UART2_OutChar(*pt);
        pt++;
    }
}

unsigned char UART2_InChar(void)
{
    while ((UART2_FR_R & UART_FR_RXFE) != 0);
    return ((unsigned char)(UART2_DR_R&0xFF));
}

unsigned char UART2_NonBlockingInChar(void)
{
    if ((UART2_FR_R & UART_FR_RXFE) == 0)
    {
        return ((unsigned char) (UART2_DR_R & 0xFF));
    }
    else
    {
        return 0;
    }
}

void UART2_InString(char *bufPt, unsigned short max) 
{
    int length=0;
    char character;
    character = UART2_InChar();
    while (character != CR)
    {
        if (character == BS)
        {
            if (length)
            {
                bufPt--;
                length--;
                UART2_OutChar(BS);
            }
        }
        else if (length < max)
        {
            *bufPt = character;
            bufPt++;
            length++;
            UART2_OutChar(character);
        }
        character = UART2_InChar();
    }
    *bufPt = 0;
}

void Delay(void)
{
    unsigned long volatile time;
    time = 145448;              // 0.1 sec
    while (time)
    {
        time--;
    }
}

