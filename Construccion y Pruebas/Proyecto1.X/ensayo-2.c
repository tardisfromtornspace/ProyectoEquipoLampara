
// See /opt/microchip/xc8/v1.10/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // necessary for __delay_us

#include <xc.h>

char x;
void init_TMR0 (void)
{
    TMR0 = 61;
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b110;
    INTCONbits.T0IE = 1;
}

void __interrupt() TRAT_INT (void)
{
    TMR0 = 61;
    PORTB = x;
    x += 1;
    INTCONbits.T0IF = 0;
}


void main(void)
{ 

    init_TMR0();
    INTCONbits.GIE = 1;
  OSCCONbits.OSTS = 1; // External cristal

  x = 0;

  TRISB = 0;    // Configure port B as output

  while(1);
}
