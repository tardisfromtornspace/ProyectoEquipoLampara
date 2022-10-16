
// See /opt/microchip/xc8/v1.10/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // necessary for __delay_us

#include <xc.h>

void main(void)
{ char x;

  
  OSCCONbits.OSTS = 1; // External cristal

  x = 0;

  TRISB = 0;    // Configure port B as output

  while(1)
  { PORTB = x;
    x += 1;
    __delay_us(4998);
   } 
}
