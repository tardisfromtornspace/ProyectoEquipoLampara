

// See /opt/microchip/xc8/v<version>/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

#include <xc.h>

void main(void)
{ char x;

  OSCCONbits.OSTS = 1; // External cristal

  x = 0;

  TRISB = 0; // configure PORTB as output

  while(1)
  { PORTB = x;
    x += 1;
   } 
}
