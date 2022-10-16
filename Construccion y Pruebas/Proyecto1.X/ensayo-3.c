
// See /opt/microchip/xc8/v1.10/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // necessary for __delay_us

#include <xc.h>

char x;
int copias = 0; // De acuerdo al profesor, empleamos varias esperas de 5 ms ya que el TMR0 no puede aguantar hasta 1 s. 1s : 5 ms = 200 veces
int los5msen1s = 200; // En caso de emplear la mejora de 1:256 que sería un ciclo de 10 ms, esto seria 100 y no 200

void init_TMR0 (void)
{
    TMR0 = 61; // Hemos tenido en cuenta que al pasar de 0 al 255 eso supone un ciclo mas 
    // Posible mejora: en vez de 200 de 1:128 (200 intervalos de 5 ms) se puede usar 100 de 1:256 (100 intervalos de 10 ms) lo que reduciria la sobrecarga
    // Ante este caso, El TMR0 quedaria igual ya que (2 x 5E-3)/(2 x 128 x 2E-7) = (5E-3)/(128 x 2E-7) = 195.31 < 255 y entonces se empezazaria por el mismo 255-196 = 60 (61)
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b110; // Para la mejora se emplearia 0b111 para que (segun el datasheet del microcontrolador) sea 1:256 en vez de 1:128
    INTCONbits.T0IE = 1;
}

void __interrupt() TRAT_INT (void)
{
    TMR0 = 61;
    if (copias >= los5msen1s) {
        PORTB = x;
        x += 1;
        copias = 0;
    } else {
       copias += 1;
    }
    INTCONbits.T0IF = 0;
}


void main(void)
{ 

  init_TMR0();
  INTCONbits.GIE = 1; // Activamos
  OSCCONbits.OSTS = 1; // External cristal

  x = 0;

  TRISB = 0;    // Configure port B as output

  while(1);
}
