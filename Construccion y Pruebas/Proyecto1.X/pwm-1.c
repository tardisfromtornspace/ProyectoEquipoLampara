

// See /opt/microchip/xc8/v{ver}/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // necessary for __delay_us

#include <xc.h>
#include <stdio.h>

int continuar = 0;
int contado1 = 15180; // TMR1
unsigned char eltimer1H = 0x3b;
unsigned char eltimer1L = 0x4d; // 0x4c +1
int valor = 0;

char porcentajeMin = 0; // Min duty cycle
char porcentajeMax = 100; // Max duty cycle
char porcentajeSubenBaja = 1; // 1 sube, 0 Baja

char porcentaje = 0; // Porcentaje del duty cycle
char porcentajeC = 100; // Porcentaje del duty cycle complementario

char elPR2 = 167; // para 30 kHz es  Fosc/4/PR2 = 30 kHz => PR2 = 167 < 255

char x;
int copias = 0; // De acuerdo al profesor, empleamos varias esperas de 5 ms ya que el TMR0 no puede aguantar hasta 1 s. 1s : 5 ms = 200 veces
int los5msen1s = 5; // En TMR1 con 1:8 (el maximo preescalado) salen 0.5 : (8 * 2E-7) = 312500 > 65536; 312500 : 65536 = 4 ciclos, resto 50356, por lo tanto, 65536 - 50356 = 15180
// Con TMR0 Medio segundo eran 100, 1 s son 200
// En caso de emplear la mejora de 1:256 que serÃ­a un ciclo de 10 ms, esto seria 100 y no 200 (en 1 s))

void init_TMR0(void) {
    TMR0 = 61; // Hemos tenido en cuenta que al pasar de 0 al 255 eso supone un ciclo mas 
    // Posible mejora: en vez de 200 de 1:128 (200 intervalos de 5 ms) se puede usar 100 de 1:256 (100 intervalos de 10 ms) lo que reduciria la sobrecarga
    // Ante este caso, El TMR0 quedaria igual ya que (2 x 5E-3)/(2 x 128 x 2E-7) = (5E-3)/(128 x 2E-7) = 195.31 < 255 y entonces se empezazaria por el mismo 255-196 = 60 (61)
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b111; // Para la mejora se emplearia 0b111 para que (segun el datasheet del microcontrolador) sea 1:256 en vez de 1:128
    INTCONbits.T0IE = 1;
}

/*
void init_TMR1(void) {
    TMR1H = eltimer1H;
    TMR1L = eltimer1L;
    T1CONbits.TMR1CS = 0;
    //T1CONbits.PSA = 0;
    T1CONbits.T1CKPS = 0b11; 
    T1CONbits.TMR1GE = 0; // TIMER1 siempre contando
    PIE1bits.TMR1IE = 1;
    T1CONbits.TMR1ON = 1;
}
 */
void init_TMR2(void) {
    PR2 = 167;
    //TMR2 =  88; // ¿Necesario con el PR2 establecido ya a 167 y no a 255?
    T2CONbits.T2CKPS = 0b00; // TIMER2 preescalado, como para 30 kHz es  Fosc/4/PR2 = 30 kHz => PR2 = 167 < 255 basta preescalado 1:1
    T2CONbits.TOUTPS = 0b0000; // TIMER2 postescalado
    PIE1bits.TMR2IE = 1; // Permitir que la interrupcion del TIMER2 avance
    PIR1bits.TMR2IF = 0;
    T2CONbits.TMR2ON = 1;
}

void init_CCP1_PWM(void) {
    TRISC = 0; // Es un output, CCP1 es pin 10, CCP2 es pin 9, segun el manual son del PORT C
    ANSEL = 0; // Es un outpur digital
    CCP1CONbits.CCP1M = 0b1100; // Modo PWM con todo a alto
    CCP1CONbits.P1M = 0b00; // Half Bridge simple
    //CCP1CONbits.DC1B = 0b00; // Los dos bits menos significativos de la cuenta.
    CCPR1L = (porcentaje * elPR2 / 4); // Los 8 bits mas significativos para el duty cycle del PWM
    CCP1CONbits.DC1B = porcentaje * elPR2 % 4; // Los dos bits menos significativos para el duty cycle del PWM

}

void init_CCP2_PWM(void) {
    TRISC = 0; // Es un output, CCP1 es pin 10, CCP2 es pin 9, segun el manual son del PORT C
    ANSEL = 0; // Es un outpur digital
    CCP2CONbits.CCP2M = 0b1100; // Modo PWM con todo a alto
    // CCP2CONbits.P2M = 0b00; // Half Bridge simple NO EXISTE EN CCP2
    //CCP2CONbits.DC2B0 = 0; // Los dos bits menos significativos de la cuenta.
    //CCP2CONbits.DC2B1 = 0; // Los dos bits menos significativos de la cuenta.
    CCPR2L = porcentajeC * elPR2 / 4; // Los 8 bits mas significativos para el duty cycle del PWM
    CCP2CONbits.DC2B1 = (unsigned char) (( porcentajeC * elPR2 % 4) >> 1 << 1); // Los dos bits menos significativos para el duty cycle del PWM
    CCP2CONbits.DC2B0 = (unsigned char) (porcentajeC * elPR2 % 2); // Los dos bits menos significativos para el duty cycle del PWM

}
// TO-DO CCP2_PWM

/*
void init_ADC(void) {
    TRISA =1;
    ANSELbits.ANS0 = 1;
    ADCON0bits.ADCS = 0b10;
    ADCON0bits.CHS = 0b0000;
    ADCON0bits.ADON = 1;
    ADCON1bits.ADFM = 1;
    
    ADCON1bits.VCFG1 = 0;
    ADCON1bits.VCFG0 = 0;

    PIE1bits.ADIE = 1;  // Permitir interrupcion A/D   
}
 */

void __interrupt() TRAT_INT(void) {
    if (INTCONbits.TMR0IF) // Esto es TMRO, para TMR 1 seria PIR1bits.TMR1IF == 1
    { // Interrupcion del timer

        TMR0 = 61;
        if (copias >= los5msen1s) {
            copias = 0; // Reseteamos contador global
            if (porcentaje <= porcentajeMin) { // Si menor o igual a 0 hay que subir
                porcentajeSubenBaja = 1;
            } else {
                if (porcentaje >= porcentajeMax) { // Si mayor a igual a 100 hay que bajar
                    porcentajeSubenBaja = 0;
                }
            }
            if (porcentajeSubenBaja == 0) {
                porcentaje -= 1;
                porcentajeC += 1;
            } else {
                porcentaje += 1;
                porcentajeC -= 1;
            }
            PORTB = porcentaje;
            // Cada 50 ms ajustar el duty cycle segun especificaciones del enunciado
            CCPR1L = porcentaje * elPR2 / 4; // Los 8 bits mas significativos para el duty cycle del PWM
            CCP1CONbits.DC1B = porcentaje * elPR2 % 4; // Los dos bits menos significativos para el duty cycle del PWM

            // Para el CCP2 lo mismo pero al revés
            CCPR2L = porcentajeC * elPR2 / 4; // Los 8 bits mas significativos para el duty cycle del PWM
            CCP2CONbits.DC2B1 = (unsigned char) ((porcentajeC * elPR2 % 4) >> 1 << 1); // Los dos bits menos significativos para el duty cycle del PWM
            CCP2CONbits.DC2B0 = (unsigned char) (porcentajeC * elPR2 % 2); // Los dos bits menos significativos para el duty cycle del PWM
            
            //TO-DO PORTC como lo sacamos?
        } else {
            copias += 1;
        }


        INTCONbits.T0IF = 0; // Evitar que vuelva a entrar en la interrupcion tras salir
        /*
        if (copias >= los5msen1s)
        {
            copias = 0;
            TMR1H = eltimer1H;
            TMR1L = eltimer1L;
            ADCON0bits.GO_DONE = 1;
        } else
        {
            copias += 1;
            TMR1H = 0;
            TMR1L = 0;
        }
        PIR1bits.TMR1IF = 0;  
         */
    } else {
        if (PIR1bits.TMR2IF) // Interrupcion TMR2
        {
            // TMR2 =  88; // 255 - 167 = 88
            TRISC = 0; // Segun el manual, activar el pin output driver del CCPx se hace limpiando el bit TRIS correspondiente
            PIR1bits.TMR2IF = 0; // Evitar volver a la interrupcion
        }
        /*// Interrupcion ADC
        PIR1bits.ADIF = 0;
        valor = (int) ADRESH * 0x10 + ADRESL;
        PORTB = ADRESL;
        continuar = 1;
         */
    }

}

void init_uart(void) {
    TXSTAbits.BRGH = 0;
    BAUDCTLbits.BRG16 = 0;

    // SPBRGH:SPBRG = 
    SPBRGH = 0;
    SPBRG = 32; // 9600 baud rate with 20MHz Clock

    TXSTAbits.SYNC = 0; /* Asynchronous */
    TXSTAbits.TX9 = 0; /* TX 8 data bit */
    RCSTAbits.RX9 = 0; /* RX 8 data bit */

    //PIE1bits.TXIE = 1; /* Disable TX interrupt */
    PIE1bits.TXIE = 0; /* Disable TX interrupt */
    PIE1bits.RCIE = 0; /* Disable RX interrupt */

    RCSTAbits.SPEN = 1; /* Serial port enable */

    TXSTAbits.TXEN = 0; /* Reset transmitter */
    TXSTAbits.TXEN = 1; /* Enable transmitter */
}

/* It is needed for printf */
void putch(char c) {
    while (!TXIF && !TXSTAbits.TRMT) {
        continue;

    }
    TXREG = c;
}

void main(void) {
    OSCCON = 0b00001000; // External cristal
    // OSCCONbits.OSTS = 1; // External cristal
    init_uart();
    //init_ADC();
    init_TMR0();
    //init_TMR1();
    init_TMR2();

    init_CCP1_PWM();
    init_CCP2_PWM();

    TRISB = 0; // Puerto B a output

    INTCONbits.GIE = 1; // Habilitar interrupción
    INTCONbits.PEIE = 1; // Habilitar interrupción de otros timers que no son el 0

    while (1) {
        while (!continuar);
        continuar = 0;
        printf("%d\r\n", valor);
    }
}
