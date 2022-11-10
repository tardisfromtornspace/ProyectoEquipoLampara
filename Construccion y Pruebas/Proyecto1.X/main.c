

// See /opt/microchip/xc8/v{ver}/docs/chips/16f886.html
// for details on #pragma config

#pragma config CPD = OFF, BOREN = OFF, IESO = OFF, DEBUG = OFF, FOSC = HS
#pragma config FCMEN = OFF, MCLRE = ON, WDTE = OFF, CP = OFF, LVP = OFF
#pragma config PWRTE = ON, BOR4V = BOR21V, WRT = OFF

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // necessary for __delay_us

#include <xc.h>
#include <stdio.h>

#include <i2c-v2.h>
#include <spi-master-v1.h>

typedef enum {
    FALSE = 0b0, TRUE = 0b1
} boolean;

int deboContinuar = 1; // Ponerla a 0 detiene nuestro programa
int continuar = 0;
int continuoEscribiendo = 0;
int reciboLED = 0; // 0 -> NO, 1 es Lumen, 2 es B, 3 es G, 4 es R
char miLED[] = {255, 255, 255, 31}; // Inicio con valor por defecto
int reciboPWM = 0; // �Estoy recibiendo datos para mi PWM? no = 0, s� = 1;
int emitirMisSensores = 0; // �Me piden el sensor? 0 = No, 1 = S�
char emitoSensores[];

int contado1 = 15180; // TMR1
unsigned char eltimer1H = 0x3b;
unsigned char eltimer1L = 0x4d; // 0x4c +1
int valor[] = {0, 0, 0}; //Entradas Ruido, Humedad, Temperatura
int anI = 0; // 0 es Ruido, 1 es Humedad, 2 es Temperatura

char direccionInicial = 0; //Direcci�n inicial de la memoria de datos
char direccionPWM = 1; //Direcci�n del valor del duty cycle en la memoria de datos
char direccionLED = 2; //Direcci�n del valor de ledes en la memoria de datos. Offset: r = 0, g = 1, b = 2, lum = 3;

int pPWM, r, g, b, lum; // Almacenados en memoria vol�til lo que sacamos de la flash

int numLedes = 10; // Tira de 10 ledes

char porcentajeMin = 0; // Min duty cycle
char porcentajeMax = 167; // Max duty cycle
char porcentajeSubenBaja = 1; // 1 sube, 0 Baja

char porcentaje = 0; // Porcentaje del duty cycle
char porcentajeC = 167; // Porcentaje del duty cycle complementario

char elPR2 = 167; // para 30 kHz es  Fosc/4/PR2 = 30 kHz => PR2 = 167 < 255

char x;
int copias1 = 0;
int copias = 0; // De acuerdo al profesor, empleamos varias esperas de 5 ms ya que el TMR0 no puede aguantar hasta 1 s. 1s : 5 ms = 200 veces
int los5msen1s = 4; // En TMR1 con 1:8 (el maximo preescalado) salen 0.5 : (8 * 2E-7) = 312500 > 65536; 312500 : 65536 = 4 ciclos, resto 50356, por lo tanto, 65536 - 50356 = 15180
// Con TMR0 Medio segundo eran 100, 1 s son 200
// En caso de emplear la mejora de 1:256 que sería un ciclo de 10 ms, esto seria 100 y no 200 (en 1 s))

void init_TMR0(void) {
    TMR0 = 61; // Hemos tenido en cuenta que al pasar de 0 al 255 eso supone un ciclo mas 
    // Posible mejora: en vez de 200 de 1:128 (200 intervalos de 5 ms) se puede usar 100 de 1:256 (100 intervalos de 10 ms) lo que reduciria la sobrecarga
    // Ante este caso, El TMR0 quedaria igual ya que (2 x 5E-3)/(2 x 128 x 2E-7) = (5E-3)/(128 x 2E-7) = 195.31 < 255 y entonces se empezazaria por el mismo 255-196 = 60 (61)
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b111; // Para la mejora se emplearia 0b111 para que (segun el datasheet del microcontrolador) sea 1:256 en vez de 1:128
    INTCONbits.T0IE = 1;
}

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

void init_TMR2(void) {
    PR2 = 167;
    //TMR2 =  88; // �Necesario con el PR2 establecido ya a 167 y no a 255?
    T2CONbits.T2CKPS = 0b00; // TIMER2 preescalado, como para 30 kHz es  Fosc/4/PR2 = 30 kHz => PR2 = 167 < 255 basta preescalado 1:1
    T2CONbits.TOUTPS = 0b0000; // TIMER2 postescalado
    PIE1bits.TMR2IE = 1; // Permitir que la interrupcion del TIMER2 avance
    PIR1bits.TMR2IF = 0;
    T2CONbits.TMR2ON = 1;
}

void init_CCP1_PWM(void) {
    // TRISC = 0; // Es un output, CCP1 es pin 10, CCP2 es pin 9, segun el manual son del PORT C
    TRISCbits.TRISC2 = 0;
    ANSEL = 0; // Es un output digital
    CCP1CONbits.CCP1M = 0b1100; // Modo PWM con todo a alto
    CCP1CONbits.P1M = 0b00; // Half Bridge simple
    //CCP1CONbits.DC1B = 0b00; // Los dos bits menos significativos de la cuenta.
    CCPR1L = porcentaje; // Los 8 bits mas significativos para el duty cycle del PWM
    //CCP1CONbits.DC1B = porcentaje * elPR2 % 4; // Los dos bits menos significativos para el duty cycle del PWM

}

void init_CCP2_PWM(void) {
    // TRISC = 0; // Es un output, CCP1 es pin 10, CCP2 es pin 9, segun el manual son del PORT C
    TRISCbits.TRISC1 = 0;
    ANSEL = 0; // Es un output digital
    CCP2CONbits.CCP2M = 0b1100; // Modo PWM con todo a alto
    // CCP2CONbits.P2M = 0b00; // Half Bridge simple NO EXISTE EN CCP2
    //CCP2CONbits.DC2B0 = 0; // Los dos bits menos significativos de la cuenta.
    //CCP2CONbits.DC2B1 = 0; // Los dos bits menos significativos de la cuenta.
    CCPR2L = porcentajeC; // Los 8 bits mas significativos para el duty cycle del PWM
    //CCP2CONbits.DC2B1 = (unsigned char) (( porcentajeC * elPR2 % 4) >> 1 << 1); // Los dos bits menos significativos para el duty cycle del PWM
    //CCP2CONbits.DC2B0 = (unsigned char) (porcentajeC * elPR2 % 2); // Los dos bits menos significativos para el duty cycle del PWM

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

    PIE1bits.TXIE = 1; /* Enable TX interrupt */
    PIE1bits.RCIE = 1; /* Enable RX interrupt */

    RCSTAbits.SPEN = 1; /* Serial port enable */

    TXSTAbits.TXEN = 0; /* Reset transmitter */
    TXSTAbits.TXEN = 1; /* Enable transmitter */
}

void init_I2C(void) {
    //TODO Configuracion adecuada
    // Seg�n el manual, debemos establecer SCL y SDA  a input para que hagan una NOR cableada y se pueda callar al master
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    // El SCL es el reloj de control y somos el master
    // El SDA es el serial data que solo nosotros recibimos - input
    SSPCONbits.SSPM = 0; // Somos el master con Fosc/4
    SSPCONbits.SSPM = 0; // Seg�n el manual para habilitarlo debemos hacer un set y reset de los bit SSPM

    ANSEL = 0; // Es digital
    /*
    In I2C Master mode, the reload value for the BRG is
    located in the lower 7 bits of the SSPADD register
    (Figure 13-11). When the BRG is loaded with this
    value, the BRG counts down to 0 and stops until
    another reload has taken place. The BRG count is
    decremented twice per instruction cycle (TCY ) on the
    Q2 and Q4 clocks. In I2C Master mode, the BRG is
    reloaded automatically. If clock arbitration is taking
    place, for instance, the BRG will be reloaded when the
    SCL pin is sampled high.
     
    Por consiguiente, no debemos restablecer el BBRG a SSPADD, solo decir SSPADD 
     */
    SSPADD = (_XTAL_FREQ / 4) - 1; // El ratio de baudios, si SSPM = 0 seg�n el manual: SSPADD + 1 = Fosc/4 ;

    PIR1bits.SSPIF = 0; //A�n no activamos la flag
    PIE1bits.SSPIE = 1; // Habilitamos interrupciones del I2C, necesario si somos master
    /*� 2009 Microchip Technology Inc. DS41291F-page 193
    PIC16F882/883/884/886/887
    13.4.3 MASTER MODE
    Master mode of operation is supported by interrupt
    generation on the detection of the Start and Stop
    conditions. The Stop (P) and Start (S) bits are cleared
    from a Reset, or when the MSSP module is disabled.
    Control of the I2C bus may be taken when the P bit is
    set, or the bus is idle, with both the S and P bits clear.
    In Master mode, the SCL and SDA lines are manipulated by the MSSP hardware.
    The following events will cause SSP Interrupt Flag bit, SSPIF, to be set (SSP Interrupt if enabled):

     * Start condition
     * Stop condition
     * Data transfer byte transmitted/received
     * Acknowledge transmit
     * Repeated Start condition 
 
     */
}

void init_SPI(void) {
    //TODO Configuracion adecuada
    // El SCK ya se inicializ� con el I2C, pero en el SPI deber�a ser output para ser master, pero por colis�n no se hace.
    // VAMOS A SIMULARLO POR I2C, as� que no queda tan definido
    // El SDI Es input como el de i2c, pero colisiona con el de i2c
    //TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0; // Seg�n el manual es un output
}

void init_ADC(void) {
    TRISA = 1; // Son entradas
    ANSELbits.ANS0 = 1; // Entrada Ruido como anal�gica
    ANSELbits.ANS1 = 1; // Entrada Humedad como anal�gica
    ANSELbits.ANS2 = 1; // Entrada Temperatura como anal�gica
    ADCON0bits.ADCS = 0b10;
    ADCON0bits.CHS = 0b0000; // Comenzaremos por leer por la entrada 0
    ADCON0bits.ADON = 1;
    ADCON1bits.ADFM = 1;

    ADCON1bits.VCFG1 = 0;
    ADCON1bits.VCFG0 = 0;

    PIE1bits.ADIE = 1; // Permitir interrupcion A/D   
}

void init_memoria() {
    EECON1bits.EEPGD = 0; // Acceder memoria de datos
    PIR2bits.EEIF = 0; // Ahora mismo nuestra memoria no deber�a saltar;
    PIE2bits.EEIE = 1; // Permitimos que haya interrupciones por memoria
}

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
            CCPR1L = porcentaje; // Los 8 bits mas significativos para el duty cycle del PWM
            CCPR2L = porcentajeC; // Los 8 bits mas significativos para el duty cycle del PWM
        } else {
            copias += 1;
        }

        INTCONbits.T0IF = 0; // Evitar que vuelva a entrar en la interrupcion tras salir
    } else {
        if (PIR1bits.TMR1IF == 1) // TMR1
        { // Interrupcion del timer

            if (copias1 >= los5msen1s) {
                copias1 = 0;
                TMR1H = eltimer1H;
                TMR1L = eltimer1L;
                ADCON0bits.GO_DONE = 1;
            } else {
                copias1 += 1;
                TMR1H = 0;
                TMR1L = 0;
            }
            PIR1bits.TMR1IF = 0;
        } else {
            if (PIR1bits.TMR2IF) // Interrupcion TMR2
            {
                // TMR2 =  88; // 255 - 167 = 88
                TRISC = 0; // Segun el manual, activar el pin output driver del CCPx se hace limpiando el bit TRIS correspondiente
                PIR1bits.TMR2IF = 0; // Evitar volver a la interrupcion
            } else {
                if (PIR1bits.ADIF) {
                    // Interrupcion ADC
                    PIR1bits.ADIF = 0;
                    valor[anI] = (int) ADRESH * 0x10 + ADRESL;
                    anI++;
                    if (anI >= 3) anI = 0;
                    ADCON0bits.CHS = anI; // Leemos el siguiente. As�, vamos Rui-Hum-Temp-Rui
                    PORTB = ADRESL;
                    continuar = 1;
                } else if (PIR2bits.EEIF) {
                    PIR2bits.EEIF = 0;
                    continuoEscribiendo = 1; // Ahora s� puedo comprobar si la escritura est� correcta
                } else if (PIR1bits.SSPIF) { // El de I2C
                    PIR1bits.SSPIF = 0;
                    //TODO
                } else if (PIR1bits.TXIF) { // El de USART de emisi�n tty/usb0/
                    PIR1bits.TXIF = 0;
                    //TODO

                } else if (PIR1bits.RCIF) { // El de USART de recepci�n tty/usb0/
                    PIR1bits.RCIF = 0;
                    //TODO
                    if (reciboLED > 0) {
                        reciboLED--;
                        miLED[reciboLED] = RCREG;
                        if (reciboLED == 0) setLED(miLED[3], miLED[2], miLED[1], miLED[0]);
                    } else if (reciboPWM > 0) {
                        reciboPWM = 0;
                        setPWM(RCREG);
                    } else {
                        switch (RCREG) {
                            case "a":
                                emitirMisSensores = 1; // Esto inicia el comienzo del printf en el main
                                break;
                            case "b": // Hago el pong del ping
                                TXREG = 'B';
                                break;
                            case "c":
                                reciboPWM = 1;
                                break;
                            case "d":
                                //tener en cuenta buffers
                                reciboLED = 4;
                                break;
                            case "e": // Apagar
                                // TODO: hacer una flag?
                                deboContinuar = 0;
                                break;
                            default: // Error
                                TXREG = 'E';
                                break;
                        }
                    }
                } else if (PIR1bits.SSPIF) { // El de SPI (Dummy)
                    PIR1bits.SSPIF = 0;
                    //Dummy
                }

            }
        }

    }

}

/* It is needed for printf */
void putch(char c) {
    while (!TXIF && !TXSTAbits.TRMT) {
        continue;

    }
    TXREG = c;
}

/*NOTA: Estas dos son las del sensor de Luminosidad*/
void setLumen() {
    //Es de los sensores del bus I2C, responder�a por interrupci�n
}

char getLumen() {
    return "TODO"; // TODO Por hacer 
}

/*NOTA: Estas dos son las del sensor de CO2*/
void setCO2() {
    //Es de los sensores del bus I2C, responder�a por interrupci�n
}

char getCO2() {
    return "TODO"; // TODO Por hacer 
}

/*FUNCIONES DE MEMORIA*/
char leerMemoria(char direccion) {
    EECON1bits.EEPGD = 0; // Acceder memoria de datos
    //EECON1bits.WRERR = // 0 error, 1 operacion escritura completada.
    EECON1bits.WREN = 0; // 1 es para habilitar escritura, 0 para lectura
    EEDATH = 0; // Forma una direcci�n de 14 con EEADR bits cuando se usa memoria flash
    EEADRH = 0; // Solo se permiten 256 direccioens para memoria de datos
    EEADR = direccion;
    if (EECON1bits.WR == 0) // Evitamos leer durante una escritura
        EECON1bits.RD = 1; // Inicia ciclo lectura. Si se pone a 0 es que ha completado
    //EECON1bits.WR = 1; // Inicia ciclo escritura. Si se pone a 0 es que ha completado
    return EEDAT;
}

int escribirMemoria(char direccion, char dato) {
    int valorSal;
    EECON1bits.EEPGD = 0; // Acceder memoria de datos
    //EECON1bits.WRERR = // 0 error, 1 operacion escritura completada.
    EECON1bits.WREN = 1; // 1 es para habilitar escritura, 0 para lectura
    EEADRH = 0; // Sol ose permiten 256 direccioens para memoria de datos
    EEADR = direccion;
    if (EECON1bits.RD == 0) { // Evitamos escribir durante una lectura
        int contador = 0;
        int confirmado = 0;
        while (contador < 10 && confirmado == 0) {
            INTCONbits.GIE = 0; // Deshabilitar interrupciones
            EEDAT = dato;
            EECON2 = 0x55;
            EECON2 = 0xAA;
            EECON1bits.WR = 1;
            INTCONbits.GIE = 1; // rehabilitar interrupci�n
            while (!continuoEscribiendo); // Esperamos a que PIR2bits.EEIF == 0; // Limpiar flag cuando la escritura est� completa
            continuoEscribiendo = 0;
            if (leerMemoria(direccion) == dato)
                confirmado = 1;
            else
                contador++;
        }
        if (contador >= 10) valorSal = 1; // Error en la escritura
        else valorSal = 0; // �xito  
    } else valorSal = 2; // Error, ya se est� realizando operacion de lectura
    return valorSal;
}

/*INICIALIZACI�N SMA_LAMP*/
void initYo(void) {
    OSCCON = 0b00001000; // External cristal
    // OSCCONbits.OSTS = 1; // External cristal
    init_uart();
    init_ADC();
    init_TMR0();
    init_TMR1();
    init_TMR2();

    init_CCP1_PWM();
    //init_CCP2_PWM();
    init_I2C();
    init_SPI();
    init_memoria();

    TRISB = 0; // Puerto B a output

    INTCONbits.GIE = 1; // Habilitar interrupci�n
    INTCONbits.PEIE = 1; // Habilitar interrupci�n de otros timers que no son el 0

}

void initActuadoresSegunMemoria(void) {
    if (getnoPrimerArranque() == FALSE) { // Si no encuentra nada, opci�n por defecto: luz blanca a m�xima intensidad y ventilador apagado
        setPWM(0);
        setLED(255, 255, 255, 31);
    } else { // Cargo configuraci�n de LED y PWM
        setPWM(leerMemoria(direccionPWM));
        setLED(leerMemoria(direccionLED), leerMemoria(direccionLED + 1 * 8), leerMemoria(direccionLED + 2 * 8), leerMemoria(direccionLED + 3 * 8));
    }
}

/*COMPROBACI�N MEMORIA DISPONIBLE*/
boolean getnoPrimerArranque() {
    char direccion = direccionInicial;
    char aux = leerMemoria(direccion);
    if (aux == NULL || aux == 0 || aux == (char) 0b00000000) return FALSE; //Es el primer arranque
    else return TRUE; // No es el primer arranque
}

/*FUNCIONES DEL PWM*/
void setPWM(int porcent) {
    porcentaje = (char) (porcent / 100);
    CCPR1L = porcentaje * porcentajeMax; // Los 8 bits mas significativos para el duty cycle del PWM. El porcentajeMax debe ser igual a la variable elPR2
    escribirMemoria(direccionPWM, porcent);
    pPWM = porcent;
}

void getPWM() {
    return porcentaje;
}
/*FUNCIONES DEL LED*/
//NOTA: TODO puede que requieran interrupci�n por emplear bus SPI

void setLED(int red, int green, int blue, int luminosidad) {
    //TO-DO cosas del SPI
    setLED((char) red, (char) green, (char) blue, (char) luminosidad);
}

void setLED(char red, char green, char blue, char luminosidad) {
    //cosas del SPI
    r = red;
    g = green;
    b = blue;
    lum = luminosidad;
    cosasSPI(r, g, b, lum);
    // Guardamos en memoria no vol�til
    escribirMemoria(leerMemoria(direccionLED), r);
    escribirMemoria(leerMemoria(direccionLED + 1 * 8), g);
    escribirMemoria(leerMemoria(direccionLED + 2 * 8), b);
    escribirMemoria(leerMemoria(direccionLED + 3 * 8), lum);
}

char *getLED() {
    return
    {
        r, g, b, lum
    };
}

void cosasSPI(char roj, char verd, char azu, char lumi) {
    int i;
    char lumo = 0b11100000 + (lumi % 32);
    spi_write_read(0x00);
    spi_write_read(0x00);
    spi_write_read(0x00);
    spi_write_read(0x00); // Starting frame
    for (i = numLedes; i = 0; i--) {
        spi_write_read(lumo);
        spi_write_read(azu);
        spi_write_read(verd);
        spi_write_read(roj);
    }
    spi_write_read(0xFF);
    spi_write_read(0xFF);
    spi_write_read(0xFF);
    spi_write_read(0xFF); // End frame

}
/*ANALISIS RUIDO*/
void analisisRuido(){
    // TODO
}
/*ANALISIS DEL RESTO DE SENSORES*/
void analisisResto(){
    //TODO
}
/*FUNCION PRINCIPAL*/
void main(void) {
    initYo();
    initActuadoresSegunMemoria();

    while (deboContinuar) {
        if (emitirMisSensores) {
            printf("A%c = %d\r\n", emitoSensores);
            //'A' + valoresSensores(); // TODO valores de los sensores
        }
        analisisRuido();
        analisisResto();
        //while (!continuar);
        //continuar = 0;
        //printf("%d = %d\r\n", anI, valor);


    }
}
