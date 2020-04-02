#include "msp430g2553.h"
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#define NUM_MIN 30
#define NUM_SEC 60
#define CS  BIT5
#define TXD BIT2
#define RXD BIT1
#define nr  0x0D
#define MIN_RECHARGE_TANK 10

int datos[] = {1,1,1,1,0};
int SECONDS = 0;
int MINUTES = 0;
int COUNTER_CHECK_STATE = -1;
int COUNTRY_UTC = 0;
int SLEEP_WISOL=0;
int INIT_SEQUENCE = 0;
int unsigned NUM_ATTEMPTS = 16;
int unsigned UART_RECEPTION = 0;
volatile int POINTER_UART_RECEPTION = -1;
time_t number = 0;
struct tm ts;
char BUFFER_RX_UART[12];
char res[];
float FIELD_MAG_1;
float FIELD_MAG_2;

void UART_init(){
    P1SEL |= RXD + TXD ;             // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= RXD + TXD ;            // P1.1 = RXD, P1.2=TXD
    P1OUT &= RXD + TXD ;
    P1DIR &= RXD + TXD ;
    UCA0CTL1 |= UCSSEL_2;            // SMCLK
    UCA0BR0 = 104;                   // 9600 BR
    UCA0BR1 = 0x00;                  // 1MHz 115200
    UCA0CTL1 &= ~UCSWRST;             // Clear UCSWRST to enable USCI_A0
    IE2 |= UCA0TXIE;                  // Enable the Transmit interrupt
}
void MCLK_init(){
    DCOCTL = 0;                      // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;           // MCLK = 1 MHz
    DCOCTL = CALDCO_1MHZ;            // MCLK = 1 MHz
}
void GPIO_init(){
    P1DIR |= CS;
    P1OUT |= CS;
}
void ADC_init(){
    ADC10CTL0 = SREF_1 + REFON +REF2_5V+ ADC10ON + ADC10SHT_3 + ADC10IE;           //2.5V ref,Ref on,64 clocks for sample
    ADC10CTL1 = INCH_0 + ADC10DIV_7;                                        //temp sensor is at 10 and clock/4
}
void ADC2_init(){
    ADC10CTL0 = SREF_1 + REFON +REF2_5V+ ADC10ON + ADC10SHT_3 + ADC10IE;           //2.5V ref,Ref on,64 clocks for sample
    ADC10CTL1 = INCH_4 + ADC10DIV_7;                                        //temp sensor is at 10 and clock/4
}
void temp_init(){
    ADC10CTL0 = SREF_1 + REFON + ADC10ON + ADC10SHT_3 + ADC10IE ;                     //1.5V ref,Ref on,64 clocks for sample
    ADC10CTL1 = INCH_10 + ADC10DIV_3;                                       //temp sensor is at 10 and clock/4
}
void volt_init(){
    ADC10CTL0 = SREF_1 + REFON + REF2_5V + ADC10ON + ADC10SHT_3 + ADC10IE ;                     //1.5V ref,Ref on,64 clocks for sample
    ADC10CTL1 = INCH_11 + ADC10DIV_3;                                       //temp sensor is at 10 and clock/4
}
void TA1_init(){
    TA1CTL  = TASSEL_1 + ID_3+ TACLR + MC_0;     // SMCLK, upmode
    TA1CCR0 = 2185;        // PWM Period
    TA1CCTL0 |= CCIE;
}
int tempOut(){
    int t=0;
    ADC10CTL0 |= ENC + ADC10SC;                                             //enable conversion and start conversion
    __low_power_mode_3();                                                   // entramos en modo LPM3
    __no_operation();
    t=ADC10MEM;                                                             //store val in t
    ADC10CTL0 &= ~ENC;                                                      //disable adc conv
    return(int) ((t * 27069L - 18169625L) >> 16);                           //convert and pass
}
int gasOut(){
    int g=0;
    ADC10CTL0 |= ENC + ADC10SC;                                             //enable conversion and start conversion
    __low_power_mode_3();                                                   // entramos en modo LPM3
    __no_operation();
    g = ADC10MEM;
    ADC10CTL0 &= ~ENC;                                                      //disable adc conv
    return(int) (g);                                                        //pass
}
int gasOut2(){
    int g=0;
    ADC10CTL0 |= ENC + ADC10SC;                                             //enable conversion and start conversion
    __low_power_mode_3();                                                   // entramos en modo LPM3
    __no_operation();
    g = ADC10MEM;
    ADC10CTL0 |= ENC + ADC10SC;                                             //enable conversion and start conversi                                                    //disable adc conv
    return(int) (g);                                                        //pass
}
int voltOut(){
    int v=0;
    ADC10CTL0 |= ENC + ADC10SC;                                             //enable conversion and start conversion
    __low_power_mode_3();                                                   // entramos en modo LPM3
    __no_operation();
    v = ((ADC10MEM * 50)/102.3);
    ADC10CTL0 &= ~ENC;                                                      //disable adc conv

    return(int) (v );
}
void printLn(char *TxArray, int ArrayLength){
    while(ArrayLength--){             // Loop until StringLength == 0 and post decrement
        UCA0TXBUF = *TxArray++;           //Write the character at the location specified by the pointer and post increment
        __low_power_mode_3();                                                   // entramos en modo LPM3
        __no_operation();
    }
    UCA0TXBUF = nr;
    __low_power_mode_3();                                                   // entramos en modo LPM3
    __no_operation();
}
void print(char *TxArray, int ArrayLength){
    while(ArrayLength--){             // Loop until StringLength == 0 and post decrement
        UCA0TXBUF = *TxArray++;           //Write the character at the location specified by the pointer and post increment
        __low_power_mode_3();                                                   // entramos en modo LPM3
        __no_operation();
    }
}
void delay_seg(int SECONDS){
    TA1CTL  = TASSEL_1 + ID_3+ TACLR + MC_1;     // SMCLK, upmode
    while(SECONDS--){
        __low_power_mode_3();                                                   // entramos en modo LPM3
        __no_operation();
    }
    TA1CTL  = MC_0;
}
void itoa(long unsigned int value, char* result, int base){
      // check that the base if valid
      if (base < 2 || base > 36) { *result = '\0';}

      char* ptr = result, *ptr1 = result, tmp_char;
      int tmp_value;

      do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
      } while ( value );

      // Apply negative sign
      if (tmp_value < 0) *ptr++ = '-';
      *ptr-- = '\0';
      while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
      }
}
int CHECK_PERMISSIONS_TRANSMIT(){
    int Tx = 0;
    if ((datos[0]==0) && (datos[4]==0)){
        Tx = 0; // sino se ha conectado el sensor no transmite
        MINUTES = NUM_MIN - 1;  // Bajar el contador para verificar al minuto nuevamente
    }
    else if((datos[0]==0) && (datos[4]!=0)){
        Tx=1;  // si estaba Tx y se desconecto
        MINUTES = NUM_MIN - 1;  // Bajar el contador para verificar al minuto nuevamente
    }
    else if((datos[4]==0) || (datos[0]-datos[4] >= MIN_RECHARGE_TANK) || (COUNTER_CHECK_STATE==NUM_ATTEMPTS-1)){
        Tx=1;  // si hubo medida O hubo recarga O toca transmitir por tiempo
    }
    datos[4]=datos[0];
    if (datos[0]==0){
       COUNTER_CHECK_STATE = -1;
    }
    else if ((COUNTER_CHECK_STATE==NUM_ATTEMPTS-1) || (COUNTER_CHECK_STATE==-1)){
       COUNTER_CHECK_STATE=0;
    }else{
       COUNTER_CHECK_STATE++;
    }
    return Tx;
}
int CHECK_DISCONNECT(){
    int POINTER=0;
    int VOLT_OUT=2.046*datos[3];
    int VAR_TEMP[]={1,1};
    VAR_TEMP[0]=fabs(datos[1]-datos[0]);
    VAR_TEMP[1]=fabs(VOLT_OUT-datos[1]);
    if ((VAR_TEMP[0]<=7) && (VAR_TEMP[1]<35)){
        POINTER=1;
    }
    return POINTER;
}
void TX_RADIO_WISOL(){
    printLn("",0);  //Power on after sleep
    delay_seg(1);
    printLn("AT$RC", 5);  //Reset Channel
    delay_seg(12);
    print("AT$SF=", 6);   //TX frame
    ////Porcentaje gas////////////
    itoa(datos[0],res,10);
    if (datos[0]==0){
        print(res,1);
        print(res,1);
    }else{
        print(res, 2);
    }
    ////Temperatura//////////////
    if ((datos[2] < 10)&&(datos[2]>=0)){
       itoa(datos[2],res,10);
       print("0",1);  //Si datos de un solo digito TX un 0 antes
       print(res,1);
    }
    else if (datos[2] < 0){  //Si es temperatura bajo 0
       if (datos[2]<-9){    // Si supera los tres digitos
           datos[2]=-9;     // Limitar a solo dos digitos (Rango minimo)
       }
       datos[2]=datos[2]*-1;
       itoa(datos[2],res,10);
       print("b",1);    // Cambiar el signo menos por b (bajo)
       print(res,1);
    }
    else{
       itoa(datos[2],res,10);
       print(res, 2);
    }
    if (INIT_SEQUENCE != 0){
    ///Voltaje de la bateria///////
       itoa(datos[3],res,10);
       printLn(res, 2);
       delay_seg(2);
    }else{
       itoa(datos[3],res,10);
       print(res, 2);
       printLn(",1",2);
       ///Activar recepcion UART
       UC0IE |= UCA0RXIE;
       delay_seg(100);
       UC0IE &= ~ UCA0RXIE;
    }
    ////Dormir el radio //////
    printLn("AT$P=1",6);
}
void SLEEP_WISOL_init(){
    if (SLEEP_WISOL==0){
        SLEEP_WISOL=1;
        delay_seg(1);
        printLn("AT$P=1",6);
    }
}
void CHECK_RECEPTION_OK(){
    int i;
    UART_RECEPTION = 1;
    for (i=11;i>=0;i--){
        if ((BUFFER_RX_UART[i] >= '0') & (BUFFER_RX_UART[i] <= 'F')){
            UART_RECEPTION = 0;
        }
    }
}
int unsigned PUT_SEQUENCE_AND_COUNTRY_UTC_NUMBER(){
    int unsigned intval = NUM_ATTEMPTS;
    if (UART_RECEPTION == 0){
        intval = (BUFFER_RX_UART[8] >= 'A') ? (BUFFER_RX_UART[8] - 'A' + 10) : (BUFFER_RX_UART[8] - '0');
        intval = intval*16 + ((BUFFER_RX_UART[9] >= 'A') ? (BUFFER_RX_UART[9] - 'A' + 10) : (BUFFER_RX_UART[9] - '0'));
        COUNTRY_UTC = (BUFFER_RX_UART[10] >= 'A') ? (BUFFER_RX_UART[10] - 'A' + 10) : (BUFFER_RX_UART[10] - '0');
        char timeraw[8];
        unsigned int i;
        for (i=0 ; i<=7 ; i++){
             timeraw[i] = BUFFER_RX_UART[i];
        }
        number = (time_t)strtol(timeraw, NULL, 16);
        ts = *localtime(&number);
        SECONDS = ts.tm_sec;
    }
    if (intval == 0){
        intval = NUM_ATTEMPTS;
    }
    return intval;
}

void PUT_TIME(){
    int hora_recv;
    if (UART_RECEPTION == 0){
         int hora = (BUFFER_RX_UART[11] >= 'A') ? (BUFFER_RX_UART[11] - 'A' + 10) : (BUFFER_RX_UART[11] - '0');
         number = number + NUM_SEC * 5;
         ts = *localtime(&number);
         hora_recv = ts.tm_hour-COUNTRY_UTC;
         if (ts.tm_hour < COUNTRY_UTC){
              hora_recv = 24 + hora_recv;
         }
         MINUTES = (ts.tm_min) % NUM_MIN;
         SECONDS = SECONDS + 16;
         int dif =(hora-hora_recv)*60-ts.tm_min;
         if (dif < 0){
             dif = 1440 + dif;
         }
         dif = dif %(NUM_ATTEMPTS*NUM_MIN);
         COUNTER_CHECK_STATE = NUM_ATTEMPTS - 1 - (int)(dif/NUM_MIN);
    }
}

void LOOP_CONNECTION_OK_INIT(){
    if ( INIT_SEQUENCE == 1){
        PUT_TIME();
    }
    if ( INIT_SEQUENCE == 0 ){
        CHECK_RECEPTION_OK();
        NUM_ATTEMPTS = PUT_SEQUENCE_AND_COUNTRY_UTC_NUMBER();
        COUNTER_CHECK_STATE=NUM_ATTEMPTS-1;
        MINUTES = NUM_MIN - 5;
        INIT_SEQUENCE = 1;
    }else{
        INIT_SEQUENCE = datos[0];
    }
}
float LOOP_RELIABILITY(int sensor){
   float level;
   if((sensor <=18) || (sensor >86)){
       level=1;
   }else if(sensor<=25){
       level=0.65;
   }else if(sensor<=32){
       level=0.5;
   }else if(sensor<=50){
       level=0.3;
   }else if(sensor<=81){
       level=0.03;
   }else if(sensor<85){
       level=0.15;
   }else{
       level=0.8;
   }
   return level;
}

int CHOICE_VALOR(int s1,int s2){
    float level1=LOOP_RELIABILITY(s1);
    float level2=1-LOOP_RELIABILITY(s2);
    return (int)rint((level1*s1+level2*s2)/(level1+level2));
}

void CALCULATE_MEASUREMENT(){
    float S11[]={8.50,7.07,5.13,3.32,1.35,-0.35,-2.12,-3.72,-5.05,-6.21,-7.31,-8.20,-8.80,-9.13,-8.86,-7.77,-4.83,-0.12};
    float S12[]={12.6,12.4,11.3,9.9,8.45,6.7,4.64,2.48,0.3,-1.7,-3.87,-6,-7.8,-9.56,-10.93,-12.24,-13.25,-12.6};
    FIELD_MAG_1 = 20.47*(2*(2.5*datos[0])/(datos[3]*10.23)-1);
    FIELD_MAG_2 = 20.47*(2*(2.5*datos[1])/(datos[3]*10.23)-1);
    int i,ind1,ind2,ind3;
    ind2 = 17;
    for (i=ind2;i>-1;i--){
        if (S11[i] < FIELD_MAG_1){
            ind2 = i;
            break;
        }
    }
    ind1=0;
    for (i=0;i<18;i++){
        if (S11[i] < FIELD_MAG_1){
            ind1 = i - 1;
            break;
        }
    }
    ind3=0;
    for (i=0;i<18;i++){
        if (S12[i] < FIELD_MAG_2){
            ind3 = i - 1;
            break;
        }
    }
    if ((ind1 == -1) && (ind2 == 17) && (FIELD_MAG_2>0)){
        datos[0]=10;
        datos[1]=10;
    }else if((ind1 == 0) && (ind2 == 17) && (FIELD_MAG_2<0)){
        ind1=0;
        float resta;
        resta=S11[ind1]-FIELD_MAG_1;
        for (i=1;i<18;i++){
           if (fabs(S11[i]-FIELD_MAG_1)<fabs(resta)){
               resta = S11[i]-FIELD_MAG_1;
               ind1=i;
           }
        }
        datos[0]=10+ind1*5;
        if (ind3==0){
            datos[1] = 90;
        }else{
            datos[1] = (int)rint(10 + ind3*5+((5)/(S12[ind3+1]-S12[ind3]))*(FIELD_MAG_2-S12[ind3]));
        }
    }else{
        ind2=(int)rint(10 + ind2*5+((5)/(S11[ind2+1]-S11[ind2]))*(FIELD_MAG_1-S11[ind2]));
        datos[0] = (int)rint(10 + ind1*5+((5)/(S11[ind1+1]-S11[ind1]))*(FIELD_MAG_1-S11[ind1]));
        if ((ind3 == 0) && (FIELD_MAG_2<0)){
            datos[1]=90;
        }else{
            datos[1] = (int)rint(10 + ind3*5+((5)/(S12[ind3+1]-S12[ind3]))*(FIELD_MAG_2-S12[ind3]));
        }
        if (fabs(ind2 - datos[1]) < fabs(datos[1] - datos[0])){
            if ((datos[1]<= 80) && (fabs(ind2-datos[1])>5)){
                datos[0]=ind2-fabs(ind2-datos[1])+5;
            }else{
                datos[0]=ind2;
            }
        }
    }
    datos[0]=CHOICE_VALOR(datos[0],datos[1]);
}

void main(void)
    {
    WDTCTL =  WDTPW + WDTHOLD;//Stop the WDT to prevent reset
    WDTCTL =  WDT_ADLY_1000;  //using ACLK from the LFXT1CLK
    IE1   |=  WDTIE;          //Enable the WDTIE bit
    TA1_init();
    MCLK_init();
    UART_init();
    GPIO_init();
    _BIS_SR( GIE); //Go to LPM3 with interrupts enabled
    while (1)
    {
        __low_power_mode_3();        // entramos en modo LPM3
        __no_operation();
        __delay_cycles(1000);
        temp_init();
        datos[2]=tempOut();
        temp_init();
        datos[2]=tempOut();
        volt_init();
        datos[3]=voltOut();
        P1OUT &= ~CS;
        delay_seg(1);
        ADC_init();
        datos[0]=gasOut();
        ADC2_init();
        datos[1]=gasOut2();
        if (CHECK_DISCONNECT() == 0){
            CALCULATE_MEASUREMENT();
        }else{    // Si es 1 esta mal conectado
            datos[0]=0;
        }
        ADC10CTL0 &= ~(REFON + ADC10ON);
        P1OUT |= CS;
        if (CHECK_PERMISSIONS_TRANSMIT() == 1){
            TX_RADIO_WISOL();
            SLEEP_WISOL=1;
            LOOP_CONNECTION_OK_INIT();
        }
        SLEEP_WISOL_init();
    }
}
#pragma vector = ADC10_VECTOR
__interrupt void adc10_interrupt(void)
{
    while (ADC10CTL1 & BUSY);
    __bic_SR_register_on_exit(CPUOFF);
}
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    __bic_SR_register_on_exit(CPUOFF);
    IFG2 &= ~UCA0TXIFG; // Clear RX flag
}
#pragma vector = WDT_VECTOR  //Interval timer vector location
__interrupt void IntervalTimer(void)
{
  if ((MINUTES < 0) || (MINUTES > NUM_MIN)){
        MINUTES = 0;
  }
  if ((SECONDS < 0) || (SECONDS > NUM_SEC)){
      MINUTES = 0;
  }
  if (SECONDS == NUM_SEC){
      SECONDS = 0;
      MINUTES ++;
  }
  if (MINUTES == NUM_MIN){
      SECONDS = 1;
      MINUTES = 0;
      __bic_SR_register_on_exit(CPUOFF);
  }
  else{
      SECONDS++;
  }
}
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A1(void)
{
    __bic_SR_register_on_exit(CPUOFF);
}
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    if (POINTER_UART_RECEPTION >= 0){
        if (UCA0RXBUF != ' '){
            BUFFER_RX_UART[POINTER_UART_RECEPTION]=UCA0RXBUF;
            POINTER_UART_RECEPTION++;
        }
        if (POINTER_UART_RECEPTION == 12){
            UC0IE &= ~ UCA0RXIE;
        }
    }
    if (UCA0RXBUF == '='){
        POINTER_UART_RECEPTION = 0;
    }
    __bic_SR_register_on_exit(CPUOFF);
    IFG2 &= ~UCA0RXIFG;
}
