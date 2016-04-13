/**********************************************************************

 * REVISION HISTORY:
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author          	Date      Comments on this revision
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Ramin Mirzahosseini 	09/07/2014  First release of source file
 * Arash Marzi          09/24/2014  Modifications to transmission protocol
 
 * The Processor starts with the External Crystal without PLL enabled and then the Clock is switched to PLL Mode.*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 *************************************************************************************************/

#if defined(__dsPIC33F__)
#include "p33fxxxx.h"
#elif defined(__PIC24H__)
#include "p24hxxxx.h"
#endif



#include <common.h>
#include <stdio.h>
#include <xlcd.h>
/*
//  Macros for Configuration Fuse Registers 
_FOSCSEL(FNOSC_FRC);	
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_XT);  
                                                                // Clock Switching is enabled and Fail Safe Clock Monitor is disabled
                                                                // OSC2 Pin Function: OSC2 is Clock Output
                                                                // Primary Oscillator Mode: XT Crystanl


_FWDT(FWDTEN_OFF);              // Watchdog Timer Enabled/disabled by user software
                                                                // (LPRC can be disabled by clearing SWDTEN bit in RCON register
 */

/* Configuration Bit Settings */
_FOSCSEL(FNOSC_PRI)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON)
_FWDT(FWDTEN_OFF)
_FPOR(FPWRT_PWR128)
_FICD(ICS_PGD1 & JTAGEN_OFF)




//#define Generator_Run          
#define Relay1          LATAbits.LATA2
#define Relay2          LATGbits.LATG2
#define Relay3          LATGbits.LATG3
#define SW0          LATEbits.LATE0 
#define SW1          LATEbits.LATE1 
#define SW2          LATEbits.LATE2 
#define SW3          LATEbits.LATE3 
#define SW4          LATEbits.LATE4 
#define SW5          LATEbits.LATE5 
#define SW6          LATEbits.LATE6 
#define Charger_Connect          LATEbits.LATE7 

#define Key_Enter          PORTGbits.RG13 //Key0
#define Key_Return          PORTGbits.RG12 //Key1
#define Key_Right          PORTGbits.RG14 //Key2
#define Key_Left          PORTCbits.RC1   //Key3
#define Key_Down          PORTCbits.RC2	  //Key4
#define Key_Up          PORTCbits.RC3     //Key5


#define Gen_Run       PORTDbits.RD14


#define DIP0          PORTAbits.RA0 
#define DIP1          PORTBbits.RB1 
#define DIP2          PORTBbits.RB2 
#define DIP3          PORTBbits.RB3

#define CC3_to_Dspic          PORTFbits.RF6



#define Dspic_to_CC3          LATAbits.LATA5

#define LED1         LATDbits.LATD11 
#define LED2         LATCbits.LATC13 
#define LED3         LATCbits.LATC14
#define LED4         LATDbits.LATD12

#define Fan_ON         LATDbits.LATD13


#define Gen_Connect         LATGbits.LATG0 
#define Util_Connect         LATGbits.LATG1 
#define Batt_Connect         LATFbits.LATF1




int i;
int ii;


char tempdata;



char Mode;
char Mode_1;
char Request_Mode;

int Night;
int Is_Fan_ON;

int ADC_RSLT8; /* ADC Reference Voltage */
int ADC_RSLT9; /* Temperature */
int ADC_RSLT10; /* ACVoltage1p */
int ADC_RSLT11; /* ACVoltage1n */
int ADC_RSLT12; /* ACVoltage1p */
int ADC_RSLT13; /* ACVoltage1n */
int ADC_RSLT14; /* ACVoltage1p */
int ADC_RSLT15; /* ACVoltage1n */

int C_Util_Ph1_P; //not connected in PCB
int C_Util_Ph1_N;
int C_Util_Ph2_P;//not connected in PCB
int C_Util_Ph2_N;
int C_Util_Ph1_Max1;
int C_Util_Ph1_Max2;
int C_Util_Ph2_Max1;
int C_Util_Ph2_Max2;

int C_Gen_Ph1_P;
int C_Gen_Ph1_N;
int C_Gen_Ph2_P;
int C_Gen_Ph2_N;
int C_Gen_Ph1_Max1;
int C_Gen_Ph1_Max2;
int C_Gen_Ph2_Max1;
int C_Gen_Ph2_Max2;


int C_Batt_Ph1_P;
int C_Batt_Ph1_N;
int C_Batt_Ph2_P;
int C_Batt_Ph2_N;
int C_Batt_Ph1_Max1;
int C_Batt_Ph1_Max2;
int C_Batt_Ph2_Max1;
int C_Batt_Ph2_Max2;

int Heatsink_Temp;

int Voltage1_Max1;
int Voltage1_Max2;
int Voltage2_Max1;
int Voltage2_Max2;
int Voltage3_Max1;
int Voltage3_Max2;

int SOC;
int SOC1;
int Batt_SOC;

int Batt_SOC0;
int Batt_SOC1;
int Batt_SOC2;
int Batt_SOC3;
int Batt_SOC4;
int Batt_SOC5;
int Batt_SOC6;
int Batt_SOC7;
int Batt_SOC8;
int Batt_SOC9;


unsigned int count;
unsigned int count_n;
unsigned int count_u;
unsigned int count_g;
unsigned int count_b;
unsigned int count_U;
unsigned int count_G;
unsigned int count_B;
unsigned int count_G_try;

int LCD_Mess_Changed;
char* LCD_Message;
char* LCD_Message2;



// Prototype Declaration
void oscConfig(void);
void clearIntrflags(void);

void init_IO(void);
void init_PWM(void);
void init_ADC(void);
void Init_Timer1(void);
void Init_Timer3(void);
//void Init_ExternalInt(void);


int main(void) {

    /* Configure Oscillator Clock Source 	*/
    //	oscConfig();


    /* Configure Oscillator to operate the device at 40Mhz
               Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
               Fosc= 7.37*(43)/(2*2)=80Mhz for Fosc, Fcy = 40Mhz */

    /* Configure PLL prescaler, PLL postscaler, PLL divisor */
    PLLFBD = 41; /* M = PLLFBD + 2 */
    CLKDIVbits.PLLPOST = 0; /* N1 = 2 */
    CLKDIVbits.PLLPRE = 0; /* N2 = 2 */

    __builtin_write_OSCCONH(0x01); /* New Oscillator FRC w/ PLL */
    __builtin_write_OSCCONL(0x01); /* Enable Switch */

    while (OSCCONbits.COSC != 0b001); /* Wait for new Oscillator to become FRC w/ PLL */
    while (OSCCONbits.LOCK != 1); /* Wait for Pll to Lock */

    /* Now setup the ADC and PWM clock for 7.37MHz			  								//CHANGED
       ((FRC * 16) / APSTSCLR ) = (7.37 * 16) / 16 = ~ 7.37MHz*/ //CHANGED

    ACLKCONbits.FRCSEL = 1; /* FRC provides input for Auxiliary PLL (x16) */
    ACLKCONbits.SELACLK = 1; /* Auxiliary Oscillator provides clock source for PWM & ADC */
    ACLKCONbits.APSTSCLR = 3; /* Divide Auxiliary clock by 16 */ //CHANGED
    ACLKCONbits.ENAPLL = 1; /* Enable Auxiliary PLL */

    while (ACLKCONbits.APLLCK != 1); /* Wait for Auxiliary PLL to Lock */



    /* Clear Interrupt Flags 				*/
    clearIntrflags();

    // UART Configurations
    cfgUart1();
    // PWM Initialize
    init_PWM();
	init_IO();
    // ADC Initialize
    init_ADC();
    Init_Timer1();
    Init_Timer3();
	//Init_ExternalInt();	




    IEC0bits.U1TXIE = 1; //Enable Transmit Interrupt
    IEC0bits.U1RXIE = 1; //Enable Receive Interrupt


    /*	TRISDbits.TRISD9=0;		//defined as output
            TRISDbits.TRISD10=0;	//defined as output
            TRISDbits.TRISD11=0;	//defined as output

            LATDbits.LATD9=0;
            LATDbits.LATD10=0;
            LATDbits.LATD11=0; */

    




    //ADC_RSLT8 = 0;
    ADC_RSLT9 = 0;
    ADC_RSLT10 = 0;
    ADC_RSLT11 = 0;
    ADC_RSLT12 = 0;
    ADC_RSLT13 = 0;
    ADC_RSLT14 = 0;
    ADC_RSLT15 = 0;

    PTCONbits.PTEN = 1; /* Turn ON PWM module */
    ADCONbits.ADON = 1; // Turn on the A/D converter
    //Timer gets ON in timer initialization

	 //External Interrupt gets ON in timer initialization

	

    Voltage1_Max1 = 0;
    Voltage1_Max2 = 0;
    Voltage2_Max1 = 0;
    Voltage2_Max2 = 0;
    Voltage3_Max1 = 0;
    Voltage3_Max2 = 0;



    C_Util_Ph1_Max1 = 0;
    C_Util_Ph1_Max2 = 0;
    C_Util_Ph2_Max1 = 0;
    C_Util_Ph2_Max2 = 0;



	count_G_try=0;


    /* Loop infinitely */
    ii = 100;

	Is_Fan_ON=0;
	Night=0;
    Mode = 'n';
	Mode_1 = 'n';
    Request_Mode = 'n';

    count = 0;
    count_n = 0;
    count_u = 0;
    count_g = 0;
    count_b = 0;
	//SOC=73;

	Batt_SOC0=0;
	Batt_SOC1=0;
	Batt_SOC2=0;
	Batt_SOC3=0;
	Batt_SOC4=0;
	Batt_SOC5=0;
	Batt_SOC6=0;
	Batt_SOC7=0;
	Batt_SOC8=0;
	Batt_SOC9=0;

	Batt_SOC=0;


	LCD_Mess_Changed=1;
	LCD_Message="ITS is running";
	OpenXLCD(0x38);



	//PutsXLCD("Hello");

//printf("ACKi");
    while (1) {

	
	if (LCD_Message!=LCD_Message2)
			{
				if(!BusyXLCD()){WriteCmdXLCD( 0x01); while(BusyXLCD());
								PutsXLCD(LCD_Message);LCD_Message2=LCD_Message;}
				 	
			}
	//if((!Key_Left)&&(!BusyXLCD())&&(LCD_Mess_Changed==1)){WriteCmdXLCD(0x10);LCD_Mess_Changed=0;}
	//if((!Key_Right)&&(!BusyXLCD())&&(LCD_Mess_Changed==1)){WriteCmdXLCD(0x14);LCD_Mess_Changed=0;}
	

	Batt_SOC=(Batt_SOC0+Batt_SOC1+Batt_SOC2+Batt_SOC3+Batt_SOC4+Batt_SOC5+Batt_SOC6+Batt_SOC7+Batt_SOC8+Batt_SOC9);
	Batt_SOC=Batt_SOC-3500;
	SOC1=Batt_SOC/35;
	if(SOC1>100){SOC1=100;}
	if(SOC1<0){SOC1=0;}
	SOC=SOC1;
	//if(Batt_SOC>500){Fan_ON=1;}
	//else{Fan_ON=0;}
	//Is_Fan_ON
LED1=(Key_Down&&Key_Up&&Key_Left);
LED2=Key_Enter;
LED3=Key_Left;
LED4=Key_Down;
	
	
 switch (Is_Fan_ON) {
        case 0:
            if(Heatsink_Temp>150) //48 Centigerad
				{
					Fan_ON=1; 
					Is_Fan_ON=1;
				}
			break;
     	case 1:
            if(Heatsink_Temp<75) //50 Centigerad
				{
					Fan_ON=0; 
					Is_Fan_ON=0;
				}
			break;
    }

}
}

void clearIntrflags(void) {
    /* Clear Interrupt Flags */

    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
    IFS5 = 0;
    IFS6 = 0;
    IFS7 = 0;
}

void oscConfig(void) {

    /*  Configure Oscillator to operate the device at 40Mhz
            Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
            Fosc= 8M*40/(2*2)=80Mhz for 8M input clock */

    PLLFBD = 38; /* M=40 */
    CLKDIVbits.PLLPOST = 0; /* N1=2 */
    CLKDIVbits.PLLPRE = 0; /* N2=2 */
    OSCTUN = 0; /* Tune FRC oscillator, if FRC is used */

    /* Disable Watch Dog Timer */

    RCONbits.SWDTEN = 0;

    /* Clock switch to incorporate PLL*/
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to Primary
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b011); // Wait for Clock switch to occur


    /* Wait for PLL to lock */

    while (OSCCONbits.LOCK != 1) {
    };
}


//void Init_ExternalInt(void){

//	IPC13bits.INT4IP=7;		//Priority
//	INTCON2bits.INT4EP=1;  //Negative edge
//	IEC3bits.INT4IE=1;   // External Interrupt 4 Enable bit

//}

void init_PWM(void) {
    PTPER = 19240;

    PDC1 = 9620; /* PWM1 duty cycle app */



    PHASE1 = 0; /* No phase shift for PWM1 */




    PTCON2bits.PCLKDIV = 0; /* divided by 1 */

    IOCON1bits.PENH = 0; /* PWM1H output controlled by GPIO not PWM */
    IOCON1bits.PENL = 0; /* PWM1L output controlled by GPIO not PWM */
    IOCON1bits.PMOD = 0; /* Select complementary output PWM mode */






};

void init_IO(void) {

	//SW0 to SW6 and Charger_Connect
    TRISEbits.TRISE0 = 0; //defined as output
    TRISEbits.TRISE1 = 0; //defined as output
    TRISEbits.TRISE2 = 0; //defined as output
    TRISEbits.TRISE3 = 0; //defined as output
    TRISEbits.TRISE4 = 0; //defined as output
    TRISEbits.TRISE5 = 0; //defined as output
    TRISEbits.TRISE6 = 0; //defined as output
    TRISEbits.TRISE7 = 0; //defined as output   Charger_Connect

	Charger_Connect=0;
	SW0=0;
	SW1=0;
	SW2=0;
	SW3=0;
	SW4=0;
	SW5=0;
	SW6=0;

//	Generator_Connect Utility_Connect Battery_Connect defined as output
	TRISGbits.TRISG0=0;
	TRISGbits.TRISG1=0;
	TRISFbits.TRISF1=0;

	Gen_Connect=0; 
	Util_Connect=0;
	Batt_Connect=0;

// Dip Switches
	TRISAbits.TRISA0=1;
	TRISBbits.TRISB1=1;
	TRISBbits.TRISB2=1;
	TRISBbits.TRISB3=1;
//LEDS
    TRISDbits.TRISD11 = 0; //defined as output
    TRISCbits.TRISC13 = 0; //defined as output
    TRISCbits.TRISC14 = 0; //defined as output
    TRISDbits.TRISD12 = 0; //defined as output

	LED1=0;
	LED2=0;
	LED3=0;
	LED4=0;
//Keys

	TRISGbits.TRISG13=1;		//Key0 or Key_Enter
	TRISGbits.TRISG12=1;		//Key1 or Key_Return
	TRISGbits.TRISG14=1;		//Key2 or Key_Right
	//TRISCbits.TRISC1=1;		//Key3 or Key_Left   		//These three are analog shared, therefore this is not enough! PCFG bits in ADPCFG or ADPCFG2 should be checked too.
	TRISCbits.TRISC2=1;		//Key4 or Key_Down
	TRISCbits.TRISC3=1;		//Key5 or Key_Up
//Relays

	TRISAbits.TRISA2=0;
	TRISGbits.TRISG2=0;
	TRISGbits.TRISG3=0;
	Relay1=0;
	Relay2=0;
	Relay3=0;
//Fan
	TRISDbits.TRISD13=0;
	Fan_ON=0;
//Dspic_to_CC3
	TRISAbits.TRISA5=0;
	Dspic_to_CC3=0; 
//CC3_to_Dspic
	TRISFbits.TRISF6=1;

//Gen_Run input signal
	TRISDbits.TRISD14=1;



};

void init_ADC(void) {
    ADCONbits.FORM = 0; //INTEGER data format     /*REMIND THAT FOR CONTROLLER OPERATION IT IS BEETER TO CHOOSE FRACTIONAL MODE*/
    ADCONbits.EIE = 0; //Early Interrupt disabled
    ADCONbits.ORDER = 0; //Convert even channel first
    ADCONbits.SEQSAMP = 0; //Select simultaneous sampling
    ADCONbits.ADCS = 6; //ADC clock = FADC/7 = 7.37MHz / 7 = 1.05MHz 		//CHANGED


    ADPCFGbits.PCFG0 = 0; //select CH0 as analog pin  Main Battery SOC
    //ADPCFGbits.PCFG1 = 0; //select CH1 as analog pin	 Inverter Current ph2 n
    //ADPCFGbits.PCFG2 = 0; //select CH2 as analog pin  Inverter Current ph1 p
    //ADPCFGbits.PCFG3 = 0; //select CH3 as analog pin  Inverter Current ph1 n


    //ADPCFGbits.PCFG8 = 0; //select CH8 as analog pin  Voltage Reference
    ADPCFGbits.PCFG9 = 0; //select CH9 as analog pin	  Temperature
    ADPCFGbits.PCFG10 = 0; //select CH10 as analog pin  Voltage1p
    ADPCFGbits.PCFG11 = 0; //select CH11 as analog pin  Voltage1n

    ADPCFGbits.PCFG12 = 0; //select CH12 as analog pin  Voltage2p
    ADPCFGbits.PCFG13 = 0; //select CH13 as analog pin  Voltage2n
    ADPCFGbits.PCFG14 = 0; //select CH14 as analog pin  Voltage3p
    ADPCFGbits.PCFG15 = 0; //select CH15 as analog pin  Voltage3n

    ADPCFG2bits.PCFG16 = 1; //select CH16 as dIGITAL PIN Key3
    ADPCFG2bits.PCFG17 = 1; //select CH17 as dIGITAL PIN Key4
    ADPCFG2bits.PCFG18 = 1; //select CH18 as dIGITAL PIN Key5
    ADPCFG2bits.PCFG19 = 1; //select CH19 as analog pin  Generator Current ph2 n


    ADPCFG2bits.PCFG20 = 0; //select CH20 as analog pin  Utility Current ph1 n
    ADPCFG2bits.PCFG21 = 0; //select CH21 as analog pin  Utility Current ph1 p
    ADPCFGbits.PCFG4 = 0; //select CH4 as analog pin  Utility Current ph2 p
    ADPCFGbits.PCFG5 = 0; //select CH5 as analog pin  Utility Current ph2 n



    IFS6bits.ADCP0IF = 0; //Clear ADC Pair 0 interrupt flag
    IPC27bits.ADCP0IP = 4; //Set ADC Pair 0 interrupt priority
    IEC6bits.ADCP0IE = 1; //Enable the ADC Pair 0 interrupt

    //IFS6bits.ADCP1IF = 0; //Clear ADC Pair 1 interrupt flag
    //IPC27bits.ADCP1IP = 4; //Set ADC Pair 1 interrupt priority
    //IEC6bits.ADCP1IE = 1; //Enable the ADC Pair 1 interrupt

    IFS7bits.ADCP2IF = 0; //Clear ADC Pair 2 interrupt flag
    IPC28bits.ADCP2IP = 4; //Set ADC Pair 2 interrupt priority
    IEC7bits.ADCP2IE = 1; //Enable the ADC Pair 2 interrupt

    //Pair3 is not used

    IFS7bits.ADCP4IF = 0; //Clear ADC Pair 4 interrupt flag
    IPC28bits.ADCP4IP = 4; //Set ADC Pair 4 interrupt priority
    IEC7bits.ADCP4IE = 1; //Enable the ADC Pair 4 interrupt


    IFS7bits.ADCP5IF = 0; //Clear ADC Pair 5 interrupt flag
    IPC28bits.ADCP5IP = 4; //Set ADC Pair 5 interrupt priority
    IEC7bits.ADCP5IE = 1; //Enable the ADC Pair 5 interrupt

    IFS7bits.ADCP6IF = 0; //Clear ADC Pair 6 interrupt flag
    IPC29bits.ADCP6IP = 4; //Set ADC Pair 6 interrupt priority
    IEC7bits.ADCP6IE = 1; //Enable the ADC Pair 6 interrupt

    IFS7bits.ADCP7IF = 0; //Clear ADC Pair 7 interrupt flag
    IPC29bits.ADCP7IP = 4; //Set ADC Pair 7 interrupt priority
    IEC7bits.ADCP7IE = 1; //Enable the ADC Pair 7 interrupt



    //IFS5bits.ADCP8IF = 0; //Clear ADC Pair 8 interrupt flag
    //IPC20bits.ADCP8IP = 4; //Set ADC Pair 8 interrupt priority
    //IEC5bits.ADCP8IE = 1; //Enable the ADC Pair 8 interrupt

    //IFS5bits.ADCP9IF = 0; //Clear ADC Pair 9 interrupt flag
    //IPC20bits.ADCP9IP = 4; //Set ADC Pair 9 interrupt priority
    //IEC5bits.ADCP9IE = 1; //Enable the ADC Pair 9 interrupt

    IFS5bits.ADCP10IF = 0; //Clear ADC Pair 10 interrupt flag
    IPC20bits.ADCP10IP = 4; //Set ADC Pair 10 interrupt priority
    IEC5bits.ADCP10IE = 1; //Enable the ADC Pair 10 interrupt







    ADSTATbits.P0RDY = 0; //Clear Pair 0 data ready bit
    ADCPC0bits.IRQEN0 = 1; //Enable ADC Interrupt pair 0
    ADCPC0bits.TRGSRC0 = 4; //ADC Pair 0 triggered by PWM Generator 1 Trigger

    //ADSTATbits.P1RDY = 0; //Clear Pair 0 data ready bit
    //ADCPC0bits.IRQEN1 = 1; //Enable ADC Interrupt pair 0
    //ADCPC0bits.TRGSRC1 = 4; //ADC Pair 0 triggered by PWM Generator 1 Trigger

    ADSTATbits.P2RDY = 0; //Clear Pair 0 data ready bit
    ADCPC1bits.IRQEN2 = 1; //Enable ADC Interrupt pair 0
    ADCPC1bits.TRGSRC2 = 4; //ADC Pair 0 triggered by PWM Generator 1 Trigger




    ADSTATbits.P4RDY = 0; //Clear Pair 4 data ready bit
    ADCPC2bits.IRQEN4 = 1; //Enable ADC Interrupt pair 1
    //ADCPC2bits.TRGSRC4 = 8; 		//ADC Pair 4 triggered by PWM Generator 5 Trigger
    ADCPC2bits.TRGSRC4 = 4; //ADC Pair 4 triggered by PWM Generator 1 Trigger


    ADSTATbits.P5RDY = 0; //Clear Pair 5 data ready bit
    ADCPC2bits.IRQEN5 = 1; //Enable ADC Interrupt pair 5
    ADCPC2bits.TRGSRC5 = 4; //ADC Pair 5 triggered by PWM Generator 1 Trigger

    ADSTATbits.P6RDY = 0; //Clear Pair 6 data ready bit
    ADCPC3bits.IRQEN6 = 1; //Enable ADC Interrupt pair 6
    ADCPC3bits.TRGSRC6 = 4; //ADC Pair 6 triggered by PWM Generator 1 Trigger

    ADSTATbits.P7RDY = 0; //Clear Pair 7 data ready bit
    ADCPC3bits.IRQEN7 = 1; //Enable ADC Interrupt pair 7
    ADCPC3bits.TRGSRC7 = 4; //ADC Pair 7 triggered by PWM Generator 1 Trigger




    //ADSTATbits.P8RDY = 0; //Clear Pair 8 data ready bit
    //ADCPC4bits.IRQEN8 = 1; //Enable ADC Interrupt pair 8
    //ADCPC4bits.TRGSRC8 = 4; //ADC Pair 8 triggered by PWM Generator 1 Trigger

    //ADSTATbits.P9RDY = 0; //Clear Pair 9 data ready bit
    //ADCPC4bits.IRQEN9 = 1; //Enable ADC Interrupt pair 9
    //ADCPC4bits.TRGSRC9 = 4; //ADC Pair 9 triggered by PWM Generator 1 Trigger

    ADSTATbits.P10RDY = 0; //Clear Pair 10 data ready bit
    ADCPC5bits.IRQEN10 = 1; //Enable ADC Interrupt pair 10
    ADCPC5bits.TRGSRC10 = 4; //ADC Pair 10 triggered by PWM Generator 1 Trigger




    TRGCON1bits.DTM = 1; //dual trigger mode
    TRIG1bits.TRGCMP = 0; //Primary trig compare value
    STRIG1bits.STRGCMP = 0x0FF; //secondary trig compare value

    TRGCON1bits.TRGDIV = 0; // Trigger generated every 1 PWM cycle
    TRGCON1bits.TRGSTRT = 1; // enable Trigger generated after 1 PWM cycles OR Wait 5 PWM cycles before generating the first trigger event after the module is enabled


    //    TRGCON5bits.TRGDIV = 0;		// Trigger generated every 1 PWM cycle
    //	TRGCON5bits.TRGSTRT = 1;		// enable Trigger generated after 1 PWM cycles OR Wait 5 PWM cycles before generating the first trigger event after the module is enabled


};

void Init_Timer1(void) {
    T1CON = 0; // Timer reset
    IFS0bits.T1IF = 0; // Reset Timer1 interrupt flag
    IPC0bits.T1IP = 6; // Timer1 Interrupt priority level=4
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
    TMR1 = 0x0000;
    PR1 = 0xFFFF; // Timer1 period register = ?????
    T1CONbits.TON = 1; // Enable Timer1 and start the counter
}

void Init_Timer3(void) {
    T2CON = 0; // Timer reset
    IFS0bits.T3IF = 0; // Reset Timer1 interrupt flag
    IPC2bits.T3IP = 6; // Timer1 Interrupt priority level=4
    IEC0bits.T3IE = 1; // Enable Timer1 interrupt
    TMR3 = 0x0000;
    PR3 = 0x8FFF; // Timer1 period register = 0x8fff equals 1ms
    T3CONbits.TCKPS = 2; // prescaler = 1/64
    T3CONbits.TON = 1; // Enable Timer1 and start the counter
}

//------------------------------------------------------------------------------
//    T4 interrupt handlers
//------------------------------------------------------------------------------

void __attribute__((__interrupt__,no_auto_psv)) _T4Interrupt(void)
{
	if((!Key_Left)&&(!BusyXLCD())){WriteCmdXLCD(0x10);}
	if((!Key_Right)&&(!BusyXLCD())){WriteCmdXLCD(0x14);}
IFS1bits.T4IF = 0;
}

//------------------------------------------------------------------------------
//    DMA interrupt handlers
//------------------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void) {
    IFS1bits.DMA2IF = 0; // Clear the DMA2 Interrupt Flag;
}


//------------------------------------------------------------------------------
//    UART1 interrupt handlers
//------------------------------------------------------------------------------

void __attribute__((__interrupt__)) _U1TXInterrupt(void) {

    



    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void) {

    tempdata = U1RXREG;
    switch (tempdata) {
        case 'a':
            printf("%d", Voltage1_Max2);
            break;
        case 'b':
            printf("%d", Voltage2_Max2);
            break;
        case 'c':
            printf("%d", Voltage3_Max2);
            break;
        case 'd':
            printf("%d", C_Util_Ph1_Max2);
            break;
        case 'e':
            printf("%d", C_Util_Ph2_Max2);
            break;
        case 'f':
            printf("%d", C_Util_Ph1_Max2);
            break;
        case 'g':
            printf("%d", C_Util_Ph2_Max2);
            break;
        case 'h':
            printf("%d", C_Util_Ph1_Max2);
            break;
        case 'i':
            printf("%d", C_Util_Ph2_Max2);
            break;
        case 'j':
            if (LATEbits.LATE0 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'k':
            if (LATEbits.LATE1 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'l':
            if (LATEbits.LATE2 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'm':
            if (LATEbits.LATE3 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'n':
            if (LATEbits.LATE4 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'o':
            if (LATEbits.LATE5 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'p':
            if (LATEbits.LATE6 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'q':
            if (LATEbits.LATE7 == 1) {
                printf("1");
            } else {
                printf("0");
            }
            break;
        case 'r':
            printf("%c", Mode);
            break;
        case 's':
            LATEbits.LATE0 = 0;
            printf("ACK");
            break;
        case 't':
            LATEbits.LATE1 = 0;
            printf("ACK");
            break;
        case 'u':
            LATEbits.LATE2 = 0;
            printf("ACK");
            break;
        case 'v':
            LATEbits.LATE3 = 0;
            printf("ACK");
            break;
        case 'w':
            LATEbits.LATE4 = 0;
            printf("ACK");
            break;
        case 'x':
            LATEbits.LATE5 = 0;
            printf("ACK");
            break;
        case 'y':
            //LATEbits.LATE6 = 0;
            printf("ACK");
            break;
        case 'z':
            //LATEbits.LATE7 = 0;
            printf("ACK");
            break;
        case 'A':
            LATEbits.LATE0 = 1;
            printf("ACK");
            break;
        case 'B':
            LATEbits.LATE1 = 1;
            printf("ACK");
            break;
        case 'C':
            LATEbits.LATE2 = 1;
            printf("ACK");
            break;
        case 'D':
            LATEbits.LATE3 = 1;
            printf("ACK");
            break;
        case 'E':
            LATEbits.LATE4 = 1;
            printf("ACK");
            break;
        case 'F':
            LATEbits.LATE5 = 1;
            printf("ACK");
            break;
        case 'G':
            LATEbits.LATE6 = 1;
            printf("ACK");
            break;
        case 'H':
            //LATEbits.LATE7 = 1;
            printf("ACK");
            break;
        case 'I':
            Request_Mode = 'b';
			Mode='M';
            printf("ACK");
            break;
        case 'J':
            Request_Mode = 'u';
			Mode='M';
            printf("ACK");
            break;
        case 'K':
            Request_Mode = 'g';
			Mode='M';
            printf("ACK");
            break;
        case 'L':				//This is the automatic mode
            Mode = 'n';
            printf("ACK");
            break;
        case 'M':				
            printf("%d", SOC);
            break;
        case 'N':
            Request_Mode = 'n';
			Mode='M';
            printf("ACK");
            break;
        case 'O':
            Request_Mode = 'n';
			Mode='M';
            printf("ACK");
            break;
        case 'P':
			Night=1;
            printf("ACK");
            break;
        case 'Q':
			Night=0;
            printf("ACK");
            break;
        case 'R':
			//Generator Run=0
            printf("ACK");
            break;
        case 'S':
			//Generator Run=1
            printf("ACK");
            break;
        case 'T':
			//Generator Stop=0
            printf("ACK");
            break;
        case 'U':
			//Generator Stop=1
            printf("ACK");
            break;
        case 'V':
			//Generator Preheat=0
            printf("ACK");
            break;
        case 'W':
			//Generator Preheat=1
            printf("ACK");
            break;
    }

    IFS0bits.U1RXIF = 0; // clear RX interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _ADCP0Interrupt() //
{
    IFS6bits.ADCP0IF = 0;
	Batt_SOC9=Batt_SOC8;
	Batt_SOC8=Batt_SOC7;
	Batt_SOC7=Batt_SOC6;
	Batt_SOC6=Batt_SOC5;
	Batt_SOC5=Batt_SOC4;
	Batt_SOC4=Batt_SOC3;
	Batt_SOC3=Batt_SOC2;
	Batt_SOC2=Batt_SOC1;
	Batt_SOC1=Batt_SOC0;
    Batt_SOC0 = ADCBUF0;
	



    ADSTATbits.P0RDY = 0; // Clear the data is ready in buffer bits

}

/*void __attribute__((__interrupt__, no_auto_psv)) _ADCP1Interrupt() //
{
    IFS6bits.ADCP1IF = 0;

    C_Batt_Ph1_P = ADCBUF2;
    C_Batt_Ph1_N = ADCBUF3; //

    if (C_Batt_Ph1_Max1 < C_Batt_Ph1_N) {
        C_Batt_Ph1_Max1 = C_Batt_Ph1_N;
    }

    ADSTATbits.P1RDY = 0; // Clear the data is ready in buffer bits

}*/

void __attribute__((__interrupt__, no_auto_psv)) _ADCP2Interrupt() //
{
    IFS7bits.ADCP2IF = 0;

    C_Util_Ph2_P = ADCBUF4;
    C_Util_Ph2_N = ADCBUF5; //

    if (C_Util_Ph2_Max1 < C_Util_Ph2_N) {
        C_Util_Ph2_Max1 = C_Util_Ph2_N;
    }

    ADSTATbits.P2RDY = 0; // Clear the data is ready in buffer bits

}

void __attribute__((__interrupt__, no_auto_psv)) _ADCP4Interrupt() //
{
    IFS7bits.ADCP4IF = 0;

    //ADC_RSLT8 = ADCBUF8;
   // ADC_RSLT9 = ADCBUF9; // Read AN21 conversion result
	Heatsink_Temp=ADCBUF9;

    ADSTATbits.P4RDY = 0; // Clear the data is ready in buffer bits

}

void __attribute__((__interrupt__, no_auto_psv)) _ADCP5Interrupt() // Votage 1 reading
{
    IFS7bits.ADCP5IF = 0;

    ADC_RSLT10 = ADCBUF10; //Positive side
    ADC_RSLT11 = ADCBUF11; // Negative side

    if (Voltage1_Max1 < ADC_RSLT10) {
        Voltage1_Max1 = ADC_RSLT10;
    }





    ADSTATbits.P5RDY = 0; // Clear the data is ready in buffer bits

}

void __attribute__((__interrupt__, no_auto_psv)) _ADCP6Interrupt() // Votage 2 reading
{
    IFS7bits.ADCP6IF = 0;

    ADC_RSLT12 = ADCBUF12; //Positive side
    ADC_RSLT13 = ADCBUF13; // Negative side

    if (Voltage2_Max1 < ADC_RSLT12) {
        Voltage2_Max1 = ADC_RSLT12;
    }





    ADSTATbits.P6RDY = 0; // Clear the data is ready in buffer bits

}

void __attribute__((__interrupt__, no_auto_psv)) _ADCP7Interrupt() // Votage 3 reading
{
    IFS7bits.ADCP7IF = 0;

    ADC_RSLT14 = ADCBUF14; //Positive side
    ADC_RSLT15 = ADCBUF15; // Negative side

    if (Voltage3_Max1 < ADC_RSLT14) {
        Voltage3_Max1 = ADC_RSLT14;
    }





    ADSTATbits.P7RDY = 0; // Clear the data is ready in buffer bits

}

/*void __attribute__((__interrupt__, no_auto_psv)) _ADCP8Interrupt() //
{
    IFS5bits.ADCP8IF = 0;

    C_Gen_Ph1_N = ADCBUF16;
    C_Gen_Ph1_P = ADCBUF17; //

    if (C_Gen_Ph1_Max1 < C_Gen_Ph1_N) {
        C_Gen_Ph1_Max1 = C_Gen_Ph1_N;
    }

    ADSTATbits.P8RDY = 0; // Clear the data is ready in buffer bits

}*/

/*void __attribute__((__interrupt__, no_auto_psv)) _ADCP9Interrupt() //
{
    IFS5bits.ADCP9IF = 0;

    C_Gen_Ph2_P = ADCBUF18;
    C_Gen_Ph2_N = ADCBUF19; //

    if (C_Gen_Ph2_Max1 < C_Gen_Ph2_N) {
        C_Gen_Ph2_Max1 = C_Gen_Ph2_N;
    }

    ADSTATbits.P9RDY = 0; // Clear the data is ready in buffer bits

}*/

void __attribute__((__interrupt__, no_auto_psv)) _ADCP10Interrupt() //
{
    IFS5bits.ADCP10IF = 0;

    C_Util_Ph1_N = ADCBUF20;
    C_Util_Ph1_P = ADCBUF21; //not connected in PCB

    if (C_Util_Ph1_Max1 < C_Util_Ph1_N) {
        C_Util_Ph1_Max1 = C_Util_Ph1_N;
    }

    ADSTATbits.P10RDY = 0; // Clear the data is ready in buffer bits

}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt() {
    IFS0bits.T1IF = 0;
    T1CONbits.TON = 0;
    count++;
    if (count == 20) {
        //printf("%d\n",Voltage3_Max2);

        count = 0;
        //LATDbits.LATD4 = ~LATDbits.LATD4;


        Voltage1_Max2 = Voltage1_Max1;
        Voltage1_Max1 = 1;

        Voltage2_Max2 = Voltage2_Max1;
        Voltage2_Max1 = 1;

        Voltage3_Max2 = Voltage3_Max1;
        Voltage3_Max1 = 1;

        C_Util_Ph1_Max2 = C_Util_Ph1_Max1;
        C_Util_Ph1_Max1 = 2;

        C_Util_Ph2_Max2 = C_Util_Ph2_Max1;
        C_Util_Ph2_Max1 = 2;

        //C_Batt_Ph1_Max2 = C_Batt_Ph1_Max1;
        //C_Batt_Ph1_Max1 = 2;

        //C_Batt_Ph2_Max2 = C_Batt_Ph2_Max1;
        //C_Batt_Ph2_Max1 = 2;

        //C_Gen_Ph1_Max2 = C_Gen_Ph1_Max1;
        //C_Gen_Ph1_Max1 = 2;

        //C_Gen_Ph2_Max2 = C_Gen_Ph2_Max1;
        //C_Gen_Ph2_Max1 = 2;

    }

    TMR1 = 0;
    T1CONbits.TON = 1;

    /* reset Timer 1 interrupt flag */
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt() {
    IFS0bits.T3IF = 0;
    T3CONbits.TON = 0;


    //LATDbits.LATD5 = ~LATDbits.LATD5;
    switch (Mode) {
		case 'M':
			
			if(Request_Mode == 'b'){
			LCD_Message="Mannual Battery";
			Batt_Connect=1;
			Util_Connect=0;
			Gen_Connect=0;
			Charger_Connect = 0;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct

			}
			if(Request_Mode == 'u'){
			LCD_Message="Mannual Utility";
			Batt_Connect=0;
			Util_Connect=1;
			Gen_Connect=0;
			Charger_Connect = 1;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct

			}
			if(Request_Mode == 'g'){
			LCD_Message="Mannual Generator";
			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=1;
			Charger_Connect = 1;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=0; //Generator Utility Sense Connect
			Relay3=0; //Generator Utility Sense Connecct

			}
			if(Request_Mode == 'n'){
			LCD_Message="Mannual Disconnect";
			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=0;
			Charger_Connect = 1;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct

			}

		break;
        case 'n':
			LCD_Message="Power Outage";
			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=0;
			Charger_Connect = 1;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct

            count_u = 0;
            count_g = 0;
            count_b = 0;
            count_U = 0;
            count_G = 0;
            count_B = 0;
            if ((Voltage3_Max2 < 250) && (Voltage2_Max2 < 250) && (Voltage1_Max2 > 250)) {
                Mode = 'b';
            }
            if ((Voltage2_Max2 > 250)) {
				Mode_1='n';
                Mode = 'u';
            }
            if ((Voltage2_Max2 < 250) && (Voltage3_Max2 > 250)) {
                Mode = 'g';
            }
            if ((Voltage1_Max2 < 200) && (Voltage2_Max2 < 200) && (Voltage3_Max2 < 200)) {
                Mode = 'n';
			       if((count_G == 0)&&(count_G_try<4)){
									 //Generator_Run=1;	//turn on the generator
									 count_G_try=count_G_try+1;
									}
					count_G=count_G+1;
					if(count_G == 300){
									count_G=0;
									if(Voltage3_Max2 > 250){
									Mode = 'g';
									}
									else{
									//Generator_Run=0;   //turn off the generator
									}
									
					}								
				
            }

            break;

        case 'u':


            if ((Voltage2_Max2 > 250)) {
                Mode = 'u';
                count_u = count_u + 1;
                if (count_u == 100) {
			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=0;
			Charger_Connect = 0;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct

                    Mode = 'U';
                    count_u = 0;
                }
            } else {
                count_u = 0;  
				Mode=Mode_1;
            }
            break;


        case 'U':
			LCD_Message="Utility Connected";
			Batt_Connect=0;
			Util_Connect=1;
			Gen_Connect=0;

			Charger_Connect = 1;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct
			
			SW0 = 1;		//connect all priority controlled loads
		    SW1 = 1;
		    SW2 = 1;
		    SW3 = 1;
		    SW4 = 1;
		    SW5 = 1;
			SW6 = 1;



            if ((Voltage2_Max2 > 250)) {
                Mode = 'U';
            } else {
                Mode = 'n';
            }
            break;



        case 'b':
			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=0;

			Charger_Connect = 0;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct



            if ((Voltage1_Max2 > 250)) {
                Mode = 'b';
                count_b = count_b + 1;   // Transition to battery is fast
                if (count_b == 2) {
                    Mode = 'B';
                    count_b = 0;
                }
            } else {
                count_b = 0;
                Mode = 'n';
            }
            break;

        case 'B':
			
			LCD_Message="Battery Running";

			Batt_Connect=1;
			Util_Connect=0;
			Gen_Connect=0;

			Charger_Connect = 0;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=0; //Generator Utility Sense Connect
			Relay3=0; //Generator Utility Sense Connecct

			SW0 = 1;		// priority controlled loads
		    SW1 = 1;
		    SW2 = 0;
		    SW3 = 0;
		    SW4 = 0;
		    SW5 = 0;
			SW6 = 0;
		    
		    

            if ((Voltage2_Max2 < 250) && (Voltage1_Max2 > 250)) {
                Mode = 'B';
            } else if (Voltage2_Max2 > 250){			//If utility is back!
			    Mode = 'u';
				Mode_1 = 'B';
			}
			 else {
                Mode = 'n';
            }

			if(Night==1){
				if(SOC<35){
					if((count_B == 0)){ 

								Relay2=0; //Generator Utility Sense Connect
								Relay3=0; //Generator Utility Sense Connec
									 
									}
					count_B=count_B+1;
					if(count_B > 400){
									count_B=0;
									if(Voltage3_Max2 > 200){
									Mode = 'g';
										}
									
					}										
				}
			
			}
			else{
			       if((count_B == 0)){ 
								Relay2=0; //Generator Utility Sense Connect
								Relay3=0; //Generator Utility Sense Connec

									}
					count_B=count_B+1;
					if(count_B > 400){
									count_B=0;
									if(Voltage3_Max2 > 200){
									Mode = 'g';
									count_G_try=0;
									     }
									
									}				
			}
            break;


        case 'g':
			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=0;

			Charger_Connect = 0;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=0; //Generator Utility Sense Connect
			Relay3=0; //Generator Utility Sense Connecct

            if ( (Voltage3_Max2 > 200)) {
                Mode = 'g';
                count_g = count_g + 1;
                if (count_g == 2) {
                    Mode = 'G';
                    count_g = 0;
                }
            } else {
                count_g = 0;
                Mode = 'n';
            }
            break;

        case 'G':
			
			LCD_Message="Generator Running";

			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=1;

			Charger_Connect = 1;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=0; //Generator Utility Sense Connect
			Relay3=0; //Generator Utility Sense Connecct
			
			SW0 = 1;		// Manage priority controlled loads
		    SW1 = 1;
		    SW2 = 1;
		    SW3 = 1;
		    SW4 = 1;
		    SW5 = 1;
			SW6 = 1;




            if ((Voltage2_Max2 < 250) && (Voltage3_Max2 > 250)) {
                Mode = 'G';
            } 
			else if (Voltage2_Max2 > 250){
			    Mode = 'u';
				Mode_1 = 'G';
			}
			else {
                Mode = 'n';
            }
			if(Night==1){
					if((SOC>85)){
						Mode='b';
			Batt_Connect=0;
			Util_Connect=0;
			Gen_Connect=0;

			Charger_Connect = 0;		//Disconnect Battery Charger Due to Grounding Issue
			Relay2=1; //Generator Utility Sense Connect
			Relay3=1; //Generator Utility Sense Connecct
					}	
			}
            break;


    }



    TMR3 = 0;
    T3CONbits.TON = 1;

    /* reset Timer 1 interrupt flag */
}
