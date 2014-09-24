/*********************************************************************
*
*                            Includes
*
*********************************************************************/
#include <p18cxxx.h>

/*********************************************************************
*
*                       Config Bit Settings
*
*********************************************************************/
#pragma config RETEN = OFF          // VREG Sleep Enable bit
#pragma config INTOSCSEL = LOW      // LF-INTOSC Low-power Enable bit
#pragma config SOSCSEL = LOW        // SOSC Power Selection
#pragma config XINST = OFF          // Extended Instruction Set
#pragma config FOSC = INTIO2        // Internal osc with I/O on RA6 and RA7
#pragma config PLLCFG = OFF


/*********************************************************************
*
*                             Defines
*
*********************************************************************/

#define CAN_RECV_ID	0x32	//even!!!
#define CAN_SEND_ID     CAN_RECV_ID //0x6B

#define YAW_AXIS

#define TRUE    1
#define FALSE   0
#define INVALID (0xffff)

#define max(A,B) ((A)>(B)?(A):(B))
#define min(A,B) ((A)<(B)?(A):(B))
#define abs(A) ((A)<0?(-(A)):(A))

#define DEVICE_OSC  64
#define ONE_MS      (unsigned int)(DEVICE_OSC/4)*80

#define M0dir_in1 PORTBbits.RB4 // Motor Dir-control output M0
#define M0dir_in2 PORTDbits.RD5 // Motor Dir-control output M0
#define M1dir_in1 PORTDbits.RD1 // Motor Dir-control output M1
#define M1dir_in2 PORTDbits.RD6 // Motor Dir-control output M1
#define PWM_val_m0 CCPR2L
#define PWM_val_m1 CCPR3L
#define TMR2_ON T2CONbits.TMR2ON //On/Off timer-2
#define TMR0_ON T0CONbits.TMR0ON //On/Off timer-0
#define SWITCHdir_in PORTDbits.RD4 // switch

// ECAN bitrate define, only can choose one rate
//  100 Kbps @ 64MHz  0x93
//  125 Kbps @ 64MHz  0x8F
//  500 Kbps @ 64MHz  0x83
//  1   Mbps @ 64MHz  0x81
#define F_ECAN_SPEED    0x93                 // 1 set ECAN module on 100Kbps
#define Trange		  10				// Defines the aceptable range of target achievement
#define SERVO_CYCLE 	0x8000

// Servo Motor related
#define TMR1_ON T1CONbits.TMR1ON //On/Off timer-0
#define ENABLE_SERVO PORTDbits.RD0


/*********************************************************************
*
*                        Function Prototypes
*
*********************************************************************/
void InitDevice(void);
void Delay(unsigned int count);

void sys_config(void);
void motor_stop(void);
unsigned int ADCRead(unsigned char);
unsigned int getPosition(unsigned char);

void InitECAN(void);

unsigned char ECAN_Receive(unsigned char *);

void ECAN_Transmit(unsigned char, unsigned char, unsigned char *);

void ISRpidFunc (void);
int updatePidAN(int,unsigned int, unsigned int);
void motor_drive(unsigned char, int);
unsigned char controlMotor(unsigned char, unsigned int, unsigned int);
unsigned int gray_bin(unsigned int);
void spi_recv16(unsigned char *val);

/*********************************************************************
*
*                            Global Variables
*
*********************************************************************/
unsigned char g_flag = 0; //bit 0 is pid timer flag; bit 1 is dir flag for m0 and bit 2 is dir flag for m1

unsigned char g_SIDH = 0x00;
unsigned char g_SIDL = 0x00;
unsigned char g_DLC = 0x00;

//PID parameters for motor 0
const float Kp_m0 = 0.85;
const float Kd_m0 = 0.095;
const float Ki_m0 = 0.00000802;

//PID parameters for motor 1
const float Kp_m1 = 3.95;
const float Kd_m1 = 0.195;
const float Ki_m1 = 0.0000802;

//internal states of PID
static int last_error_m0 = 0;
long Istate_m0 = 0;
static int last_error_m1 = 0;
long Istate_m1 = 0;

//servo
unsigned int g_servo_target = SERVO_CYCLE;

//---------------------------------------------------------------------------
#pragma code ISRpidVectSec = 0x0018
void
ISRpidVect (void)
{
  _asm
    goto ISRpidFunc //jump to interrupt routine
  _endasm
}

//----------------------------------------------------------------------------
#pragma code
#pragma interrupt ISRpidFunc

void ISRpidFunc (void){
	unsigned int tmp;
    if(PIR1bits.TMR1IF == 1){
        TMR1_ON = 0;
		if(ENABLE_SERVO) {
			tmp = 0xffff-(SERVO_CYCLE-g_servo_target);
	        TMR1H = (tmp>>8);                 // FOSC/4 = 16MHz; Prescaller = 8 :: 2MHz; 0.0005ms period;
	        TMR1L = (tmp&0xff);
		}
		else {
			tmp = 0xffff-g_servo_target;
	        TMR1H = (tmp>>8);                 // FOSC/4 = 16MHz; Prescaller = 8 :: 2MHz; 0.0005ms period;
	        TMR1L = (tmp&0xff);
		}
        ENABLE_SERVO ^= (g_servo_target!=INVALID)?1:0;
        PIR1bits.TMR1IF = 0;
        TMR1_ON = 1;
    }
    if(INTCONbits.TMR0IF == 1){
        g_flag |= 1;
        INTCONbits.TMR0IF = 0;
    }
}

//----------------------------------------------------------------------------

void set16(unsigned char *buffer, unsigned int val) {
    *(buffer++) = val&0xff;
    *buffer = (val>>8);
}


/*********************************************************************
*
*                            Main Function
*
*********************************************************************/
void main (){
    /******************* Init System ****************/
    unsigned int CurrPos_m0 = 0;
    unsigned int CurrPos_m1 = 0;
    unsigned int targetValue_m0 = INVALID, targetValue_m1 = INVALID;
    int cnt=0, send = 0;
    unsigned char buffer[8];

    InitDevice();
    sys_config();

    motor_stop();

	TMR1_ON = 1; //start servo
    while(1){
        TMR0_ON = 1;

#ifndef YAW_AXIS
        CurrPos_m0 = getPosition(0);
#else
        spi_recv16((unsigned char*)&CurrPos_m0);
		CurrPos_m0 = gray_bin(CurrPos_m0);
#endif
        CurrPos_m1 = getPosition(1);

        if(g_flag & 1) {	//update pwm
            TMR0_ON = 0; //lock
            g_flag &= ~(1);
            if(targetValue_m0!=INVALID) send|=controlMotor(0, CurrPos_m0, targetValue_m0);
            if(targetValue_m1!=INVALID) send|=controlMotor(1, CurrPos_m1, targetValue_m1);
            TMR0H = 0xFF;                 //PWM will be updated after every 16 PWM pulses // FC
            TMR0L = 0x00;
            TMR0_ON = 1;
        }

        if(ECAN_Receive(buffer) == TRUE){
            TMR1_ON = 0;
            motor_stop();
            ENABLE_SERVO = 0;

            targetValue_m0 = (((unsigned int)buffer[2])<<8) | (unsigned int)buffer[1];
            targetValue_m1 = (((unsigned int)buffer[4])<<8) | (unsigned int)buffer[3];
            g_servo_target = min(SERVO_CYCLE, (((unsigned int)buffer[6])<<8) | (unsigned int)buffer[5]);

            PIR1bits.TMR1IF = 0;
            TMR1_ON = 1;

            buffer[0]=0x80;
            ECAN_Transmit(g_SIDH, g_SIDL, buffer);

            last_error_m0 = 0;
            Istate_m0 = 0;
            last_error_m1 = 0;
            Istate_m1 = 0;
        }

        //send pos.
        cnt++;
        if(cnt>9 || send) {
            //set bit of switch here
            buffer[0] = (send&0x01) | (SWITCHdir_in?0x02:0x00);
            set16(buffer+1, CurrPos_m0);
            set16(buffer+3, CurrPos_m1);
            //set16(buffer+5, gray_bin(CurrPos_m2));	//absolute encoder (by SPI)
            ECAN_Transmit(CAN_SEND_ID, 0xC0, buffer);
            cnt=0;
            send=0;
        }

        Delay(ONE_MS);
    }
}


unsigned char controlMotor(unsigned char ch, unsigned int pos, unsigned int target) {
    if(max(target, pos)-min(target, pos) < Trange) {
        motor_drive(ch, 0);
        return 1;
    }

    motor_drive(ch, updatePidAN(ch, pos, target));
    return 0;
}

/*********************************************************************
*
*                       Initialize the Device
*
*********************************************************************/
void InitDevice(void)
{
    // Set the internal oscillator to 64MHz
    OSCCONbits.IRCF = 7;
    OSCTUNEbits.PLLEN = 1;

    // Initialize CAN module
    InitECAN();
}

/*********************************************************************
*
*                 Perform a simple delay
*
*********************************************************************/
void Delay(unsigned int count)
{
    // Countdown until equal to zero and then return
    while(count--);
}

void spi_recv8(unsigned char *val) {
    SSPBUF = 0;	//load 0 to spi
	while(!PIR1bits.SSPIF);
	*val = SSPBUF;
    PIR1bits.SSPIF = 0; //reset flag
}

void spi_recv16(unsigned char *val)
{
    spi_recv8(val+1);
	spi_recv8(val);
}

/*********************************************************************
*
*                 Sys config for Sensors and PID motor control
*
*********************************************************************/
void sys_config(){ // System Configuration
    ADCON2=0b10001101;			//Right-Justified, 2Tad, Fosc/16 clock
    //ADCON1=0x00;			//
    ANCON0 = 0x03;				  //Make portA digital except RA0/AN0 RA1/AN1
    ANCON1 = 0x00;				  //Make portB digital
    TRISA=0x03;

    INTCON = 0xE0;                //Disable global and enable TMR0 interrupt  //changed by me prev was 0x20
    INTCON2 = 0x20;               //TMR0 high priority //changed by me prev was 0x84
    INTCON3 = 0x00;
    RCONbits.IPEN = 1;            //enable priority levels //changed by me

//necessary for motor encoder pid	TMR0H = 0xFF;                 //PWM will be updated after every 16 PWM pulses // FC
//necessary for motor encoder pid	TMR0L = 0x00;                    //Four PWM pulses are produced while TMR0L goes from 0 to FF
    TMR0H = 0xFA;                 //PWM will be updated after every 16 PWM pulses // FC
    TMR0L = 0x00;                    //Four PWM pulses are produced while TMR0L goes from 0 to FF
    T0CON = 0x00;

    TRISB=0x08;  //changed by me
    PORTB=0x00;  //changed by me prev
    TRISC=0x00; //set PORTC as OUTPUT
    PORTC=0x18;

    TRISD = 0x10;  //new for M1 in 1 trial
    PORTD=0x00;  //new for M1 in 1 trial

    //Timer 2 Period Register
    PR2=0xFF ;
    //Timer 2 Control Register:: bit 2 (Timer 2 on bit::1=ON:0=OFF); bit 1-0(Prescaler 00=1:01=4:10=16)
    T2CON = 0b00000000 ;
    CCPTMRS = 0b00000000; //0= CCPx is based off of TMR1/TMR2
    //CCP Module 2 Configuration register:: bit 3-0 (11xx PWM mode)
    //CCP Module 2 Configuration register:: bit 5-4 (bit 1 and bit 0 of 10bit PWM duty cycle)
    CCP2CON = 0b00001100 ;
    CCP3CON = 0b00001100 ;

    PWM_val_m0 = 0x00;
    PWM_val_m1 = 0x00;

    // servo motor related
    PIE1bits.TMR1IE = 1;
    IPR1bits.TMR1IP = 0;
    TMR1H = 0x73;                 // FOSC/4 = 16MHz; Prescaller = 8 :: 2MHz; 0.0005ms period;
    TMR1L = 0x5F;                    //Need 18ms interval interrupt; 0.0005*36,000 = 18ms; TMR from 735F to FFFF is 36,000
    T1CON = 0x3A; 	// Prescaller is 8

    /* Initial the PIC18 SPI Peripheral */
    SSPCON1bits.SSPEN = 0;					// disable SPI port for reconfiguration
    SSPSTAT = 0x80;				// set SSPSTAT (sampling at end,  Transmit occurs on transition from Idle (1) to active (0) clock state)
    SSPCON1  = 0x12;				// set SSPCON (high is idle, Fosc/64)
    SSPCON1bits.SSPEN = 1;					// re-enable SPI port
}

/*********************************************************************
*
*                 motor control functions
*
*********************************************************************/
void motor_stop(){
    PWM_val_m1 = 0;
    PWM_val_m0 = 0;
    Delay(ONE_MS);
    TMR2_ON = 0;
    PORTCbits.RC6 = 0;
    PORTCbits.RC2 = 0;
    M1dir_in1 = 0;
    M1dir_in2 = 0;
    M0dir_in1 = 0;
    M0dir_in2 = 0;
}

/*********************************************************************
*
*                 Read Analog sensor function
*
*********************************************************************/
unsigned int ADCRead(unsigned char ch)
{
   if(ch>13) return 0;  //Invalid Channel

   ADCON0=0x00;

   ADCON0=(ch<<2);   //Select ADC Channel

   ADCON0bits.ADON=1;  //switch on the adc module

   ADCON0bits.GO=1;  //Start conversion

   while(ADCON0bits.GO); //wait for the conversion to finish

   ADCON0bits.ADON=0;  //switch off adc

   return ADRES;
}



/*********************************************************************
*
*                       Configure the CAN Module
*
*********************************************************************/
void InitECAN(void)
{
    // Enter CAN module into config mode
    CANCON = 0x80;    //REQOP<2:0>=100
    while(!(CANSTATbits.OPMODE ==0x04));

    // Enter CAN module into Mode 0
    ECANCON = 0x00;

    BRGCON1 = F_ECAN_SPEED; //0001 1111     //SJW=3TQ     BRP  19
    BRGCON2 = 0xB8; //1011 1000     //SEG2PHTS 1    sampled once  PS1=8TQ  PropagationT 1TQ
    BRGCON3 = 0x05; //0000 0101     //PS2  6TQ

    // Initialize Receive Masks
    //  The first mask is used that accepts all SIDs and no EIDs
    RXM0EIDH = 0x00;    //
    RXM0EIDL = 0x00;
    RXM0SIDH = 0xFF;    // Standard ID FILTER
    RXM0SIDL = 0xE0;

    //  The second mask is used to ignore all SIDs and EIDs
    RXM1EIDH = 0x00;    // 0's for EID and SID
    RXM1EIDL = 0x00;
    RXM1SIDH = 0xFF;
    RXM1SIDL = 0xE0;

    // Enable Filters
    //  Only using two filters
    RXFCON0 = 0x03;     //Disable all
    RXFCON1 = 0x00;     //Disable all

    // Initialize Receive Filters
    //  Filter 0 = 0x196
    //  Filter 1 = 0x19E

    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;
    RXF0SIDH = CAN_RECV_ID;
    RXF0SIDL = 0xC0;

    RXF1EIDH = 0x00;
    RXF1EIDL = 0x00;
    RXF1SIDH = CAN_RECV_ID+1;
    RXF1SIDL = 0xC0;


    // Enter CAN module into normal mode
    CANCON = 0x00;
    while(CANSTATbits.OPMODE==0x00);

    // Set Receive Mode for buffers
    RXB0CON = 0x00;
    RXB1CON = 0x00;

}

/*********************************************************************
*
*                Check the buffers, if it have message
*
*********************************************************************/
unsigned char ECAN_Receive(unsigned char *buffer)
{
    unsigned char RXMsgFlag;

    RXMsgFlag = 0x00;

    if (RXB0CONbits.RXFUL) //CheckRXB0
    {
        g_SIDH = RXB0SIDH;
        g_SIDL = RXB0SIDL;
        g_DLC = RXB0DLC;
        *(buffer++) = RXB0D0;
        *(buffer++) = RXB0D1;
        *(buffer++) = RXB0D2;
        *(buffer++) = RXB0D3;
        *(buffer++) = RXB0D4;
        *(buffer++) = RXB0D5;
        *(buffer++) = RXB0D6;
        *(buffer++) = RXB0D7;
        RXB0CONbits.RXFUL = 0;
        RXMsgFlag = 0x01;
    }
    else if (RXB1CONbits.RXFUL) //CheckRXB1
    {
        g_SIDH = RXB1SIDH;
        g_SIDL = RXB1SIDL;
        g_DLC = RXB1DLC;
        *(buffer++) = RXB1D0;
        *(buffer++) = RXB1D1;
        *(buffer++) = RXB1D2;
        *(buffer++) = RXB1D3;
        *(buffer++) = RXB1D4;
        *(buffer++) = RXB1D5;
        *(buffer++) = RXB1D6;
        *(buffer++) = RXB1D7;
        RXB1CONbits.RXFUL = 0;
        RXMsgFlag = 0x01;
    }
    else if (B0CONbits.RXFUL) //CheckB0
    {
        g_SIDH = B0SIDH;
        g_SIDL = B0SIDL;
        g_DLC = B0DLC;
        *(buffer++) = B0D0;
        *(buffer++) = B0D1;
        *(buffer++) = B0D2;
        *(buffer++) = B0D3;
        *(buffer++) = B0D4;
        *(buffer++) = B0D5;
        *(buffer++) = B0D6;
        *(buffer++) = B0D7;

        B0CONbits.RXFUL = 0;
        RXMsgFlag = 0x01;
    }

    if  (RXMsgFlag == 0x01)
    {
        RXMsgFlag = 0x00;
        PIR5bits.RXB1IF = 0; //A CAN Receive Buffer has received a new message
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}



/*********************************************************************
*
*                      Transmit Sample Mesaage
*
*********************************************************************/
void ECAN_Transmit(unsigned char SIDH, unsigned char SIDL, unsigned char *buffer)
{
    TXB0EIDH = 0x00;
    TXB0EIDL = 0x00;

    TXB0SIDH = SIDH;
    TXB0SIDL = SIDL;

    TXB0DLC = 0x08;
    TXB0D0 = *(buffer++);
    TXB0D1 = *(buffer++);
    TXB0D2 = *(buffer++);
    TXB0D3 = *(buffer++);
    TXB0D4 = *(buffer++);
    TXB0D5 = *(buffer++);
    TXB0D6 = *(buffer++);
    TXB0D7 = *(buffer++);

    TXB0CONbits.TXREQ = 1; //Set the buffer to transmit
}

void motor_drive(unsigned char ch, int pwm){
    if(ch==0) {
        M0dir_in1 = (pwm > 0?1:0);
        M0dir_in2 = (pwm < 0?1:0);
        PWM_val_m0 = (unsigned char)(abs(pwm));
    } else if(ch==1) {
        M1dir_in1 = (pwm > 0?1:0);
        M1dir_in2 = (pwm < 0?1:0);
        PWM_val_m1 = (unsigned char)(abs(pwm));
    }
    TMR2_ON = 1;
}

int updatePidAN(int motor_sel, unsigned int pos, unsigned int target){             // compute PWM value
    float pidTerm = 0;                                                            // PID correction
    int error=0;
    int retVal = 0;

    error = target - pos;
    if(motor_sel == 0){
        Istate_m0 = Istate_m0 + error;
        pidTerm = (Kp_m0 * error) + (Kd_m0 * (error - last_error_m0)) + Istate_m0*Ki_m0;
        last_error_m0 = error;
    }
    else{
        Istate_m1 = Istate_m1 + error;
        pidTerm = (Kp_m1 * error) + (Kd_m1 * (error - last_error_m1)) + Istate_m1*Ki_m1;
        last_error_m1 = error;
    }

    //retVal = PWM + (int)pidTerm;
    retVal = (int)(pidTerm);

    if((retVal <= -256))
        retVal = -255;
    else if(retVal >= 256)
        retVal = 255;

    return retVal;
}

unsigned int getPosition(unsigned char ch) {
    unsigned int val = 0, j;
    for(j=0; j<50; j++)
        val += ADCRead(ch);
    return val / 50;
}

/*********************************************************************
*
*                 Perform a Gray to Binary conversion
*
*********************************************************************/
unsigned int gray_bin(unsigned int gray_val)
{
	int i, shft;
	unsigned int tmp1, tmp2, gray_tmp, bin_tmp;
	unsigned int one = 1;
	unsigned int one_tmp;
	gray_tmp = gray_val;
	bin_tmp = gray_val;
	for(i=1; i<16; i++)
	{	
		tmp1 = gray_tmp & 0x08000;
		tmp2 = gray_tmp & 0x04000;
		if(tmp1 == 0x8000){
			shft = (15 - i);
			if(tmp2 == 0x04000){
			//	bin_tmp &= ~(one << shft);
				one_tmp = ~(one << shft);
				bin_tmp = bin_tmp & one_tmp;
			}
			else{
			//	bin_tmp |= (one << shft);
				one_tmp = (one << shft);
				bin_tmp = bin_tmp | one_tmp;
			}
			gray_tmp = (bin_tmp << i);
			//gray_tmp = (gray_tmp << 1);
		}
		else
			gray_tmp = (gray_tmp << 1);
	}

	return bin_tmp;
}