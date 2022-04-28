#include <msp430.h> 
#include "reg_map.h"
#include "sd_card_raw_library.h"
#include "math.h"

//=================================================================
// GLOBAL VARIABLES
//=================================================================
int BMESendCnt = 0;
char setup = 1;
int Data_Cnt = 0;
int ModeChg = 0;
int i;
char j = 0;
unsigned char sd_buffer[512];
unsigned char BMEmode[] = {0x74, 0x49};
unsigned long address_cnt  = 0; // CNT for address of microSD
unsigned long address_last = 0; // For error flag
int secondCount = 0;
int minuteCount = 0;
int hourCount = 0;
unsigned int *FRAM_ptr = (unsigned long *)0x18FC;
char algcheck = 1;


//Timer Length = (1/(SMCLK/divider)) * counter
void timer0Init(int divider, int counter);


//=================================================================
// FUNCTION DECLARATIONS
//=================================================================
void BMEInit(void);
void GPSInit(void);
void algControl();
void BMESetup(void);
void BMEDataGrab(void);
void SDSend(unsigned char* dataIn);
void FRAMWrite(unsigned long);
unsigned long FRAMRead();

/**
 * main.c
 */

int main(void)
{
//=================================================================
// INIT
//=================================================================
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    BMEInit();

    SPIInit();

//    GPSInit();
//--------------------------------------Port Config---------------------------------------
    P6DIR |= ALG; //  Port 6.0 as Alg control output
    P6OUT &= ~ALG; //  off at start

    P6DIR &= ~XB;
    P6REN |= XB;
    P6OUT &= ~XB;
    P6IES &= ~XB;

    P4DIR &= ~BIT7;
    P4REN |= BIT7;
    P4OUT &= ~BIT7;
    P4IES &= ~BIT7;

    PM5CTL0 &= ~LOCKLPM5; // TURN ON DIGITAL I/O

    P6IFG &= ~XB;
    P6IE |= XB;
    P4IFG &= ~BIT7;
    P4IE |= BIT7;

    UCB0CTLW0 &= ~UCSWRST;  // OUT OF RESET

    UCB0IE |= UCTXIE0;      // ENABLE I2C TX IRQ
    UCB0IE |= UCRXIE0;      //ENABLE I2C RX IRQ

    // for SD Card
    sdCardInit();                                       // Send Initialization Commands to SD Card
    unsigned char dataIn[6];
    sendCommand(0x50, 0x200, 0xFF, dataIn);             // CMD 16 : Set Block Length

    __enable_interrupt();
    // Sensor Setup Message
    BMESetup();
    timer0Init(48, 0x5161);

//=================================================================
// MAIN WHILE LOOP
//=================================================================

        while(1){
            if((secondCount % 10) == 0){
                if(Data_Cnt < 492){
                    sd_buffer[Data_Cnt] = 'T';
                    Data_Cnt++;
                    sd_buffer[Data_Cnt] = 'I';
                    Data_Cnt++;
                    sd_buffer[Data_Cnt] = 'M';
                    Data_Cnt++;
                    sd_buffer[Data_Cnt] = 'E';
                    Data_Cnt++;
                    sd_buffer[Data_Cnt] = ':';
                    Data_Cnt++;
                    sd_buffer[Data_Cnt] = hourCount;
                    Data_Cnt++;
                    sd_buffer[Data_Cnt] = minuteCount;
                    Data_Cnt++;
                    sd_buffer[Data_Cnt] = secondCount;
                    Data_Cnt++;
                    BMEDataGrab();
                }else{
                    SDSend(dataIn);
                }
            }
        }
    return 0;
}

//=================================================================
// FUNCTIONS
//=================================================================

//------------------------------ Algorithm Control Function ------------------------------

/*void algControl(){

    if(altitude > 60000ft && altitude < 80000ft){
        P6OUT |= ALG;
    }else if(altitude > 80000ft || altitude < 60000ft){
        P6OUT &= ~ALG;
    }
}
*/

//------------------------------ GPS Init ------------------------------
void GPSInit(void){

}

//====================================== BME680 Functions ======================================
//------------------------------ I2C Init for BME680 ------------------------------
void BMEInit(void){
    P1REN |= SCL | SDA;
    P1OUT |= SCL | SDA;

    UCB0CTLW0 |= UCSWRST; //RESET

    UCB0CTLW0 |= UCSSEL_3; //SMCLK

    UCB0CTLW0 |= UCMODE_3; //SELECT I2C
    UCB0CTLW0 |= UCMST;   //MASTER MODE
    UCB0CTLW0 |= UCSYNC;

    UCB0CTLW0 &= ~UCTR;   //RECEIVE MODE
    UCB0I2CSA = 0x0077;   //SLAVE ADDRESS - BME680

    UCB0CTLW1 |= UCASTP_2; //AUTO STOP MODE

    P1SEL1 &= ~SCL;    //P1.3 = SCL
    P1SEL0 |= SCL;

    P1SEL1 &= ~SDA;     //P1.2 = SDA
    P1SEL0 |= SDA;
}
//------------------------------ Setup BME680 ------------------------------
void BMESetup(){
    // Setup Commands to Send
    UCB0CTLW0 |= UCTR;    //PUT +I2C IN TX MODE
    UCB0TBCNT = 0x02;        // SENDING 2 BYTEs OF DATA
    while(BMESendCnt < 7){
        UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION
        while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
        UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG
    }
    // Apply Delimiter
    sd_buffer[Data_Cnt] = 'P';
    Data_Cnt++;
    sd_buffer[Data_Cnt] = 'A';
    Data_Cnt++;
    sd_buffer[Data_Cnt] = 'R';
    Data_Cnt++;
    sd_buffer[Data_Cnt] = ':';
    Data_Cnt++;
    // Acquire Calibration Parameters
    while(BMESendCnt < 37){
        UCB0CTLW0 |= UCTR;    //PUT I2C IN TX MODE
        UCB0TBCNT = 0x01;        // SENDING 1 BYTE OF DATA
        UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION

        while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION

        UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG

        UCB0CTLW0 &= ~UCTR;    //PUT I2C IN RX MODE
        UCB0TBCNT = 0x01;      // Length of Receiving data
        UCB0CTLW0 |= UCTXSTT;  //GENERATE A START CONDITION

        while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION

        UCB0IFG &= ~UCSTPIFG;   //CLEAR STOP FLAG
    }
    setup = 0;
}

//------------------------------ Read Data from BME680 ------------------------------
void BMEDataGrab(){
    // Apply Delimiter
    sd_buffer[Data_Cnt] = 'B';
    Data_Cnt++;
    sd_buffer[Data_Cnt] = 'M';
    Data_Cnt++;
    sd_buffer[Data_Cnt] = 'E';
    Data_Cnt++;
    sd_buffer[Data_Cnt] = ':';
    Data_Cnt++;

    // Set Mode to forced mode
    ModeChg = 1;
    UCB0CTLW0 |= UCTR;    // TX mode
    UCB0TBCNT = 2;        // SENDING 2 BYTEs OF DATA
    UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION

    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG

    ModeChg = 0;
    j = 0;

    // Retrieve Data
    UCB0CTLW0 |= UCTR;    //PUT I2C IN TX MODE
    UCB0TBCNT = 1;        // SENDING 1 BYTE OF DATA
    UCB0CTLW0 |= UCTXSTT;//GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG

    UCB0CTLW0 &= ~UCTR;    //PUT I2C IN RX MODE
    UCB0TBCNT = 8;      // Grabs 8 bytes of data
    UCB0CTLW0 |= UCTXSTT;  //GENERATE A START CONDITION
    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG
}

//====================================== SD Card Functions ======================================
//------------------------------ Store data to SD Card ------------------------------
void SDSend(unsigned char* dataIn){
    if(address_error != 0){
        address_cnt = FRAMRead();
        address_error = 0;
    }
    //send buffer1 for block 1
    address_cnt = FRAMRead();
    sendData(address_cnt, sd_buffer);
    address_cnt++;
    FRAMWrite(address_cnt);
    __delay_cycles(10000);
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    __delay_cycles(10000);
    Data_Cnt = 0;

}

void FRAMWrite (unsigned long data)
{
    SYSCFG0 = FRWPPW | PFWP;
    *FRAM_ptr = data;
    SYSCFG0 = FRWPPW | PFWP | DFWP;
}

unsigned long FRAMRead(void){
    unsigned long dat;
    dat = *FRAM_ptr; // data to read
    return dat;
}

//------------------------------ Configure Timer0 ------------------------------
void timer0Init(int divider, int counter){
// Timer Length = (1/(SMCLK/TB0CTL_IDXX)) * TBRCCRR0
        TB0CTL |= TBCLR;            //Clear timers and dividers
        TB0CTL |= TBSSEL__SMCLK;    //Source = SMclk
        TB0CTL |= CNTL_0;           //Use 16bit counter
        TB0CTL |= MC__UP;           //Count up to value in TB0CCR0

        switch(divider){
        case 0x01:
        TB0CTL |= ID__1;            //Divide by 1
        break;

        case 0x02:
        TB0CTL |= ID__2;            //Divide by 2
        break;

        case 0x04:
        TB0CTL |= ID__4;            //Divide by 4
        break;

        case 0x08:
        TB0CTL |= ID__8;            //Divide by 8
        break;

        case 0x10:
        TB0CTL |= ID__4;            //Division by 4
        TB0EX0 |= TBIDEX__4;        //Division by 4
        // 4*4 = 16 (0x10)          //Divide by 16
        break;

        case 0x14:
        TB0CTL |= ID__4;            //Division by 4
        TB0EX0 |= TBIDEX__5;        //Division by 5
        // 4*5 = 20 (0x14)          //Divide by 20
        break;

        case 0x18:
        TB0CTL |= ID__4;            //Division by 4
        TB0EX0 |= TBIDEX__6;        //Division by 6
        // 4*6 = 24 (0x18)          //Divide by 24
        break;

        case 0x20:
        TB0CTL |= ID__4;            //Division by 4
        TB0EX0 |= TBIDEX__8;        //Division by 8
        // 4*6 = 24 (0x20)          //Divide by 32
        break;

        case 0x30:
        TB0CTL |= ID__8;            //Division by 8
        TB0EX0 |= TBIDEX__6;        //Division by 6
        // 8*6 = 32 (0x30)          //Divide by 48
        break;

        }//End Switch

        TB0CCR0 = counter;
        // Timer Length = (1/(SMCLK/divider)) * counter

        TB0CCTL0 |= CCIE;           //Enable timer interrupt
        TB0CCTL0 &= ~CCIFG;         //Clear interrupt flag

    return;
}

//=================================================================
// INTERRUPT SERVICE ROUTINES
//=================================================================

//Timer interrupt 0 ----------------------------------
#pragma vector = TIMER0_B0_VECTOR
__interrupt void OneSecondTimer(void)
{
    secondCount++;

    if(secondCount == 60){
        minuteCount++;
        secondCount = 0;
        if((minuteCount % 30) == 0){
            P6OUT ^= ALG;
            sd_buffer[Data_Cnt] = 'V';
            Data_Cnt++;
            sd_buffer[Data_Cnt] = 'A';
            Data_Cnt++;
            sd_buffer[Data_Cnt] = 'L';
            Data_Cnt++;
            sd_buffer[Data_Cnt] = 'V';
            Data_Cnt++;
            sd_buffer[Data_Cnt] = 'E';
            Data_Cnt++;
        }
    }
    if(minuteCount == 60){
        hourCount++;
        minuteCount = 0;
    }

    TB0CCTL0 &= ~CCIFG;         //clear flag
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    switch(UCB0IV){
    case 0x16://RX
        sd_buffer[Data_Cnt] = UCB0RXBUF; //Retrieve Data
        if ((Data_Cnt)==(sizeof(sd_buffer))-1){
            Data_Cnt = 0;
        }else{
            Data_Cnt++;
        }
        break;

    case 0x18://TX
        if(setup == 1){
            UCB0TXBUF=BMECMD[BMESendCnt];
            BMESendCnt++;
        }else if(ModeChg == 1){
            UCB0TXBUF = BMEmode[j];
            j++;
        }
        else{
            UCB0TXBUF=0x1F;
        }
        break;
    }
}
/*
#pragma vector = PORT6_VECTOR
__interrupt void ISR_Port6_S2(void)
{
    algcheck = 0;
    P6IFG &= ~XB;
}
*/
/*
#pragma vector = PORT4_VECTOR
__interrupt void ISR_Port4_S2(void)
{
    FRAMWrite(0x0);
    P4IFG &= ~BIT7;
}
*/
