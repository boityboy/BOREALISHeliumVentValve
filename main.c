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
unsigned int *FRAM_ptr = (unsigned long *)0x18FC;;
char algcheck = 1;


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
    P6DIR |= ALG; //  Alg Control Output Port
    P6OUT &= ~ALG; 

    P6DIR &= ~XB; //  XB Control Input Port
    P6REN |= XB;
    P6OUT |= XB;
    P6IES |= XB;
    
    P4DIR &= ~BIT7; //  Reset Input Port
    P4REN |= BIT7;
    P4OUT &= ~BIT7;
    P4IES &= ~BIT7;

    PM5CTL0 &= ~LOCKLPM5; // TURN ON DIGITAL I/O

    // Port Interrupt Setup
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

//=================================================================
// MAIN WHILE LOOP
//=================================================================

        while(1){
            if(Data_Cnt < 500){
               BMEDataGrab();
            }else{
                SDSend(dataIn);
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
    UCB0I2CSA = 0x0076;   //SLAVE ADDRESS - BME680

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
// Adds Delimiter to SD packet then activates a reading on the BME
// Finally reads 3 bytes of pressure, 3 bytes of temperature, and 2 bytes of humidity
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
// Checks the Address value in the FRAM and sends data to that location in the sd card
// The address count increments and writes the new address to the FRAM
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

//====================================== Non-Volatile Memory Functions ======================================
//------------------------------ Store data to FRAM ------------------------------
// Keep track of address on SD Card regardless of power cycling
void FRAMWrite (unsigned long data)
{
    SYSCFG0 = FRWPPW | PFWP;
    *FRAM_ptr = data;
    SYSCFG0 = FRWPPW | PFWP | DFWP;
}

//------------------------------ Read data to FRAM ------------------------------
unsigned long FRAMRead(void){
    unsigned long dat;
    dat = *FRAM_ptr; // data to read
    return dat;
}

//=================================================================
// INTERRUPT SERVICE ROUTINES
//=================================================================

// BME I2C Interrupt
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
        // Sends Setup and Parameter Commands
        if(setup == 1){
            UCB0TXBUF=BMECMD[BMESendCnt];
            BMESendCnt++;
        // Sets BME to be ready for reading
        }else if(ModeChg == 1){
            UCB0TXBUF = BMEmode[j];
            j++;
        }
        // First Register for BME Data Reading
        else{
            UCB0TXBUF=0x1F;
        }
        break;
    }
}

// Manual XB Override interrupt
#pragma vector = PORT6_VECTOR
__interrupt void ISR_Port6_S2(void)
{
    algcheck = 0;
    P6IFG &= ~XB;
}

// Reset for FRAM Address value
#pragma vector = PORT4_VECTOR
__interrupt void ISR_Port4_S2(void)
{
    FRAMWrite(0x0);
    P4IFG &= ~BIT7;
}
