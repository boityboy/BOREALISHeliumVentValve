#include <msp430.h> 
#include "reg_map.h"
#include "sd_card_raw_library.h"

//=================================================================
// GLOBAL VARIABLES
//=================================================================
int i = 0;
char BMEPacket[] = {0,0,0,0,0,0,0,0};
char setup = 1;
int Data_Cnt = 0;
unsigned char sd_buffer1[512];
unsigned char sd_buffer2[512];

//=================================================================
// FUNCTION DECLARATIONS
//=================================================================
void sensorInit(void);
void GPSInit(void);
void algControl();
void BMESetup(void);
void BMEDataGrab(void);

/**
 * main.c
 */

int main(void)
{
//=================================================================
// INIT
//=================================================================
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	BMEInit();

	SPIInit();

	GPSInit();
//--------------------------------------Port Config---------------------------------------
	P6DIR |= ALG; //  Port 6.0 as testing output
	P6OUT &= ~ALG; //  off at start

	P6DIR &= ~XB; //  Port 5.2 as testing output
	P6REN |= XB;
	P6OUT |= XB;
	P6IES |= XB;
	
	PM5CTL0 &= ~LOCKLPM5; // TURN ON DIGITAL I/O
	
   	__enable_interrupt();

    	P6IFG &= ~XB;
    	P6IE |= XB;

    	UCB0CTLW0 &= ~UCSWRST;  // OUT OF RESET

    	UCB0IE |= UCTXIE0;      // ENABLE I2C TX IRQ
    	UCB0IE |= UCRXIE0;      //ENABLE I2C RX IRQ
	
//=================================================================
// MAIN WHILE LOOP
//=================================================================

    	while(1){
		BMEDataGrab();
    	}
	return 0;
}

//=================================================================
// FUNCTIONS
//=================================================================

//------------------------------ Algorithm Control Function ------------------------------
void algControl(){

    if(altitude > 60000ft && altitude < 80000ft){
        P6OUT |= ALG;
    }else if(altitude > 80000ft || altitude < 60000ft){
        P6OUT &= ~ALG;
    }
}

//------------------------------ GPS Init ------------------------------
void GPSInit(void){

}

//====================================== BME680 Functions ======================================
//------------------------------ I2C Init for BME680 ------------------------------
void BMEInit(void){
    UCB0CTLW0 |= UCSWRST; //RESET

    UCB0CTLW0 |= UCSSEL_3; //SMCLK

    UCB0CTLW0 |= UCMODE_3; //SELECT I2C
    UCB0CTLW0 |= UCMST;   //MASTER MODE
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
    UCB0CTLW0 |= UCTR;    //PUT I2C IN TX MODE
    UCB0TBCNT = 2;        // SENDING 2 BYTEs OF DATA
	
    // Set Oversampling on temperature and pressure
    UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG

    // Set Oversampling on humidity
    UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG

    // Set IIR Filter
    UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG

    // Set to forced mode instead of sleep mode
    UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG
}

//------------------------------ Read Data from BME680 ------------------------------
void BMEDataGrab(){
    UCB0CTLW0 |= UCTR;    //PUT I2C IN TX MODE
    UCB0TBCNT = 1;        // SENDING 1 BYTE OF DATA
    UCB0CTLW0 |= UCTXSTT; //GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG

    UCB0CTLW0 &= ~UCTR;    //PUT I2C IN RX MODE
    UCB0TBCNT = 0x08;      // Grabs 8 bytes of data
    UCB0CTLW0 |= UCTXSTT;  //GENERATE A START CONDITION

    while((UCB0IFG & UCSTPIFG)==0){} //WAITS FOR STOP CONDITION
    UCB0IFG &= ~UCSTPIFG;            //CLEAR STOP FLAG
}

//====================================== SD Card Functions ======================================
//------------------------------ Store data to SD Card ------------------------------
void SDSend(){
    if(address_error != 0){
        address_cnt = address_last;
        address_error = 0;
    }
    //send buffer1 for block 1
    address_last = address_cnt;
    sendData(address_cnt, sd_buffer1);
    address_cnt++;
    __delay_cycles(10000);
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    __delay_cycles(10000);

    //send buffer2 for block 2
    address_last = address_cnt;
    sendData(address_cnt, sd_buffer2);
    address_cnt++;
    __delay_cycles(10000);
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    __delay_cycles(10000);
}

//=================================================================
// INTERRUPT SERVICE ROUTINES
//=================================================================

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    switch(UCB0IV){
    case 0x16://RX
        BMEPacket[Data_Cnt] = UCB0RXBUF; //Retrieve Data
        if ((Data_Cnt)==(sizeof(BMEPacket))-1){
            Data_Cnt = 0;
        }else{
            Data_Cnt++;
        }
        break;

    case 0x18://TX
        if(setup){
            UCB0TXBUF=BMECMD[i];
            i++;
        }else{
            UCB0TXBUF=0x1F;
        }
        break;
    }
}
