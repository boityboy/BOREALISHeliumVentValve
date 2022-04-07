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
int p1;
int p2;
int p3;
int p4;
int p5;
int p6;
int p7;
int p8;
int p9;
int p10;
int t1;
int t2;
int t3;

//=================================================================
// FUNCTION DECLARATIONS
//=================================================================
void BMEInit(void);
void GPSInit(void);
void algControl();
void BMESetup(void);
void BMEDataGrab(void);
void SDSend(unsigned char* dataIn);
int pressCalc(void);

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
    P6DIR |= ALG; //  Port 6.0 as testing output
    P6OUT &= ~ALG; //  off at start

    P6DIR &= ~XB; //  Port 5.2 as testing output
    P6REN |= XB;
    P6OUT |= XB;
    P6IES |= XB;

    PM5CTL0 &= ~LOCKLPM5; // TURN ON DIGITAL I/O

    P6IFG &= ~XB;
    P6IE |= XB;

    UCB0CTLW0 &= ~UCSWRST;  // OUT OF RESET

    UCB0IE |= UCTXIE0;      // ENABLE I2C TX IRQ
    UCB0IE |= UCRXIE0;      //ENABLE I2C RX IRQ

    // for SD Card
    sdCardInit();                                       // Send Initialization Commands to SD Card
    unsigned char dataIn[6];
    sendCommand(0x50, 0x200, 0xFF, dataIn);             // CMD 16 : Set Block Length
    char pargrab = 1;
    float pressReal;
    __enable_interrupt();
    // Sensor Setup Message
    BMESetup();

//=================================================================
// MAIN WHILE LOOP
//=================================================================

        while(1){
            if(Data_Cnt < 500){
               BMEDataGrab();
               if(pargrab == 1){
                   p1 = sd_buffer[5] << 8 | sd_buffer[6];
                   p2 = sd_buffer[7] << 8 | sd_buffer[8];
                   p3 = sd_buffer[9];
                   p4 = sd_buffer[10] << 8 | sd_buffer[11];
                   p5 = sd_buffer[12] << 8 | sd_buffer[13];
                   p6 = sd_buffer[14];
                   p7 = sd_buffer[15];
                   p8 = sd_buffer[16] << 8 | sd_buffer[17];
                   p9 = sd_buffer[18] << 8 | sd_buffer[19];
                   p10 = sd_buffer[20];
                   t1 = sd_buffer[0] << 8 | sd_buffer[1];
                   t2 = sd_buffer[2] << 8 | sd_buffer[3];
                   t3 = sd_buffer[4];
                   pargrab = 0;
               }
               pressReal = pressCalc();
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
int pressCalc(void){
    int tvar1, tvar2, tvar3, t_fine, pvar1, pvar2, pvar3, press_comp, temp_comp;
    int tempadc = (sd_buffer[Data_Cnt - 5] << 12) | (sd_buffer[Data_Cnt - 4] << 4) | ((sd_buffer[Data_Cnt - 3] | 0b11110000) >> 4);
    int pressadc = (sd_buffer[Data_Cnt - 8] << 12) | (sd_buffer[Data_Cnt - 7] << 4) | ((sd_buffer[Data_Cnt - 6] | 0b11110000) >> 4);
    tvar1 = (tempadc >> 3) - (t1 << 1);
    tvar2 = (tvar1 * t2) >> 11;
    tvar3 = ((((tvar1 >> 1) * (tvar1 >> 1)) >> 12) * (t3 << 4)) >> 14;
    t_fine = tvar3+tvar2;
    temp_comp = ((t_fine * 5) + 128) >> 8;;
    pvar1 = (t_fine >> 1) - 64000;
    pvar2 = ((((pvar1 >> 2) * (pvar1 >> 2)) >> 11) * p6) >> 2;
    pvar2 = pvar2 + ((pvar1 * p5) << 1);
    pvar2 = (pvar2 >> 2) + (p4 << 16);
    pvar1 = (((((pvar1 >> 2) * (pvar1 >> 2)) >> 13) * (p3 << 5)) >> 3) + ((p2 * pvar1) >> 1);
    pvar1 = pvar1 >> 18;
    pvar1 = ((32768 + pvar1) * p1) >> 15;
    press_comp = 1048576 - pressadc;
    press_comp = ((press_comp - (pvar2 >> 12)) * 3125);
    if (press_comp >= (1 << 30)){
        press_comp = ((press_comp / pvar1) << 1);
    }else{
        press_comp = ((press_comp << 1) / pvar1);
    }
    pvar1 = (p9 * (((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
    pvar2 = ((press_comp >> 2) * p8) >> 13;
    pvar3 = ((press_comp >> 8) * (press_comp >> 8) * (press_comp >> 8) * p10) >> 17;
    press_comp = press_comp + ((pvar1 + pvar2 + pvar3 + (p7 << 7)) >> 4);
    press_comp = press_comp * 1;
    return press_comp;
}

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
    while(BMESendCnt < 9){
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
    while(BMESendCnt < 38){
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
        address_cnt = address_last;
        address_error = 0;
    }
    //send buffer1 for block 1
    address_last = address_cnt;
    sendData(address_cnt, sd_buffer);
    address_cnt++;
    __delay_cycles(10000);
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    __delay_cycles(10000);
    Data_Cnt = 0;

}

//=================================================================
// INTERRUPT SERVICE ROUTINES
//=================================================================

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
