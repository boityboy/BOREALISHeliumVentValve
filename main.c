#include <msp430.h> 
#include "reg_map.h"
#include "sd_card_raw_library.h"

void sensorInit(void);
void GPSInit(void);
void algControl();




/**
 * main.c
 */


int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	sensorInit();

	SPIInit();

	GPSInit();

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


    while(1){

    }
	return 0;
}

// I2C BME680 Init
void sensorInit(void){
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

void GPSInit(void){

}

void algControl(){

    if(altitude > 60000ft){
        P6OUT |= ALG;
    }else if(altitude > 80000ft){
        P6OUT &= ~ALG;
    }
}

