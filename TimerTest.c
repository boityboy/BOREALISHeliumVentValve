#include <msp430.h> 


/**
 * main.c
 */

int secondCount = 0;
int minuteCount = 0;
int hourCount = 0;


//Timer Length = (1/(SMCLK/divider)) * counter
void timer0Init(int divider, int counter);

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    timer0Init(48, 0x5161);

	//Enable Global interrupt
	__enable_interrupt();

	while(1){
	    int i = 0;
	    for(i = 0; i < 100; i++){}
	}

	return 0;
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

//Timer interrupt 0 ----------------------------------
#pragma vector = TIMER0_B0_VECTOR
__interrupt void OneSecondTimer(void)
{
    secondCount++;

    if(secondCount == 60){
        minuteCount++;
        secondCount = 0;
    }
    if(minuteCount == 60){
        hourCount++;
        minuteCount = 0;
    }

    TB0CCTL0 &= ~CCIFG;         //clear flag
}
