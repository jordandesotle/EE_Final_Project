/***********************************************************************************
 * Final Lab Code for EE321
 *
 * The purpose of this program is to control a system utilizing two buttons, 3 LEDs,
 * a motor, and a potentiometer. This code is meant to prepare us for the lab final
 * exam where professor Ming Li could ask us to do anything. By setting up the board
 * and function ahead of time, this should help to ensure that the lab goes as
 * smoothly as possible.
 *
 * The following code relies on the fact that you have the same physical setup as
 * described in the code. The pin definitions tell you what pin to connect each of
 * your components to.
 *
 * Authors: Jordan De Sotle
***********************************************************************************/
#include <msp430.h> 
#include <stdbool.h>


/**********************************************************************************/

// Motor
#define PWMA_PIN BIT7       // Pin 1.7
#define AIN2_PIN BIT6       // Pin 1.6
#define AIN1_PIN BIT5       // Pin 1.5

// LEDs
#define RED_LED_PIN BIT1    // Pin 1.1
#define YELLOW_LED_PIN BIT3 // Pin 1.3
#define GREEN_LED_PIN BIT4  // Pin 1.4

#define BLUE_LED1_PIN BIT2  // Pin 3.2
#define BLUE_LED2_PIN BIT3  // Pin 3.3

// Potentiometer
#define POT_PIN BIT2        // Pin 1.2

// Button
#define BUTTON1_PIN BIT0    // Pin 2.0
#define BUTTON2_PIN BIT1    // Pin 2.1

/**********************************************************************************/

/**
 * Initializes the Pin inputs and outputs
 */
void initialize() {

    // LED setup
    P1DIR |= RED_LED_PIN | YELLOW_LED_PIN | GREEN_LED_PIN;      // LED pins as output
    P3DIR |= BLUE_LED1_PIN | BLUE_LED2_PIN;                     // Blue LEDs as output

    P1OUT &= ~RED_LED_PIN;                                      // Turn off Red LED
    P1OUT &= ~YELLOW_LED_PIN;                                   // Turn off Yellow LED
    P1OUT &= ~GREEN_LED_PIN;                                    // Turn off Green LED
    P3OUT &= ~BLUE_LED1_PIN;                                    // Turn off Blue LED 1
    P3OUT &= ~BLUE_LED2_PIN;                                    // Turn off Blue LED 2

    // Button 1 setup
    P2DIR &= ~BUTTON1_PIN;                                      // Button 1 pin as input
    P1REN |= BUTTON1_PIN;
    P1OUT |= BUTTON1_PIN;
    P1IES |= BUTTON1_PIN;

    P1IFG &= ~BUTTON1_PIN;
    P1IE  |=  BUTTON1_PIN;


    // Button 2 setup
    P2DIR &= ~BUTTON2_PIN;                                      // Button 2 pin as input
    P2REN |= BUTTON2_PIN;
    P2OUT |= BUTTON2_PIN;
    P2IES |= BUTTON2_PIN;

    P2IFG &= ~BUTTON2_PIN;
    P2IE  |=  BUTTON2_PIN;


    // Motor Setup
    P1DIR |= PWMA_PIN;      // PWMA
    P1DIR |= AIN1_PIN;      // AIN1
    P1DIR |= AIN2_PIN;      // AIN2

    P1OUT &= ~PWMA_PIN;
    P1OUT &= ~AIN1_PIN;
    P1OUT &= ~AIN2_PIN;

    PM5CTL0 &= ~LOCKLPM5;
}


/**
 * Turns on the red LED.
 */
void turnOnRedLED() {
    P1OUT |= RED_LED_PIN;
}

/**
 * Turns off the red LED.
 */
void turnOffRedLED() {
    P1OUT &= ~RED_LED_PIN;
}

/**
 * Turns on the yellow LED.
 */
void turnOnYellowLED() {
    P1OUT |= YELLOW_LED_PIN;
}

/**
 * Turns off the yellow LED.
 */
void turnOffYellowLED() {
    P1OUT &= ~YELLOW_LED_PIN;
}

/**
 * Turns on the green LED.
 */
void turnOnGreenLED() {
    P1OUT |= GREEN_LED_PIN;
}

/**
 * Turns off the green LED.
 */
void turnOffGreenLED() {
    P1OUT &= ~GREEN_LED_PIN;
}


///**
// * This function takes a time in milliseconds and starts
// * a background timer for the specified duration
// */
//void setTimer(int timeMS) {
//
//    // Set up timer
//    TB0CTL |= TBCLR;
//    TB0CTL |= TBSSEL__ACLK;
//    TB0CTL |= MC__UP;
//    TB0CCR0 = 32768;                // Represents 1 second
//
//    timerExpired = false;
//    timeUnderTwoSeconds = false;
//    timeSeconds = 0;
//
//    /* Checks to see what type of timing to use
//     * - For times over 2000ms, a loop is needed to increment every second until the desired time
//     * - For times under 2000ms, the value of the time in ms is divided by 1000 and multiplied against the frequency of the clock
//     */
//    if(timeMS <= 2000) {
//        timeUnderTwoSeconds = true;
//        TB0CCR0 = 32768 * timeMS / 1000;
//    } else {
//        timeSeconds = (int)(timeMS / 1000);
//    }
//
//    TB0CCTL0 |= CCIE;
//    TB0CCTL0 &= ~CCIFG;
//
//    __enable_interrupt();       // Enable global interrupts
//}

/**
 * Turns off the motor
 */
void turnOffMotor() {
    P1OUT &= ~PWMA_PIN;
    P1OUT &= ~AIN1_PIN;
    P1OUT &= ~AIN2_PIN;
}

/**
 * Turns the motor on clockwise (door open) for 350ms
 */
void turnMotorOnClockwise() {

    P1OUT &= ~AIN1_PIN;
    P1OUT |= AIN2_PIN;

    P1OUT |= PWMA_PIN;

}

/**
 * Turns the motor on counter clockwise (door closed) for 350ms
 */
void turnMotorOnCounterClockwise() {

    P1OUT |= AIN1_PIN;
    P1OUT &= ~AIN2_PIN;

    P1OUT |= PWMA_PIN;
}



/**
 * Main function
 */
int main(void) {

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    initialize();               // Initialization function

    __enable_interrupt();       // Enable global interrupts

    while(true) {
        turnMotorOnCounterClockwise();
        __delay_cycles(1000000);

        turnOffMotor();
        __delay_cycles(1000000);

        turnMotorOnClockwise();
        __delay_cycles(1000000);

        turnOffMotor();
        __delay_cycles(1000000);

        turnOnRedLED();
        __delay_cycles(1000000);

        turnOffRedLED();
        __delay_cycles(1000000);

        turnOnYellowLED();
        __delay_cycles(1000000);

        turnOffYellowLED();
        __delay_cycles(1000000);

        turnOnGreenLED();
        __delay_cycles(1000000);

        turnOffGreenLED();
        __delay_cycles(1000000);

    }






    return 0;
}


#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void) {

        if(P2IFG & BIT0) {
            P3OUT ^= BLUE_LED1_PIN;
            P2IFG &= ~BIT0;
        }

        if(P2IFG & BIT1) {
            P3OUT ^= BLUE_LED2_PIN;
            P2IFG &= ~BIT1;
        }

}

//#pragma vector=TIMER0_B0_VECTOR
//__interrupt void Timer_B0_CCR0(void)
//{
//
//    if(timeUnderTwoSeconds) {
//        // code to trigger timer is up
//        timerExpired = true;
//    } else {
//        if(timerCount < timeSeconds) {
//           timerCount++;
//        } else {
//           // code to trigger timer is up
//           timerExpired = true;
//           timerCount = 0;
//        }
//    }
//
//    TB0CCTL0 &= ~CCIFG;
//}


//#pragma vector=PORT1_VECTOR
//__interrupt void P1_ISR(void) {
//
//    if(!dontRegisterButton) {
//        if(P1IFG & BIT1) {
//            programRunning = !programRunning;
//            P1IFG &= ~BIT1;
//        }
//    }
//
//}

