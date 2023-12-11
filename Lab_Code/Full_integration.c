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

unsigned int ADC_Value;

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
#define BLUE_LED2_PIN BIT7  // Pin 3.7

#define POT_LED_PIN BIT7    // Pin 4.7

// Potentiometer
#define POT_PIN BIT2        // Pin 1.2

// Button
#define BUTTON1_PIN BIT0    // Pin 2.0
#define BUTTON2_PIN BIT1    // Pin 2.1

/**********************************************************************************/


init_motor_control() {

    TB0CCR0 = 100;             // Set Timer B0 period
    TB0CCR1 = 10;              // Set Timer B0 CCR1 value for PWM duty cycle

}

init_led_control() {

    TB0CCR0 = 4096;
    TB0CCR1 = 2000;

    ADCIE |= ADCIE0;
}

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
    P2REN |= BUTTON1_PIN;
    P2OUT |= BUTTON1_PIN;
    P2IES |= BUTTON1_PIN;
    P2IFG &= ~BUTTON1_PIN;
    P2IE  |=  BUTTON1_PIN;

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

    // Potentiometer Setup
    P1SEL1 |= POT_PIN;     // Configure P1.2 for PWM
    P1SEL0 |= POT_PIN;

/**************************************************/


    // Initialize ADC and PWM
    ADCCTL0 &= ~ADCSHT;
    ADCCTL0 |= ADCSHT_2;
    ADCCTL0 |= ADCON;

    ADCCTL1 |= ADCSSEL_2 | ADCSHP;
    ADCCTL2 &= ~ADCRES;
    ADCCTL2 |= ADCRES_2;

    P1SEL1 |= BIT2;                 // Configure P1.2 for ADC (analog-to-digital converter)
    P1SEL0 |= BIT2;
    ADCMCTL0 |= ADCINCH_2;          // Set the selected channel to A2 (P1.2)

    // Configure Timer B0 for PWM
    TB0CTL |= TBCLR;           // Clear Timer B0
    TB0CTL |= TBSSEL__SMCLK;   // Set Timer B0 clock source to SMCLK
    TB0CTL |= MC__UP;          // Set Timer B0 to Up mode
    TB0CCTL0 |= CCIE;          // Enable Timer B0 CCR0 interrupt
    TB0CCTL1 |= CCIE;          // Enable Timer B0 CCR1 interrupt
    TB0CCTL0 &= ~CCIFG;        // Clear Timer B0 CCR0 interrupt flag
    TB0CCTL1 &= ~CCIFG;        // Clear Timer B0 CCR1 interrupt flag


   // init_motor_control();
    init_led_control();



/**************************************************/


    // Enable interrupts
    __enable_interrupt();

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

void controlSpeedWithPotentiometer() {
    // Start ADC conversion
    ADCCTL0 |= ADCENC + ADCSC;

    // Wait for ADC conversion to complete
    while ((ADCIFG & ADCIFG) == 0);

    // Read ADC value and calculate speed percentage
    volatile float ADC_Value = ADCMEM0;
    volatile float speedPercentage = (ADC_Value * 100) / 4095;

    // Adjust the speed control logic as needed
    TB0CCR1 = (int)speedPercentage;  // Update PWM duty cycle based on potentiometer value
}


/**
 * Main function
 */
int main(void) {

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    initialize();               // Initialization function

    __enable_interrupt();       // Enable global interrupts

    // Enable on of these to set the motor direction
    //turnMotorOnClockwise();
    //turnMotorOnCounterClockwise();

    while(true) {

        // This is code for the LED control
        ADCCTL0 |= ADCENC + ADCSC;
        __bis_SR_register(CPUOFF + GIE); // Enter LPM0

        // This controls the motor
       // controlSpeedWithPotentiometer();



//        turnMotorOnCounterClockwise();
//        __delay_cycles(1000000);
//
//        turnOffMotor();
//        __delay_cycles(1000000);
//
//        turnMotorOnClockwise();
//        __delay_cycles(1000000);
//
//        turnOffMotor();
//        __delay_cycles(1000000);
//
//        turnOnRedLED();
//        __delay_cycles(1000000);
//
//        turnOffRedLED();
//        __delay_cycles(1000000);
//
//        turnOnYellowLED();
//        __delay_cycles(1000000);
//
//        turnOffYellowLED();
//        __delay_cycles(1000000);
//
//        turnOnGreenLED();
//        __delay_cycles(1000000);
//
//        turnOffGreenLED();
//        __delay_cycles(1000000);

    }



    return 0;
}

// ADC ISR
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void) {
    // Enable code in here for LED control

    ADC_Value = ADCMEM0;
    TB0CCR1 = ADC_Value;
    __bic_SR_register_on_exit(CPUOFF); // Exit LPM0
}

// Timer B0 CCR0 ISR
#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void) {

   // P1OUT |= PWMA_PIN;                                  // Set motor control pin
    P4OUT |= POT_LED_PIN;
    TB0CCTL0 &= ~CCIFG;                                 // Clear Timer B0 CCR0 interrupt flag
}

// Timer B0 CCR1 ISR
#pragma vector=TIMER0_B1_VECTOR
__interrupt void ISR_TB1_CCR1(void) {

//    P1OUT &= ~PWMA_PIN;                                 // Clear motor control pin
    P4OUT &= ~POT_LED_PIN;
    TB0CCTL1 &= ~CCIFG;                                 // Clear Timer B0 CCR1 interrupt flag
}


// Button interrupts
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void) {

        if(P2IFG & BIT0) {
            P3OUT ^= BLUE_LED1_PIN;
        }

        if(P2IFG & BIT1) {
            P3OUT ^= BLUE_LED2_PIN;
        }

        P2IFG &= ~BIT0;
        P2IFG &= ~BIT1;
}
