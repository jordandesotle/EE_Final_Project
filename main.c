/***********************************************************************************
 * Final Project Code for EE321
 *
 * The purpose of this program is to control a system utilizing a PIR sensor, LEDs, a buzzer,
 * and a motor to automate the opening and closing of a door. The system is activated and
 * deactivated using a button. When activated, the PIR sensor detects motion, triggering the
 * system to close the door. After a cooldown period, during which the system is inactive,
 * the door automatically opens. The system provides visual and audible feedback during
 * various states, such as armed, motion detected, cooldown, and system off.
 *
 * Authors: Jordan De Sotle, Ash Amarnath
***********************************************************************************/
#include <msp430.h> 
#include <stdbool.h>


// Variables
int timerCount = 0;                         // This is a counter that is used to count up each second until a max (timeSeconds)
int timeSeconds = 0;                        // This gets set in the setTimer() function and represents the time delay in seconds

bool timerExpired = false;                  // This bool is true once the timer expires
bool timeUnderTwoSeconds = false;           // This is used to determine how the time is kept track of
bool dontRegisterButton = false;            // Used to make sure the button press doesnt register during activation of the system (door could get stuck open or closed)

bool pirSensorState = false;                // Set in detectMotion() function. Represents if motion has been detected or not
bool programRunning = false;                // Can be toggled using button
bool motionDetected = false;                // Used to escape first conditional in main program logic to prevent looping
bool cooldownActive = false;                // Used to escape second conditional in main program logic to prevent looping


/**********************************************************************************/

// Motor
#define PWMA_PIN BIT2       // Pin 1.2
#define AIN1_PIN BIT3       // Pin 1.3
#define AIN2_PIN BIT4       // Pin 1.4

// LEDs
#define RED_LED_PIN BIT5    // Pin 4.5
#define ARMED_LED BIT5      // Pin 1.5
#define YELLOW_LED_PIN BIT6 // Pin 1.6
#define GREEN_LED_PIN BIT7  // Pin 1.7

// Button
#define BUTTON_PIN BIT1     // Pin 1.1

// PIR Sensor
#define PIR_SENSOR_PIN BIT4 // Pin 6.4

// Buzzer
#define BUZZER_PIN BIT2     // Pin 2.2

/**********************************************************************************/

/**
 * Initializes the Pin inputs and outputs
 */
void initialize() {

    // Configure PIR sensor, LEDs, and Buzzer
    P6DIR &= ~PIR_SENSOR_PIN;
    P1DIR |= YELLOW_LED_PIN | GREEN_LED_PIN | ARMED_LED;
    P4DIR |= RED_LED_PIN;
    P2DIR |= BUZZER_PIN;

    P2OUT &= ~BUZZER_PIN;
    P4OUT &= ~RED_LED_PIN;
    P1OUT &= ~YELLOW_LED_PIN;
    P1OUT &= ~GREEN_LED_PIN;
    P1OUT &= ~ARMED_LED;

    // Motor Setup
    P1DIR |= PWMA_PIN;      // PWMA
    P1DIR |= AIN1_PIN;      // AIN1
    P1DIR |= AIN2_PIN;      // AIN2

    P1OUT &= ~PWMA_PIN;
    P1OUT &= ~AIN1_PIN;
    P1OUT &= ~AIN2_PIN;

    // Button
    P1DIR &= ~BIT1;
    P1REN |= BIT1;
    P1OUT |= BIT1;
    P1IES |= BIT1;

    P1IFG &= ~BIT1;
    P1IE  |=  BIT1;

    PM5CTL0 &= ~LOCKLPM5;
}

/**
 * Detects motion using the PIR sensor.
 * Returns true if motion is detected, false otherwise.
 */
bool detectMotion() {
    pirSensorState = P6IN & PIR_SENSOR_PIN;
    return pirSensorState;
}

/**
 * Turns on the buzzer.
 */
void turnOnBuzzer() {
    P2OUT |= BUZZER_PIN;
}

/**
 * Turns off the buzzer.
 */
void turnOffBuzzer() {
    P2OUT &= ~BUZZER_PIN;
}

/**
 * Turns on the red LED.
 */
void turnOnRedLED() {
    P4OUT |= RED_LED_PIN;
}

/**
 * Turns off the red LED.
 */
void turnOffRedLED() {
    P4OUT &= ~RED_LED_PIN;
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
 * Turns on the armed LED.
 */
void turnOnArmedLED() {
    P1OUT |= ARMED_LED;
}

/**
 * Turns off the armed LED.
 */
void turnOffArmedLED() {
    P1OUT &= ~ARMED_LED;
}

/**
 * This function takes a time in milliseconds and starts
 * a background timer for the specified duration
 */
void setTimer(int timeMS) {

    // Set up timer
    TB0CTL |= TBCLR;
    TB0CTL |= TBSSEL__ACLK;
    TB0CTL |= MC__UP;
    TB0CCR0 = 32768;                // Represents 1 second

    timerExpired = false;
    timeUnderTwoSeconds = false;
    timeSeconds = 0;

    /* Checks to see what type of timing to use
     * - For times over 2000ms, a loop is needed to increment every second until the desired time
     * - For times under 2000ms, the value of the time in ms is divided by 1000 and multiplied against the frequency of the clock
     */
    if(timeMS <= 2000) {
        timeUnderTwoSeconds = true;
        TB0CCR0 = 32768 * timeMS / 1000;
    } else {
        timeSeconds = (int)(timeMS / 1000);
    }

    TB0CCTL0 |= CCIE;
    TB0CCTL0 &= ~CCIFG;

    __enable_interrupt();       // Enable global interrupts
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

    __delay_cycles(350000);

    turnOffMotor();

}

/**
 * Turns the motor on counter clockwise (door closed) for 350ms
 */
void turnMotorOnCounterClockwise() {

    P1OUT |= AIN1_PIN;
    P1OUT &= ~AIN2_PIN;

    P1OUT |= PWMA_PIN;

    __delay_cycles(350000);

    turnOffMotor();
}

/**
 * Beeps for 500ms and then turns off for 500ms
 */
void beep() {

    turnOnBuzzer();
    __delay_cycles(500000);  // Delay for 0.5s

    turnOffBuzzer();
    __delay_cycles(500000);

}


/**
 * Main function
 */
int main(void) {

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    initialize();               // Initialization function

    __enable_interrupt();       // Enable global interrupts

    while(true) {
        if (programRunning) {
            // System is toggled on
            turnOffGreenLED();
            turnOnArmedLED();

            if (detectMotion() && !motionDetected && !cooldownActive) {
                // Motion has been detected
                motionDetected = true;
                dontRegisterButton = true;

                // Turn on red LED
                turnOnRedLED();

                // Beep while motionDetected value from sensor is high
                while(detectMotion()) {
                    beep();
                }

            } else if (!detectMotion() && motionDetected && !cooldownActive) {
                // Motion was just detected and the value from the sensor has gone from HIGH to LOW
                dontRegisterButton = true;
                motionDetected = false;
                turnOffRedLED();
                turnOffBuzzer();
                turnOnYellowLED();

                // trigger cooldown
                cooldownActive = true;
                setTimer(8000);  // 8 seconds

                // start motor
                turnMotorOnCounterClockwise();          // Close the door

            }

            if(!motionDetected && cooldownActive) {
                dontRegisterButton = true;
                // Cooldown has started for sensor

                // Listener for timer expiration
                if(timerExpired) {

                    turnMotorOnClockwise();             // Open the door

                    turnOffYellowLED();

                    // Signal that cooldown has ended
                    cooldownActive = false;
                    turnOnBuzzer();
                    __delay_cycles(100000);
                    turnOffBuzzer();

                    dontRegisterButton = false;
                    __delay_cycles(1000000);

                }

            }

        } else {
            // System is toggled off
            motionDetected = false;
            turnOffArmedLED();
            turnOffRedLED();
            turnOnGreenLED();
            turnOffBuzzer();
        }
    }

    return 0;
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer_B0_CCR0(void)
{

    if(timeUnderTwoSeconds) {
        // code to trigger timer is up
        timerExpired = true;
    } else {
        if(timerCount < timeSeconds) {
           timerCount++;
        } else {
           // code to trigger timer is up
           timerExpired = true;
           timerCount = 0;
        }
    }

    TB0CCTL0 &= ~CCIFG;
}


#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {

    if(!dontRegisterButton) {
        if(P1IFG & BIT1) {
            programRunning = !programRunning;
            P1IFG &= ~BIT1;
        }
    }

}

