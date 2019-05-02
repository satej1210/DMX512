/**
 * @file satej_matthew.c
 * @author Satej Mhatre, Matthew Hilliard
 * @date 1 May 2019
 * @brief File containing everything for the DMX Controller Receiver Project. <br>
 * For CSE 4342: Embedded II Spring 2019 <br>
 * Instructor: Dr. Jason Losh<br>
 * Hardware Target:
 * -----------------------------------------------------------------------------
 * Target Platform: EK-TM4C123GXL Evaluation Board <br>
 * Target uC:       TM4C123GH6PM<br>
 * System Clock:    40 MHz<br>
 * Hardware configuration:
 * -----------------------------------------------------------------------------
 * Red LED:<br>
 *   PF1 drives an NPN transistor that powers the red LED<br>
 * Blue LED:<br>
 *   PF2 drives an NPN transistor that powers the green LED<br>
 * Green LED:<br>
 *   PF3 drives an NPN transistor that powers the green LED<br>
 * UART Interface:<br>
 *   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller<br>
 *   U1TX (PA1) and U1RX (PA0) are used for DMX Data Transmit and Receive<br>
 * Other Interface:<br>
 *   PD0, PD1, PD2, PD3 is connected to a mux that reads the value from a DIP switch<br>
 *   PF1, PF2, PF3 are also configured as PWM outputs to control servos and LEDs on-board.<br>
 * To Do:<br>
 *   PD6, PD7 will be connected to a ESP8266-01 that will serve a webpage for UART communication so that launchpad can be controlled without
 *   physically using a USB cable.<br>
 * The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port<br>
 * Configured to 115,200 baud, 8N1<br>
 */


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
/*!< Bit banding for PORTF1 Red LED */

#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
/*!< Bit banding for PORTF3 GREEN LED */

#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
/*!< Bit banding for PORTF2 Blue LED */

#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
/*!< Bit banding for PORTF4 PushButton 1 */

#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 0*4)))
/*!< Bit banding for PORTF0 PushButton 0 */


#define GREEN_LED_MASK 8
/*!< GPIO PORTF Green LED Mask */

#define RED_LED_MASK 2
/*!< GPIO PORTF Red LED Mask */

#define BLUE_LED_MASK 4
/*!< GPIO PORTF Blue LED Mask */

#define PUSH_BUTTON_MASK 16
/*!< GPIO PORTF Push Button 1 Mask */

#define PUSH_BUTTON2_MASK 1
/*!< GPIO PORTF Push Button 2 Mask */


#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
/*!< Delaying for 4 cycles */

#define delay1Cycle() __asm(" NOP\n")
/*!< Delaying for 1 cycle */

#define delay6Cycles() __asm(" NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n")
/*!< Delaying for 6 cycles */

uint8_t i = 0;
uint8_t dec = 0;
uint8_t up;

/*
 * UART0 Global Variables
 * ========================
 */

char command[20]; /*!< To Store characters from UART0 command*/
char arg1[20]; /*!< To Store characters from UART0 command 1st Argument*/
char arg2[20]; /*!< To Store characters from UART0 command 2nd Argument*/
int8_t enteringField = 0; /*!< Iterates over the different command fields while entering a command. 0: Command, 1: 1st Argument, 2: 2nd Argument*/
int8_t pos = 0; /*!< Position of the character in the entering field. */

/*
 * DMX Transmit Global Variables
 * ========================
 */

uint16_t maxAddress = 512; /*!< Maximum Number of DMX Bins to Transmit. */
uint8_t continuous = 0; /*!< Flag to indicate whether transmit of DMX is enabled or not. */
uint16_t DMXMode = 0; /*!< Mode to indicate what is being transmitted. 0: Break, 1: Mark After Break, 2: Start Code, > 2: DMX Data bins */

/*
 * DMX Receive Global Variables
 * ========================
 */

uint16_t deviceModeAddress = 0; /*!< Device Address of the Current Receiver */
uint8_t prevRX = 0;
uint8_t rxError = 0; /*!< Flag to indicate whether the receiver is in error state, ie, has not received a break in 2 secs. */
uint16_t rxState = 0; /*!< Mode to indicate what is being received. 0: Break, 1: Mark After Break, 2: Start Code, > 2: DMX Data bins. */

/*
 * DMX Special Functions Global Variables
 * ========================
 *
 */

float seconds = 0; /*!< Used by special ramp function to indicate current second count. Timer counts up seconds every 100ms. (not very accurate) */
int upR, upG, upB; /*!< Used by special ramp function for ramping logic */
int goR, goG, goB; /*!< Used by special ramp function for ramping logic */
float secondsTrigger = 0.0; /*!< Used for special ramp function to indicate the number of seconds to complete ramp. */
uint16_t dimStart = 0; /*!< Used for special ramp function to indicate the start value. */
uint16_t dimEnd = 0; /*!< Used for special ramp function to indicate the stop value of ramp function. */
float dimValue = 0; /*!< Used for special ramp function to indicate the current ramp value at time t. */
uint8_t woo = 0; /*!< Variable to indicate what special function is running. 0: Nothing, 1: Sets all addresses to 255
 , 2: Ramp Animation using Timer2, 3: Set servo angle ([14,58] -> [0,180] degrees), 4: Sweep Servo from 0-180-0, 5: Special Timer
 based ramp control. */
int servoDir = 0; /*!< Used by servoSweep to indicate direction of sweep. */
char ch[3]; /*!< For storing integer to character */
uint8_t vall = 8; /*!< For EEPROM Data */
uint8_t incr = 1; /*!< For EEPROM Data */
uint16_t program, Address, opMode, setval; /*!< For EEPROM Data */

/*
 * Launchpad Control Global Variables
 * ========================
 */
uint8_t mode = 0; /*!< Indicates the current mode of the launchpad. 0: Device, 1: Controller. */
uint8_t dmxData[512]; /*!< Array to store bins of DMX data. */
uint8_t RGBMode = 0; /*!< Flag to indicate whether in 1: full device mode or 0: normal device mode. (Full device mode: Onboard R,G,B LED has address 1,2,3 wrt device Address)
 normal device mode: device will function according to specifications.*/

/*
 * Function Definitions
 * ========================
 */
void animationRamp();
void clearStr();
char getcUart0();
void getModeEE();
char* intToChar(uint16_t x);
bool isLetter(char c);
bool isNumber(char c);
uint8_t main(void);
void printCommandList();
void putcUart1(uint8_t i);
void Uart0Isr(void);
void waitMicrosecond(uint32_t us);
void wooone();
void putsUart0(char*);
void changeTimer1Value(uint32_t);

/*
 * Subroutines
 * ========================
 */

/**
 * @brief
 *
 * Function to initialize all required hardware functions
 */
void initHw()
{

    /**
     *    Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
     */
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    /**
     *    Set GPIO ports to use APB (not needed since default configuration -- for clarity)
     * Note UART on port A must use APB
     */
    SYSCTL_GPIOHBCTL_R = 0;

    /**
     *   Enable GPIO port A for UART0, port C for UART1 and port F peripherals, and PORTD for DIP Switch
     */
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC
            | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOD;

    /**
     *  Give clock to EEPROM
     */
    SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;

    /**
     *  Unlock PORTF pin that is configured by default for NMI
     */
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= 0x00000001;

    /**
     *  Configure pins for DIP Switch Reading
     */
    GPIO_PORTD_DIR_R |= 0x00000007;
    GPIO_PORTD_DEN_R |= 0x0000000F;

    /**
     *  Configure LED Pins on PORTF
     */
    GPIO_PORTF_AFSEL_R = 0;
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | BLUE_LED_MASK | RED_LED_MASK; // bits 1, 2, and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | BLUE_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = PUSH_BUTTON2_MASK | PUSH_BUTTON_MASK | GREEN_LED_MASK
            | BLUE_LED_MASK | RED_LED_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = PUSH_BUTTON2_MASK | PUSH_BUTTON_MASK; // enable internal pull-up for push button

    /**
     *  Configure UART0
     */
    GPIO_PORTA_DIR_R |= 2; // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3; // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;       // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX; // select UART0 to drive pins PA0 and PA1: default, added for clarity

    /**
     *  Configure PORTC for UART1 Transmit
     */
    GPIO_PORTC_DIR_R |= 0x60;
    GPIO_PORTC_DEN_R |= 0x70;
    GPIO_PORTC_AFSEL_R |= 0x30;
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    /**
     *  Give clock to UART0, UART1, TIMER1, TIMER2
     */
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1 | SYSCTL_RCGCUART_R0; // turn-on UART0,1 , leave other UARTs in same status
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2;

    delay4Cycles();
    // wait 4 clock cycles

    /**
     * Configuring UART0
     */
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    /**
     * Configuring UART1
     */
    UART1_CTL_R = 0;
    UART1_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART1_IBRD_R = 10; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 0;
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;

    UART0_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
    NVIC_EN0_R |= 1 << (INT_UART0 - 16);         // turn-on interrupt 21 (UART0)

    UART1_IM_R = UART_IM_RXIM | UART_IM_TXIM;
    NVIC_EN0_R |= 1 << (INT_UART1 - 16);

    /**
     * Configuring Timer 1 for DMX Transmit and Receive
     */
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x30D40; // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

    /**
     * Configuring Timer 2 for DMX Transmit and Receive
     */
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER2_TAILR_R = 3000000; //10000000; // set load value to 2e5 for 200 Hz interrupt rate

    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER2A - 16);     // turn-on interrupt 39 (TIMER2A)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

    /**
     * EEPROM initialize and configuration from datasheet
     */
    delay6Cycles();
    while (EEPROM_EEDONE_R & 0x01);

    if (EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY
            || EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY)
    {
        putsUart0("EEPROM Init Error before reset!");
        exit(0);
    }
    else
    {
        SYSCTL_SREEPROM_R |= SYSCTL_SREEPROM_R0;
        SYSCTL_SREEPROM_R &= ~SYSCTL_SREEPROM_R0;
    }

    delay6Cycles();
    while (EEPROM_EEDONE_R & 0x01)
        ;
    if (EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY
            || EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY)
    {
        putsUart0("EEPROM Init Error after reset!");
        exit(0);
    }

    /**
     * Configuring PWM and PORTF for LEDs and Servo Control
     */
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;            // turn-on PWM1 module
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;          // enable clock for PWM
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_16; // use SysClk / 16 for PWM clock

    GPIO_PORTF_DIR_R |= 0x0E;   // make bits 1,2,3
    GPIO_PORTF_DR2R_R |= 0x0E;  // set drive strength to 2mA
    GPIO_PORTF_DEN_R |= 0x0E;   // enable bits 1,2,3 digital
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6
            | GPIO_PCTL_PF3_M1PWM7;  //Use PORTF 1,2,3 as PWM Outputs.

    SYSCTL_SRPWM_R = 0;                             // leave reset state
    PWM1_1_CTL_R = 0;                               // turn-off PWM1 generator 2
    PWM1_2_CTL_R = 0;                               // turn-off PWM1 generator 2
    PWM1_3_CTL_R = 0;                               // turn-off PWM1 generator 3

    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE; // output 3 on PWM0, gen 1b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE; // output 4 on PWM0, gen 2a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE; // output 5 on PWM0, gen 2b, cmpb

    PWM1_2_LOAD_R = 50000; // set period to 40 MHz sys clock / 16 / 50000 = 50Hz for servo control
    PWM1_3_LOAD_R = 50000; // set period to 40 MHz sys clock / 16 / 50000 = 50Hz for servo control
    PWM1_INVERT_R =
    PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV; // invert outputs so duty cycle increases with increasing compare values
    PWM1_2_CMPB_R = 0;               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_2_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM1_3_CTL_R = PWM_3_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;

}

/**
 * @brief
 *
 * Function to Handle Interrupts from UART1
 */
void Uart1Isr()
{

    //For controller mode
    if (mode == 1)
    {
        if (DMXMode - 3 < maxAddress)
        {
            UART1_DR_R = dmxData[DMXMode - 3];
            DMXMode++;
            UART1_ICR_R = UART_ICR_TXIC;
        }
        else
        {
            UART1_ICR_R = UART_ICR_TXIC;
            GPIO_PORTC_AFSEL_R &= 0x00;
            GPIO_PORTC_DATA_R &= 0xDF;
            DMXMode = 0;
            UART1_CTL_R = 0;
            changeTimer1Value(176);
            TIMER1_CTL_R |= TIMER_CTL_TAEN;

        }
    }

    //For device mode
    if (UART1_MIS_R & UART_MIS_RXMIS)
    {

        uint16_t U1_DR = UART1_DR_R;
        uint8_t data = U1_DR & 0xFF;

        //enable error Timer if rxState is 0; blinks green LED after 2 seconds if no activity.
        if (rxState == 0)
        {
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
        }

        //if you get break bit
        if (U1_DR & 0x400)
        {
            changeTimer1Value(2000000);
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            rxState = 1;
            rxError = 0;
            prevRX = 0;
            GREEN_LED = 1;
        }

        //ignore mark after break
        //get start bit
        else if (rxState == 1 && data == 0)
        {
            prevRX = 1;
            rxState = 2;

        }

        //get dmx data
        else if (rxState >= 2 && rxState <= 514)
        {
            dmxData[(rxState) - 2] = data;
            prevRX = rxState;
            rxState++;

            if (rxState == 514)
            {
                GREEN_LED ^= 1;
                rxState = 0;
            }
        }

        //turn on error state if no data
        else
        {
            rxState = 0;
        }

    }
    UART1_ICR_R = 0;

}

/**
 * @brief
 *
 * Function to send characters to UART0
 */
void putcUart1(uint8_t i)
{

    while (UART1_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART1_DR_R = i;                                  // write character to fifo
}

/**
 * @brief
 *
 * Function to change load value of Timer1
 */
void changeTimer1Value(uint32_t us)
{

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAILR_R = us * 40;             // turn-off timer before reconfiguring
    // reset interrupt
    // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

/**
 * @brief
 *
 * Function to Handle Interrupts from Timer2
 */
void Timer2ISR(void)
{

    if (woo == 2)
    {
        if (dmxData[deviceModeAddress - 1] == 0)
        {
            upR = 1;
            dmxData[deviceModeAddress - 1] = 2;
            goR = 0;
            goG = 1;
            goB = 0;
        }
        if (dmxData[deviceModeAddress - 1] == 254)
        {
            upR = 0;
            dmxData[deviceModeAddress - 1] = 252;
            goR = 0;
            goG = 1;
            goB = 0;

        }

        if (dmxData[deviceModeAddress + 1 - 1] == 0)
        {
            upG = 1;
            dmxData[deviceModeAddress + 1 - 1] = 2;
            goR = 0;
            goG = 0;
            goB = 1;
        }
        if (dmxData[deviceModeAddress + 1 - 1] == 254)
        {
            upG = 0;
            dmxData[deviceModeAddress + 1 - 1] = 252;
            goR = 0;
            goG = 0;
            goB = 1;
        }

        if (dmxData[deviceModeAddress + 2 - 1] == 0)
        {
            upB = 1;
            dmxData[deviceModeAddress + 2 - 1] = 2;
            goR = 1;
            goG = 0;
            goB = 0;
        }
        if (dmxData[deviceModeAddress + 2 - 1] == 254)
        {
            upB = 0;
            dmxData[deviceModeAddress + 2 - 1] = 252;
            goR = 1;
            goG = 0;
            goB = 0;
        }

        if (goR)
        {

            if (upR)
                dmxData[deviceModeAddress - 1] = (2
                        + dmxData[deviceModeAddress - 1]) % 256;
            else
                dmxData[deviceModeAddress - 1] = (dmxData[deviceModeAddress - 1]
                        - 2) % 256;
        }
        if (goG)
        {
            if (upG)
                dmxData[deviceModeAddress + 1 - 1] = (2
                        + dmxData[deviceModeAddress + 1 - 1]) % 256;
            else
                dmxData[deviceModeAddress + 1 - 1] = (dmxData[deviceModeAddress
                        + 1 - 1] - 2) % 256;
        }
        if (goB)
        {
            if (upB)
                dmxData[deviceModeAddress + 2 - 1] = (2
                        + dmxData[deviceModeAddress + 2 - 1]) % 256;
            else
                dmxData[deviceModeAddress + 2 - 1] = (dmxData[deviceModeAddress
                        + 2 - 1] - 2) % 256;
        }

    }

    if (woo == 4)
    {
        if (servoDir == 0)
        {
            dmxData[deviceModeAddress + 0 - 1]--;
        }
        else
        {
            dmxData[deviceModeAddress + 0 - 1]++;
        }

    }

    if (woo == 5)
    {
        seconds += 0.1;
        dimValue -= (dimStart - dimEnd) / secondsTrigger / 10;
        dmxData[deviceModeAddress - 1] = dimValue;
        if (dimStart - dimEnd > 0)
        {
            if (dimValue < dimEnd)
            {
                putsUart0("Done Ramp\n\r");
                dmxData[deviceModeAddress - 1] = dimEnd;
                woo = 0;
            }
        }
        if (dimStart - dimEnd < 0)
        {
            if (dimValue > dimEnd)
            {
                putsUart0("Done Ramp\n\r");
                dmxData[deviceModeAddress - 1] = dimEnd;
                woo = 0;
            }
        }
    }

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

/**
 * @brief
 *
 * Function to handle TIMER1 interrupts
 */
void Timer1ISR(void)
{

    if (mode == 3 && GREEN_LED == 0)
    {
        GREEN_LED ^= 1;
        changeTimer1Value(15000);

    }
    if (mode == 3 && GREEN_LED == 1)
    {
        GREEN_LED ^= 1;
        changeTimer1Value(200000);

    }
    if (mode == 1 && continuous)
    {
        //using state machine-like interrupt handling
        //Diagram Used for reference: http://www.etcconnect.com/Support/Articles/DMX-Speed.aspx
        if (DMXMode == 0)
        {
            //Break
            //send nothing
            GPIO_PORTC_AFSEL_R &= 0x00;
            GPIO_PORTC_DATA_R &= 0xDF;
            changeTimer1Value(176); //For Break
            DMXMode++;
        }
        else if (DMXMode == 1)
        {
            //Mark After Break
            GPIO_PORTC_DATA_R |= 0x20;
            changeTimer1Value(12); //For MAB
            DMXMode++;
        }
        else if (DMXMode == 2)
        {
            //Start Code with post start(2 stop bits)
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;
            GPIO_PORTC_AFSEL_R |= 0x30;
            DMXMode++;
            putcUart1(0);

        }
    }

    if (mode == 0)
    {

        if (rxState == 0 && !rxError)
        {
            changeTimer1Value(2000000);
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            rxError = 1;
        }
        else if (rxError)
        {
            changeTimer1Value(500000);
            GREEN_LED ^= 1;

        }
        else if (rxState == 1)
        {
            rxError = 0;
            rxState = 0;
        }
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

/**
 * @brief
 *
 * Function to convert integer to character for UART0
 */
char* intToChar(uint16_t x)
{

    int8_t i = 2;

    uint16_t temp = x;
    for (; i >= 0; --i)
    {
        ch[i] = '0' + temp % 10;
        temp /= 10;
    }
    return ch;
}

/**
 * @brief
 *
 * Blocking function that writes a serial character when the UART buffer is not full
 */
void putcUart0(char c)
{

    while (UART0_FR_R & UART_FR_TXFF)
        ;               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

/**
 * @brief
 *
 * Blocking function that writes a string when the UART buffer is not full
 */
void putsUart0(char* str)
{

    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

/**
 * @brief
 *
 * Blocking function that returns with serial data once the buffer is not empty
 */
char getcUart0()
{

    if (!(UART0_FR_R & UART_FR_RXFE))             // wait if uart0 rx fifo empty
        return UART0_DR_R & 0xFF;                     // get character from fifo
    else
        return '\0';
}

/**
 * @brief
 *
 * Function to get the launchpad mode from EEPROM
 */
void getModeEE()
{

    while (EEPROM_EEDONE_R & 0x01)
        ;
    delay6Cycles();
    EEPROM_EEBLOCK_R = 0;
    EEPROM_EEOFFSET_R = 2;
    mode = (uint16_t) EEPROM_EERDWR_R;
    delay6Cycles();
    EEPROM_EEBLOCK_R = 1;
    EEPROM_EEOFFSET_R = 2;
    deviceModeAddress = (uint16_t) EEPROM_EERDWR_R;
}

/**
 * @brief
 *
 * Function to write to EEPROM to set address
 */
void EEWRITE(uint16_t B, uint16_t offSet, uint16_t val)
{

    EEPROM_EEBLOCK_R = B;
    EEPROM_EEOFFSET_R = offSet;
    EEPROM_EERDWR_R = val;
}

/**
 * @brief
 *
 * Function to clear DMX data bins.
 */
void clearDMX()
{

    uint16_t i = 0;
    for (i = 0; i < 512; ++i)
    {
        dmxData[i] = 0;
    }
}

/**
 * @brief
 *
 * Function to parse commands from UART0 and execute functions or set flags.
 */
uint8_t parseCommand()
{

    if (mode == 1)
    { //controller mode
        if (strcmp(command, "device") == 0)
        {
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            UART1_IFLS_R = UART_IFLS_RX1_8;
            UART1_IM_R = UART_IM_RXIM;
            GPIO_PORTC_AFSEL_R |= 0x30;
            GPIO_PORTC_DATA_R &= 0x9F;
            UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
            UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;

            putsUart0("\n\rDevice Mode\n\r");
            mode = 0;
            EEWRITE(0, 2, 0);
            return 0;
        }
        if (strcmp(command, "seconds") == 0)
        {
            putsUart0("\n\rSetting:");
            putsUart0("\n\r Seconds:");

            putsUart0(arg1);
            secondsTrigger = atoi(arg1);
            return 0;
        }
        if (strcmp(command, "startend") == 0)
        {
            putsUart0("\n\rSetting:");
            putsUart0("\n\r Start:");

            putsUart0(arg1);
            dimStart = atoi(arg1);
            putsUart0("\n\rSetting:");
            putsUart0("\n\r End:");

            putsUart0(arg2);
            dimEnd = atoi(arg2);
            return 0;
        }
        else if (strcmp(command, "woo") == 0)
        {
            woo = atoi(arg1);
            if (woo == 1)
            {
                putsUart0("\r\nAll Addresses 255 :)\r\n");
            }
            else if (woo == 2)
            {
                putsUart0("\r\nRamp Animation :D\r\n");
            }
            else if (woo == 3)
            {
                putsUart0("\r\nServo Set :D\r\n");
            }
            else if (woo == 4)
            {
                putsUart0("\r\nServo Sweep :D\r\n");
            }
            else if (woo == 5)
            {
                TIMER2_CTL_R |= TIMER_CTL_TAEN;
                putsUart0("\r\nRamping\r\n");
                putsUart0("\n\r Start:");

                putsUart0(intToChar(dimStart));

                putsUart0("\n\rSetting:");
                putsUart0("\n\r End:");

                putsUart0(intToChar(dimEnd));
                dimValue = dimStart;
            }
            else
            {
                putsUart0("\r\nNo Woo :(\r\n");
            }
            return 0;
        }
        else if (strcmp(command, "clear") == 0)
        {

            clearDMX();

            putsUart0("\n\rCleared.\n\r");
            return 0;
        }

        else if (strcmp(command, "set") == 0)
        {
            uint16_t addr = atoi(arg1);
            if (addr > 0 && addr < 513)
            {
                putsUart0("\n\rSetting:");
                putsUart0("\n\r Address:");
                putsUart0(intToChar(addr));
                putsUart0("\n\r Value:");
                putsUart0(arg2);
                dmxData[addr - 1] = atoi(arg2);
            }

            else
            {
                putsUart0("\n\rAddresses from 1 to 512 only.\n\r");
            }

            return 0;
        }
        else if (strcmp(command, "get") == 0)
        {
            putsUart0("\n\rGetting:");
            putsUart0("\n\rAddress:");
            putsUart0(arg1);
            putsUart0("\n\rValue:");
            uint16_t addr = atoi(arg1);
            if (addr > 0)
                putsUart0(intToChar(dmxData[addr - 1]));
            else
                putsUart0("\n\rAddresses from 1 to 512 only.\n\r");

            return 0;
        }

        else if (strcmp(command, "max") == 0)
        {
            putsUart0("\n\rSetting Max to ");
            putsUart0(arg1);
            maxAddress = atoi(arg1);
            return 0;
        }
        else if (strcmp(command, "on") == 0)
        {
            putsUart0("\n\rContinuous On\n\r");
            continuous = 1;
            GPIO_PORTC_DATA_R = 0x40;
            return 0;
        }
        else if (strcmp(command, "off") == 0)
        {
            putsUart0("\n\rContinuous off\n\r");
            continuous = 0;
            return 0;
        }
        else if (strcmp(command, "controller") == 0)
        {
            UART1_IM_R = UART_IM_TXIM;
            putsUart0("\n\rAlready in Controller Mode\n\r");

            return 0;
        }
        else
        {
            putsUart0("\n\rInvalid Controller Mode Command\r\n");
            printCommandList();
            return 0;
        }
    }

    //device Mode
    else if (mode == 0)
    {
        if (strcmp(command, "address") == 0)
        {
            putsUart0("\n\rDevice address set to: ");
            putsUart0(arg1);
            deviceModeAddress = atoi(arg1);
            EEWRITE(1, 2, deviceModeAddress);
            return 0;
        }
        else if (strcmp(command, "device") == 0)
        {
            UART1_IFLS_R = UART_IFLS_RX1_8;
            UART1_IM_R = UART_IM_RXIM;
            GPIO_PORTC_AFSEL_R |= 0x30;
            GPIO_PORTC_DATA_R &= 0x9F;
            UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
            UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            putsUart0("\n\rAlready in Device Mode\n\r");
            return 0;
        }
        else if (strcmp(command, "controller") == 0)
        {
            UART1_IM_R = UART_IM_TXIM;
            GPIO_PORTC_DATA_R &= 0xDF;
            putsUart0("\n\rController Mode\n\r");
            mode = 1;
            EEWRITE(0, 2, 1);
            return 0;
        }
        else
        {
            putsUart0("\n\rInvalid Device Mode Command\n\r");
            printCommandList();
            return 0;
        }
    }
    else
    {
        putsUart0("\n\rInvalid Mode\n\r");
        return 1;
    }
}

/**
 * @brief
 *
 * Function to clear command, arg1, and arg2 arrays.
 */
void clearStr()
{

    uint8_t i = 0;
    for (; i < 20; ++i)
    {
        command[i] = '\0';
        arg1[i] = '\0';
        arg2[i] = '\0';
    }
    pos = 0;
    enteringField = 0;
}

/**
 * @brief
 *
 * Function to check if character is letter
 */
bool isLetter(char c)
{

    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

/**
 * @brief
 *
 * Function to check if character is number
 */
bool isNumber(char c)
{

    return (c >= '0' && c <= '9');
}

/**
 * @brief
 *
 * Function to print available commands to user
 */
void printCommandList()
{

    putsUart0("\n\rAvailable Commands:\r\n");
    putsUart0("For Device Mode:\r\n");
    putsUart0("\tcontroller\n\r");
    putsUart0("\taddress <address of device>\r\n");

    putsUart0("For Controller Mode:\r\n");
    putsUart0("\tdevice\r\n");
    putsUart0("\tset <address>,<value>\r\n");
    putsUart0("\tget <address>,<value>\r\n");
    putsUart0(
            "\twoo < 0 for no woo :( \r\n\t    | 1 for all addresses 255 \r\n\t    | 2 for ramp animation >\r\n");
    putsUart0(
            "\twoo < 3 for servo angle set \r\n\t    | 4 for servo sweep \r\n\t    | 5 for special ramping function >\r\n");
    putsUart0("\tmax <number of addresses>\r\n");

}

/**
 * @brief
 *
 * Function to wait for specified microseconds
 */
void waitMicrosecond(uint32_t us)
{

    __asm("WMS_LOOP0:   MOV  R1, #6");
    // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
    // 6
    __asm("             CBZ  R1, WMS_DONE1");
    // 5+1*3
    __asm("             NOP");
    // 5
    __asm("             NOP");
    // 5
    __asm("             B    WMS_LOOP1");
    // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");
    // 1
    __asm("             CBZ  R0, WMS_DONE0");
    // 1
    __asm("             NOP");
    // 1
    __asm("             B    WMS_LOOP0");
    // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");
    // ---
    // 40 clocks/us + error
}

/**
 * @brief
 *
 * Function to handle UART0 interrupts
 */
void Uart0Isr(void)
{

    char c = getcUart0();

    if (c == '\0')
    {
        return;
    }
    if (!(isLetter(c) || isNumber(c) || c == ' ' || c == '\n' || c == '\r'
            || c == 8 || c == ','))
    {
        return;
    }
    if (isLetter(c) && enteringField == 0)
    {
        command[pos++] = tolower(c);
        putcUart0(c);
    }
    else if (enteringField == 0 && c == ' ')
    {
        putcUart0(' ');
        ++enteringField;
        pos = 0;
    }
    else if (enteringField == 1 && c == ',')
    {
        putcUart0(',');
        ++enteringField;
        pos = 0;
    }
    else if (enteringField == 0 && isNumber(c))
    {
        putsUart0("\r\nInvalid Command. Resetting...\r\n");
        printCommandList();
        putcUart0('>');
        clearStr();
    }
    else if (enteringField == 1 && (isNumber(c) || isLetter(c)))
    {
        arg1[pos++] = c;
        putcUart0(c);
    }
    else if (enteringField == 2 && (isNumber(c) || isLetter(c)))
    {
        arg2[pos++] = c;
        putcUart0(c);
    }
    else if (c == '\n' || c == '\r')
    {
        putcUart0(c);
        GREEN_LED = 1;
        uint8_t ret = parseCommand();
        if (ret != 0)
        {
            putsUart0("\r\nInvalid Command\r\n");
            printCommandList();
            putcUart0('>');
            clearStr();
        }
        else
        {
            putsUart0("\r\n");
            putcUart0('>');
            clearStr();
        }

        GREEN_LED = 0;
    }
    else if (c == 8)
    {
        if (pos > 0)
        {
            if (enteringField == 2)
            {
                arg2[--pos] = '\0';

            }
            else if (enteringField == 1)
            {
                arg1[--pos] = '\0';
            }
            else
            {
                command[--pos] = '\0';
            }
            putcUart0('\b');
            putcUart0(' ');
            putcUart0('\b');

        }
        else if (pos == 0)
        {
            enteringField--;
            if (enteringField == 1)
            {
                pos = strlen(arg1);
            }
            else if (enteringField == 0)
            {
                pos = strlen(command);
            }
            else
            {
                enteringField = 0;
                pos = 0;
            }

        }
        else
        {
            if (enteringField < 0)
            {
                enteringField = 0;
            }
        }
    }
    else
    {
        putsUart0("\r\nInvalid Command. Resetting...\r\n");
        printCommandList();
        putcUart0('>');
        clearStr();
    }

    if (mode == 1 && continuous == 1)
    {
        RED_LED = 1;
    }
    else
    {
        RED_LED = 0;
    }

}

/**
 * @brief
 *
 * Function to set all DMX values to 255
 */
void wooone()
{

    int x = 0;
    for (x = 0; x < 512; x += 1)
    {
        dmxData[x] = 255;
    }

}

/**
 * @brief
 *
 * Function to sweep servo
 */
void sweepServo()
{

    if (woo == 3)
    {
        GPIO_PORTF_AFSEL_R |= 0x0F;
        SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        if (dmxData[deviceModeAddress + 0 - 1] * 100 >= 1400
                && dmxData[deviceModeAddress + 0 - 1] * 100 <= 5800)
        {
            PWM1_2_CMPB_R = dmxData[deviceModeAddress + 0 - 1] * 100;
        }

    }
    if (woo == 4)
    {
        GPIO_PORTF_AFSEL_R |= 0x0F;
        SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        if (dmxData[deviceModeAddress + 0 - 1] * 100 >= 1400
                && dmxData[deviceModeAddress + 0 - 1] * 100 <= 5800)
        {
            PWM1_2_CMPB_R = dmxData[deviceModeAddress + 0 - 1] * 100;
        }
        else if (dmxData[deviceModeAddress + 0 - 1] * 100 < 1400)
        {
            servoDir = 1;
        }
        else if (dmxData[deviceModeAddress + 0 - 1] * 100 > 5800)
        {
            servoDir = 0;
        }

    }

    else
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    }
}

/**
 * @brief
 *
 * Function to enable ramping animation
 */
void animationRamp()
{

    TIMER2_CTL_R = TIMER_CTL_TAEN;
}

/**
 * @brief
 *
 * Runs everything
 */
uint8_t main(void)
{

    // Initialize hardware
    initHw();

    getModeEE();
    putsUart0("\r\n\r\nCurrent Mode: ");
    if (mode == 0)
    {
        mode = 0;
        putsUart0("Device");
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        UART1_IFLS_R = UART_IFLS_RX1_8;
        UART1_IM_R = UART_IM_RXIM;
        GPIO_PORTC_AFSEL_R |= 0x30;
        GPIO_PORTC_DATA_R &= 0x9F;
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
        UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;
    }
    else
    {
        mode = 1;
        putsUart0("Controller");
        UART1_IM_R = UART_IM_TXIM;
        GPIO_PORTC_DATA_R &= 0xDF;

    }

    putsUart0("\r\nCurrent Device Mode Address: ");
    putsUart0(intToChar(deviceModeAddress));
    printCommandList();
    putsUart0("\r\n>");

    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);
    BLUE_LED = 0;

    //Setting initial values for dmx for testing
    uint16_t x = 0;

    for (x = 0; x < 512; ++x)
    {
        dmxData[x] = x % 256;
    }

    while (1)
    {

        //to read values from mux from DIP switch
        //NOT TESTED with DIP SWITCH
        if (!PUSH_BUTTON2)
        {
            uint8_t ix = 0;
            deviceModeAddress = 0;
            for (ix = 0; ix < 8; ++ix)
            {
                GPIO_PORTD_DATA_R = ix;
                waitMicrosecond(10000);
                uint8_t d = GPIO_PORTD_DATA_R & 0x8;

                deviceModeAddress += (d >> 4);

            }
            putsUart0(intToChar(deviceModeAddress));

            putcUart0('\n');
            putcUart0('\r');
            waitMicrosecond(250000);
            if (deviceModeAddress == 0)
            {
                deviceModeAddress++;
            }
        }

        if (!PUSH_BUTTON)
        {
            RGBMode ^= 1;
            if (RGBMode)
            {
                GPIO_PORTF_AFSEL_R = 0;

                GREEN_LED = 1;
                waitMicrosecond(200000);
                BLUE_LED = 1;
                waitMicrosecond(200000);
                RED_LED = 1;
                waitMicrosecond(250000);
                GREEN_LED = 0;
                waitMicrosecond(200000);
                BLUE_LED = 0;
                waitMicrosecond(200000);
                RED_LED = 0;
                SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;

            }
            else
            {
                GPIO_PORTF_AFSEL_R &= 0;
                SYSCTL_RCGCPWM_R &= ~SYSCTL_RCGCPWM_R1;
                GREEN_LED = 1;
                BLUE_LED = 1;
                RED_LED = 1;
                waitMicrosecond(250000);
                GREEN_LED = 0;
                BLUE_LED = 0;
                RED_LED = 0;
                waitMicrosecond(250000);
                GREEN_LED = 1;
                BLUE_LED = 1;
                RED_LED = 1;
                waitMicrosecond(250000);
                GREEN_LED = 0;
                BLUE_LED = 0;
                RED_LED = 0;
                waitMicrosecond(250000);
            }

        }

        if (woo == 3 || woo == 4)
        {

            SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
            sweepServo();

        }

        if (woo == 2)
            animationRamp();
        else
        {
            TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
        }
        if (woo == 1)
            wooone();
        if (woo == 5)
        {
            TIMER2_CTL_R |= TIMER_CTL_TAEN;
        }

        if (RGBMode && woo != 3 && woo != 4 && mode == 0)
        {

            GPIO_PORTF_AFSEL_R |= 0x0F;
            SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
            PWM1_2_CMPB_R = dmxData[deviceModeAddress + 0 - 1] * 100; //red

            PWM1_3_CMPB_R = dmxData[deviceModeAddress + 1 - 1] * 100;    //green

            PWM1_3_CMPA_R = dmxData[deviceModeAddress + 2 - 1] * 100; //blue

        }
        else
        {
            SYSCTL_RCGCPWM_R |= ~SYSCTL_RCGCPWM_R1;
            GPIO_PORTF_AFSEL_R = 0;
            if (mode == 0)
            {
                if (dmxData[deviceModeAddress - 1] != 0)
                {
                    BLUE_LED = 1;
                }
                else
                {
                    BLUE_LED = 0;
                }
            }
        }
    }
}
