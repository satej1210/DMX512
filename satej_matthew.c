// DMX-512 controller and receiver

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

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
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 0*4)))

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define PUSH_BUTTON_MASK 16
#define PUSH_BUTTON2_MASK 1

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
#define delay1Cycle() __asm(" NOP\n")
#define delay6Cycles() __asm(" NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n")

uint8_t dmxData[512];

uint8_t i = 0;
uint8_t dec = 0;
char command[20];
char arg1[20];
char arg2[20];
int8_t enteringField = 0; //0-command, 1-arg1, 2-arg2
int8_t pos = 0;
uint16_t maxAddress = 512;
uint16_t deviceModeAddress = 0;
uint8_t continuous = 0;
uint8_t mode = 0, rxError = 0; //0-device, 1-controller
uint16_t DMXMode = 0; //0-break, 1-Mark After Break, 3-start
uint16_t rxState = 0;
uint8_t woo = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware

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

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A for UART0, port C for UART1 and port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC
            | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOD;

    SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;

    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= 0x00000001;

    GPIO_PORTD_DIR_R |= 0x00000007;
    GPIO_PORTD_DEN_R |= 0x0000000F;
    // Configure LED pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | BLUE_LED_MASK | RED_LED_MASK; // bits 1, 2, and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | BLUE_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = PUSH_BUTTON2_MASK | PUSH_BUTTON_MASK | GREEN_LED_MASK | BLUE_LED_MASK
            | RED_LED_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = PUSH_BUTTON2_MASK | PUSH_BUTTON_MASK; // enable internal pull-up for push button

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2; // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3; // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;       // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    // select UART0 to drive pins PA0 and PA1: default, added for clarity

    GPIO_PORTC_DIR_R |= 0x60;
    GPIO_PORTC_DEN_R |= 0x70;
    GPIO_PORTC_AFSEL_R |= 0x30;
    //GPIO_PORTC_PCTL_R &= 0xFFFFFF00;
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1 | SYSCTL_RCGCUART_R0; // turn-on UART0, leave other UARTs in same status
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    delay4Cycles();
    // wait 4 clock cycles
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // enable TX, RX, and module

    UART1_CTL_R = 0;
    UART1_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART1_IBRD_R = 10; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 0;
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;

    UART0_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
    NVIC_EN0_R |= 1 << (INT_UART0 - 16);         // turn-on interrupt 21 (UART0)

    UART1_IM_R = UART_IM_RXIM;
    NVIC_EN0_R |= 1 << (INT_UART1 - 16);
//
//    // Configure Timer 1 for keyboard service
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x30D40; // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

    //Initializing EEPROM
    delay6Cycles();
    while (EEPROM_EEDONE_R & 0x01)
        ;

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

    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM1 module
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    delay6Cycles();
    GPIO_PORTF_DIR_R |= 0x0E;   // make bits 1,2,3
    GPIO_PORTF_DR2R_R |= 0x0E;  // set drive strength to 2mA
    GPIO_PORTF_DEN_R |= 0x0E;   // enable bits 1,2,3 digital
    //GPIO_PORTF_AFSEL_R |= 0x0E; // select auxilary function for bits 1, 2 and 3
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6
            | GPIO_PCTL_PF3_M1PWM7;
    // SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_1_CTL_R = 0;
    PWM1_2_CTL_R = 0;                               // turn-off PWM0 generator 1
    PWM1_3_CTL_R = 0;                               // turn-off PWM0 generator 2
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    // output 3 on PWM0, gen 1b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
    // output 4 on PWM0, gen 2a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    // output 5 on PWM0, gen 2b, cmpb
    PWM1_2_LOAD_R = 256; // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 256;
    PWM1_INVERT_R =
            PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
    // invert outputs so duty cycle increases with increasing compare values
    PWM1_2_CMPB_R = 0;               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_2_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM1_3_CTL_R = PWM_3_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;

}

void Uart1Isr()
{
    if (mode == 1)
    {
        //UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;
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

    if (UART1_MIS_R & UART_MIS_RXMIS)
    {
        //if(!(UART1_FR_R & UART_FR_RXFE)){

        uint16_t U1_DR = UART1_DR_R;
        uint8_t data = U1_DR & 0xFF;
        //putsUart0(data + '0');
        //putcUart0('\n');
        if (rxState == 0){
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
        }
        if (U1_DR & 0x400)
        {        //get break bit){
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            rxState = 1;
            rxError = 0;
            //UART1_ECR_R = 0;
            //BLUE_LED = 1;
            GREEN_LED = 1;
        }
        else if (rxState == 1 && data == 0)
        {
            rxState = 2;
            GREEN_LED = 0;
        }
        else if (rxState >= 2 && rxState <= 514)
        {
            dmxData[(rxState) - 2] = data;

            rxState++;

            if (rxState == 514)
            {
                //BLUE_LED = 0;
                GREEN_LED ^= 1;
                rxState = 0;
            }
        }
        else{
            rxState = 0;
        }

        //
        //}

    }
    UART1_ICR_R = 0;

}

void putcUart1(uint8_t i)
{
    while (UART1_FR_R & UART_FR_TXFF)
        ;               // wait if uart0 tx fifo full
    UART1_DR_R = i;                                  // write character to fifo
}

void changeTimer1Value(uint32_t us)
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAILR_R = us * 40;             // turn-off timer before reconfiguring
    // reset interrupt
    // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

uint8_t seconds = 0;

void Timer1ISR(void)
{
    if (mode == 1)
    {
        //using state machine-like interrupt handling
        //Diagram Used for reference: http://www.etcconnect.com/Support/Articles/DMX-Speed.aspx
        if (DMXMode == 0)
        {
            //Break
            //send nothing
            GPIO_PORTC_AFSEL_R &= 0x00;
            GPIO_PORTC_DATA_R &= 0xDF;
            //putcUart1(0);
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
//        else
//        {
//            DMXMode = 0;
//            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
//            TIMER1_IMR_R = 0;
//
//            GPIO_PORTC_AFSEL_R |= 0x30;
//            putcUart1(0);
//            uint16_t i = 0;
//
//            for (i = 0; i < 512; ++i)
//            {
//                putcUart1(dmxData[i]);
//            }

        // }
    }
    if (mode == 0){

        if (rxState == 0 && !rxError){
            changeTimer1Value(2000000);
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            rxError = 1;
        }
        else if (rxError){
            changeTimer1Value(500000);
            GREEN_LED ^= 1;

        }
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

char ch[3];

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

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    if (!(UART0_FR_R & UART_FR_RXFE))             // wait if uart0 rx fifo empty
        return UART0_DR_R & 0xFF;                     // get character from fifo
    else
        return '\0';
}

uint8_t vall = 8;
uint8_t incr = 1;

uint16_t program, Address, opMode, setval;

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
    EEPROM_EEOFFSET_R = 0;
    deviceModeAddress = (uint16_t) EEPROM_EERDWR_R;
//    if(EEPROM_EERDWR_R == 0xFFFFFFFF)
//    {
//     EEPROM_EERDWR_R = 0x01;
//     program = EEPROM_EERDWRINC_R;
//     EEPROM_EEOFFSET_R = 1;
//     EEPROM_EERDWR_R = 0; //Starts in device mode
//     opMode = (uint16_t)EEPROM_EERDWRINC_R;
//    }
//    else{
//        program = EEPROM_EERDWRINC_R;
//        opMode = (uint16_t)EEPROM_EERDWRINC_R;
//        EEPROM_EERDWR_R = 0x04;
//        setval = EEPROM_EERDWRINC_R; //get last value of info to controller
//        Address = EEPROM_EERDWR_R;   //get device address
//    }
}

void EEWRITE(uint16_t B, uint16_t offSet, uint16_t val) //write to EEPROM at block + offset
{
    EEPROM_EEBLOCK_R = B;
    EEPROM_EEOFFSET_R = offSet;
    EEPROM_EERDWR_R = val;
}

void clearDMX(){
    uint16_t i = 0;
    for (i = 0; i < 512; ++i)
                {
                    dmxData[i] = 0;
                }
}

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
            else
            {
                putsUart0("\r\nNo Woo :(\r\n");
            }
            return 0;
        }
        else if (strcmp(command, "clear") == 0)
        {
            uint16_t i = 0;

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
    else if (mode == 0)
    {
        if (strcmp(command, "address") == 0)
        {
            putsUart0("\n\rDevice address set to: ");
            putsUart0(arg1);
            deviceModeAddress = atoi(arg1);
            EEWRITE(1, 0, deviceModeAddress);
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

bool isLetter(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

bool isNumber(char c)
{
    return (c >= '0' && c <= '9');
}

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
    putsUart0("\tmax <number of addresses>\r\n");

}

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
void wooone()
{
    int x = 0;
    for (x = 0; x < 512; x += 1)
    {
        dmxData[x] = 255;
    }

}
uint8_t up;
void animationRamp()
{
    uint16_t x;

    if (dmxData[0] == 0)
    {
        up = 1;
    }
    if (dmxData[0] == 254)
    {
        up = 0;
    }

    for (x = 0; x < 512; ++x)
    {
        if (up)
            dmxData[x] = (2 + dmxData[x]) % 256;
        else
            dmxData[x] = (dmxData[x] - 2) % 256;
    }
    waitMicrosecond(10000);

}

uint8_t RGBMode = 0;
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
uint8_t main(void)
{
    // Initialize hardware
    initHw();

    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);
    BLUE_LED = 0;

    uint16_t x = 0;

    for (x = 0; x < 512; ++x)
    {
        dmxData[x] = x % 256;
    }

    dmxData[0] = 00;
    dmxData[1] = 0;
    dmxData[2] = 0;

    getModeEE();
    // Display greeting
    putsUart0("\r\n\r\nCurrent Mode: ");
    if (mode == 0)
    {
        putsUart0("Device");
    }
    else
    {
        putsUart0("Controller");
    }

    putsUart0("\r\nCurrent Device Mode Address: ");
    putsUart0(intToChar(deviceModeAddress));
    printCommandList();
    putsUart0("\r\n>");

    // For each received character, toggle the green LED
    // For each received "1", set the red LED
    // For each received "0", clear the red LED

    while (1)
    {
        if (!PUSH_BUTTON2){
            uint8_t ix = 0;
            deviceModeAddress = 0;
            for (ix = 0; ix < 8; ++ix){
                GPIO_PORTD_DATA_R = ix;
                waitMicrosecond(100);
                uint8_t d = GPIO_PORTD_DATA_R & 0x8;

                deviceModeAddress +=  (d >> 3) << ix;


            }
            putsUart0(intToChar(deviceModeAddress));

            putcUart0('\n');
            putcUart0('\r');
            waitMicrosecond(250000);
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



        if (woo == 2)
            animationRamp();
        else if (woo == 1)
            wooone();
        else
            clearDMX();
        if (RGBMode)
        {
            GPIO_PORTF_AFSEL_R |= 0x0F;
            SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
            PWM1_2_CMPB_R = dmxData[deviceModeAddress + 0 - 1]; //red                           // red off (0=always low, 1023=always high)
            PWM1_3_CMPA_R = dmxData[deviceModeAddress + 1 - 1]; //blue                           // green off
            PWM1_3_CMPB_R = dmxData[deviceModeAddress + 2 - 1];    //green

        }
        else
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
