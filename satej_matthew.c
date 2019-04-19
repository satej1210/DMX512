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

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

uint8_t dmxData[512];

uint8_t i = 0;
uint8_t dec = 0;
char command[20];
char arg1[20];
char arg2[20];
int8_t enteringField = 0; //0-command, 1-arg1, 2-arg2
int8_t pos = 0;
int maxAddress = 512;
int deviceModeAddress = 0;
uint8_t continuous = 0;
uint8_t mode = 1; //0-controller, 1-device
uint16_t DMXMode = 0; //0-break, 1-Mark After Break, 3-start
uint16_t rxState = 0;
uint8_t woo = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware


void initHw();
void Uart1Isr();
void Timer1ISR(void);
void changeTimerValue(uint32_t us);
uint8_t parseCommand();



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
            | SYSCTL_RCGC2_GPIOF;

    // Configure LED pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK; // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs

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
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_STP2;
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;

    UART0_IM_R = UART_IM_RXIM;                       // turn-on RX interrupt
    NVIC_EN0_R |= 1 << (INT_UART0 - 16);         // turn-on interrupt 21 (UART0)

    UART1_IM_R = UART_IM_TXIM;
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

}

void Uart1Isr(){
    if (mode == 0){
        if (DMXMode - 3 < maxAddress){
            UART1_DR_R = dmxData[DMXMode - 3];
            DMXMode++;
            UART1_ICR_R = UART_ICR_TXIC;
        }
        else{
            UART1_ICR_R = UART_ICR_TXIC;
            GPIO_PORTC_AFSEL_R &= 0x00;
            GPIO_PORTC_DATA_R &= 0xDF;
            DMXMode = 0;
            UART1_CTL_R = 0;
            changeTimerValue(176);
            TIMER1_CTL_R |= TIMER_CTL_TAEN;

        }
    }
    if (mode == 1){
        if (UART1_RSR_R & 0x04 == 0x04){//get break bit){
            rxState = 1;
            UART1_ECR_R = 0;
        }
        else if (rxState == 1 && UART1_DR_R == 0){
            rxState = 2;
        }
        else if (rxState >= 2){
            dmxData[(rxState++) - 2] = UART1_DR_R;
        }
    }
}

void putcUart1(uint8_t i)
{
    while (UART1_FR_R & UART_FR_TXFF)
        ;               // wait if uart0 tx fifo full
    UART1_DR_R = i;                                  // write character to fifo
}

void changeTimerValue(uint32_t us)
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAILR_R = us * 40;             // turn-off timer before reconfiguring
    // reset interrupt
    // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void Timer1ISR(void)
{
    if (mode == 0)
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
            changeTimerValue(176); //For Break
            DMXMode++;
        }
        else if (DMXMode == 1)
        {
            //Mark After Break
            GPIO_PORTC_DATA_R |= 0x20;
            changeTimerValue(12); //For MAB
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
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
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

uint8_t parseCommand()
{
    if (mode == 0)
    { //controller mode
        if (strcmp(command, "device") == 0)
        {
            putsUart0("Device Mode\n");
            mode = 1;
            return 0;
        }
        else if(strcmp(command, "woo") == 0){
            if(woo == 1)
                woo = 0;
            else
                woo = 1;
            return 0;
        }
        else if (strcmp(command, "clear") == 0)
        {
            uint16_t i = 0;

                     for (i = 0; i < 512; ++i)
                     {
                         dmxData[i] = 0;
                     }

            putsUart0("Cleared\n");
            return 0;
        }
        else if (strcmp(command, "set") == 0)
        {
            putsUart0("Setting \n");
            putsUart0("Address:");
            putsUart0(arg1);
            putsUart0("Value:");
            putsUart0(arg2);
            dmxData[atoi(arg1) - 1] = atoi(arg2);
            return 0;
        }
        else if (strcmp(command, "get") == 0)
        {
            putsUart0("Getting \n");
            putsUart0("Address:");
            putsUart0(arg1);
            putsUart0("Value \n");
            uint8_t num = dmxData[atoi(arg1) - 1];
            uint8_t i = 2;
            char ch[] = {0,0,0};
            while(num > 0){
                ch[i--] = num % 10;
                num /= 10;
                //putcUart0((i+'0'));
            }
            for(i = 0; i < 3; ++i){
                putcUart0(ch[i] + '0');
            }


            return 0;
        }
        else if (strcmp(command, "max") == 0)
        {
            putsUart0("Setting max \n");
            putsUart0(arg1);
            maxAddress = atoi(arg1);
            return 0;
        }
        else if (strcmp(command, "on") == 0)
        {
            putsUart0("continuous\n");
            continuous = 1;
            GPIO_PORTC_DATA_R = 0x40;
            return 0;
        }
        else if (strcmp(command, "off") == 0)
        {
            putsUart0("continuous off\n");
            continuous = 0;
            return 0;
        }
        else if (strcmp(command, "controller") == 0)
        {
            putsUart0("Already in Controller Mode\n");
            return 0;
        }
        else
        {
            putsUart0("Invalid Controller Mode Command\n");
            return 0;
        }
    }
    else if (mode == 1)
    {
        if (strcmp(command, "address") == 0)
        {
            putsUart0("Device address set to: ");
            putsUart0(arg1);
            deviceModeAddress = atoi(arg1);
            return 0;
        }
        else if (strcmp(command, "device") == 0)
        {
            putsUart0("Already in Device Mode\n");
            return 0;
        }
        else if (strcmp(command, "controller") == 0)
        {
            putsUart0("Controller Mode\n");
            mode = 0;
            return 0;
        }
        else
        {
            putsUart0("Invalid Device Mode Command\n");
            return 0;
        }
    }
    else
    {
        putsUart0("Invalid Mode\n");
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
    }
    else if (enteringField == 0 && c == ' ')
    {
        ++enteringField;
        pos = 0;
    }
    else if (enteringField == 1 && c == ',')
    {
        ++enteringField;
        pos = 0;
    }
    else if (enteringField == 0 && isNumber(c))
    {
        putsUart0("\r\nInvalid Command. Resetting...\r\n");
        clearStr();
    }
    else if (enteringField == 1 && (isNumber(c) || isLetter(c)))
    {
        arg1[pos++] = c;
    }
    else if (enteringField == 2 && (isNumber(c) || isLetter(c)))
    {
        arg2[pos++] = c;
    }
    else if (c == '\n' || c == '\r')
    {
        GREEN_LED = 1;
        uint8_t ret = parseCommand();
        if (ret != 0)
        {
            putsUart0("\r\nInvalid Command\r\n");
            clearStr();
        }
        else
        {
            putsUart0("\r\n");
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
        clearStr();
    }

    if (mode == 0 && continuous == 1)
    {
        RED_LED = 1;
        //UART1_DR_R = 'w';
    }
    else
    {
        RED_LED = 0;
        //UART1_DR_R = 1;
    }

}

void animationRamp(){
    if(woo == 1){
            if(i>=255){
                dec = 1;

            }
            if(i<=0){
                dec = 0;
            }
            if(dec == 0){
                dmxData[0] = i+=1;
                                    dmxData[1] = i+=1;
                                    dmxData[2] = i+=1;
            }
            else{
            dmxData[0] = i-=1;
                                dmxData[1] = i-=1;
                                dmxData[2] = i-=1;
            }
            waitMicrosecond(80000);
    }


}

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

    dmxData[0] = 00;
    dmxData[1] = 1;
    dmxData[2] = 2;


    // Display greeting
    putsUart0("\nCommand\r\n");
    putsUart0("Enter string followed by new line:\r\n");
    putcUart0('>');

    // For each received character, toggle the green LED
    // For each received "1", set the red LED
    // For each received "0", clear the red LED


    while (1)
    {
        animationRamp();
    }
}
