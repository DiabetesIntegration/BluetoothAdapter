/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430G2xx3 Demo - Timer_A, Ultra-Low Pwr UART 9600 Echo, 32kHz ACLK
//
//  Description: Use Timer_A CCR0 hardware output modes and SCCI data latch
//  to implement UART function @ 9600 baud. Software does not directly read and
//  write to RX and TX pins, instead proper use of output modes and SCCI data
//  latch are demonstrated. Use of these hardware features eliminates ISR
//  latency effects as hardware insures that output and input bit latching and
//  timing are perfectly synchronised with Timer_A regardless of other
//  software activity. In the Mainloop the UART function readies the UART to
//  receive one character and waits in LPM3 with all activity interrupt driven.
//  After a character has been received, the UART receive function forces exit
//  from LPM3 in the Mainloop which configures the port pins (P1 & P2) based
//  on the value of the received byte (i.e., if BIT0 is set, turn on P1.0).
//  ACLK = TACLK = LFXT1 = 32768Hz, MCLK = SMCLK = default DCO
//  //* An external watch crystal is required on XIN XOUT for ACLK *//  
//
//               MSP430G2xx3
//            -----------------
//        /|\|              XIN|-
//         | |                 | 32kHz
//         --|RST          XOUT|-
//           |                 |
//           |   CCI0B/TXD/P1.1|-------->
//           |                 | 9600 8N1
//           |   CCI0A/RXD/P1.2|<--------
//
//  D. Dang
//  Texas Instruments Inc.
//  December 2010
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
#include <msp430.h>
//------------------------------------------------------------------------------
// Hardware-related definitions
//------------------------------------------------------------------------------
#define UART_TXD   0x01                     // TXD on P1.1 (Timer0_A.OUT0)
#define UART_RXD   0x02                     // RXD on P1.2 (Timer0_A.CCI1A)
//------------------------------------------------------------------------------
// Conditions for 9600 Baud SW UART, SMCLK = 1MHz
//------------------------------------------------------------------------------
#define UART_TBIT_DIV_2     (1000000 / (9600 * 2))
#define UART_TBIT           (1000000 / 9600)
//------------------------------------------------------------------------------
// LED Defines
//------------------------------------------------------------------------------
#define LED_0 BIT0 
#define LED_1 BIT6
#define LED_OUT P1OUT
#define LED_DIR P1DIR
//------------------------------------------------------------------------------
// Global variables used for full-duplex UART communication
//------------------------------------------------------------------------------
unsigned int txData;                        // UART internal variable for TX
unsigned char rxBuffer;                     // Received UART character
//------------------------------------------------------------------------------
// Global variables used for NFC comms
//------------------------------------------------------------------------------
char setupArray1[] =   {0x02  /* Set protocol command */ ,
                        0x02  /* length of data to follow */ ,
                        0x01  /* code for ISO/IEC 15693 */ ,
                        0x0D}; /* Wait for SOF, 10% modulation, append CRC */

char setupArray2[] =   {0x04  /* Send Receive CR95HF command */ ,
                        0x02  /* length of data to follow */ ,
                        0x02  /* request Flags byte */ ,
                        0x2B /* Inventory Command for ISO/IEC 15693 */,
                        0x00}; /* mask length for inventory command */
int setupPosition = 0;
int i =0, j=0;
char string1[200];
char c;
//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void TimerA_UART_init(void);
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);


//------------------------------------------------------------------------------
// main()
//------------------------------------------------------------------------------
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer
    if (CALBC1_1MHZ==0xFF)                  // If calibration constant erased
    {                                           
      while(1);                               // do not load, trap CPU!!    
    }
    
    DCOCTL = 0;                             // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                  // Set DCOCLK to 1MHz
    DCOCTL = CALDCO_1MHZ;
    P2OUT = 0x00;                           // Initialize all GPIO
    P2SEL = UART_TXD + UART_RXD;            // Timer function for TXD/RXD pins
    P2DIR = 0xFF & ~UART_RXD;               // Set all pins but RXD to output
    P1OUT = 0x00;
    P1DIR = 0xFF;
    
    LED_DIR |= (LED_0 + LED_1); // Set P1.0 and P1.6 to output direction
    LED_OUT |= (LED_0 + LED_1); // Set the LEDs off
    P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 =  BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
    P1OUT |= BIT2;
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 17;                            // 1MHz 9600
    UCA0BR1 = 0;                              // 1MHz 9600
    UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  
    __delay_cycles(100);
    P1OUT &= ~BIT2;
    __delay_cycles(10);
    P1OUT |= BIT2;
    __delay_cycles(10000);
    
    IE2 |= UCA0TXIE;                          // Enable USCI_A0 RX interrupt
    UCA0TXBUF = 0x02;
    
    __enable_interrupt();
    
    TimerA_UART_init();                     // Start Timer_A UART
    TimerA_UART_print("G2xx2 TimerA UART\r\n");
    TimerA_UART_print("READY.\r\n");
    
    for (;;)
    {
        // Wait for incoming character
        __bis_SR_register(LPM0_bits + GIE);
        
        if(rxBuffer=='r'){
          TimerA_UART_print("Wooooo\r\n");
        }
    }
}
//------------------------------------------------------------------------------
// Function configures Timer_A for full-duplex UART operation
//------------------------------------------------------------------------------
void TimerA_UART_init(void)
{
    TA1CCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TA1CCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TA1CTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
}
//------------------------------------------------------------------------------
// Outputs one byte using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_tx(unsigned char byte)
{
    while (TA1CCTL0 & CCIE);                 // Ensure last char got TX'd
    TA1CCR0 = TAR;                           // Current state of TA counter
    TA1CCR0 += UART_TBIT;                    // One bit time till first bit
    TA1CCTL0 = OUTMOD0 + CCIE;               // Set TXD on EQU0, Int
    txData = byte;                          // Load global variable
    txData |= 0x100;                        // Add mark stop bit to TXData
    txData <<= 1;                           // Add space start bit
}
//------------------------------------------------------------------------------
// Prints a string over using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_print(char *string)
{
    while (*string) {
        TimerA_UART_tx(*string++);
    }
}
//------------------------------------------------------------------------------
// Timer_A UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) Timer_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static unsigned char txBitCnt = 10;
    TA1CCR0 += UART_TBIT;                    // Add Offset to CCRx
    if (txBitCnt == 0) {                    // All bits TXed?
        TA1CCTL0 &= ~CCIE;                   // All bits TXed, disable interrupt
        txBitCnt = 10;                      // Re-load bit counter
    }
    else {
        if (txData & 0x01) {
          TA1CCTL0 &= ~OUTMOD2;              // TX Mark '1'
        }
        else {
          TA1CCTL0 |= OUTMOD2;               // TX Space '0'
        }
        txData >>= 1;
        txBitCnt--;
    }
}      
//------------------------------------------------------------------------------
// Timer_A UART - Receive Interrupt Handler
//------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) Timer_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static unsigned char rxBitCnt = 8;
    static unsigned char rxData = 0;
    switch (__even_in_range(TA1IV, TA1IV_TAIFG)) { // Use calculated branching
        case TA1IV_TACCR1:                        // TACCR1 CCIFG - UART RX
            TA1CCR1 += UART_TBIT;                 // Add Offset to CCRx
            if (TA1CCTL1 & CAP) {                 // Capture mode = start bit edge
                TA1CCTL1 &= ~CAP;                 // Switch capture to compare mode
                TA1CCR1 += UART_TBIT_DIV_2;       // Point CCRx to middle of D0
            }
            else {
                rxData >>= 1;
                if (TA1CCTL1 & SCCI) {            // Get bit waiting in receive latch
                    rxData |= 0x80;
                }
                rxBitCnt--;
                if (rxBitCnt == 0) {             // All bits RXed?
                    rxBuffer = rxData;           // Store in global variable
                    rxBitCnt = 8;                // Re-load bit counter
                    TA1CCTL1 |= CAP;              // Switch compare to capture mode
                    __bic_SR_register_on_exit(LPM0_bits);  // Clear LPM0 bits from 0(SR)
                }
            }
            break;
    }
}
//------------------------------------------------------------------------------

// USCI A0/B0 Transmit ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  
  if(0==setupPosition){
    UCA0TXBUF = setupArray1[i++];
    if(i>=sizeof setupArray1){
      IE2 |= UCA0RXIE;    // TX -> RXed character
      IE2 &= ~UCA0TXIE;
    }
  } else if(1==setupPosition){
    UCA0TXBUF = setupArray2[i++];
    if(i>=sizeof setupArray2){
      j=0;
      IE2 |= UCA0RXIE;    // TX -> RXed character
      IE2 &= ~UCA0TXIE;
    }
  }
  //UCA0TXBUF = 0x55;                 // TX next character

  /*if (i == sizeof string1){                 // TX over?
    IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
    i=0;
  }*/
}


//  Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  TimerA_UART_print("Interrupt Fired\r\n");
  string1[j++] = UCA0RXBUF;
  if(0==setupPosition){
    if(j>=2 && string1[0] == 0x00 && string1[1] == 0x00){
      __delay_cycles(100);
      setupPosition++;
      IE2 &= ~UCA0RXIE;
      IE2 |= UCA0TXIE;
      i=0;
    }
  } else if(1==setupPosition){
    if(j>8){
      __delay_cycles(100);
      setupPosition++;
      IE2 &= ~UCA0RXIE;
      IE2 |= UCA0TXIE;
      P1OUT &= ~(LED_0 + LED_1);
      i=0;
    } else if(j==2&&string1[0] !=128){
      __delay_cycles(100);
      i=0;
      IE2 &= ~UCA0RXIE;
      IE2 |= UCA0TXIE;
    }
    
  }
    
      // TX -> RXed character
  /*if(UCA0RXBUF=='\n'){
    IE2 |= UCA0TXIE;    // TX -> RXed character
    i = 0;
  }*/
}