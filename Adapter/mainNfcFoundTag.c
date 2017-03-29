/*--------------------------------------------
Breahting Light code for MSP430
By Adam Liddell

--------------------------------------------*/

#include <msp430.h>

#define LED_0 BIT0 
#define LED_1 BIT6
#define LED_OUT P1OUT
#define LED_DIR P1DIR

char setupArray1[] =   {0x02  /* Set protocol command */ ,
                        0x02  /* length of data to follow */ ,
                        0x01  /* code for ISO/IEC 15693 */ ,
                        0x0D}; /* Wait for SOF, 10% modulation, append CRC */

char setupArray2[] =   {0x04  /* Send Receive CR95HF command */ ,
                        0x03  /* length of data to follow */ ,
                        0x02  /* request Flags byte */ ,
                        0x01 /* Inventory Command for ISO/IEC 15693 */,
                        0x00}; /* mask length for inventory command */
int setupPosition = 0;
int i =0, j=0;
char string1[200];
char c;

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
  {											
    while(1);                               // do not load, trap CPU!!	
  }
  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;
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
  IE2 |= UCA0TXIE;                          // Enable USCI_A0 RX interrupt
  UCA0TXBUF = 0x02;
  
  __delay_cycles(100);
  P1OUT &= ~BIT2;
  __delay_cycles(10);
  P1OUT |= BIT2;
  __delay_cycles(10000);

  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
}

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
    if(j>8 && string1[0] == 128){
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