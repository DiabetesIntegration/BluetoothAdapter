/*--------------------------------------------
Breahting Light code for MSP430
By Adam Liddell

--------------------------------------------*/

#include <msp430.h>

int i =0;
char string1[20];

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
  P1SEL = BIT1 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7 ;                     // P1.1 = RXD, P1.2=TXD P1.4 = SS, P1.5 = SPICLK P1.6 = SOMI P1.7 = SIMO
  P1SEL2 = BIT1 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7 ;                    // P1.1 = RXD, P1.2=TXD P1.4 = SS, P1.5 = SPICLK P1.6 = SOMI P1.7 = SIMO
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 17;                            // 1MHz 9600
  UCA0BR1 = 0;                              // 1MHz 9600
  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCIA state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
  
  UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_2;                     // SMCLK
  UCB0BR0 |= 0x02;                          // /2
  UCB0BR1 = 0;                              //
  UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCIB state machine**
  IE2 |= UCB0RXIE;                          // Enable USCI0 RX interrupt

  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  if(IFG2&UCA0TXIFG){                        // Check for UART TX
    UCA0TXBUF = 0x55;     // Load TX buffer
  }
  
  if(IFG2&UCB0TXIFG)                        // Check for SPI TX
    _NOP();                 // Load TX buffer
  
  _NOP();
}


// Similarly for the RX vector...

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
  if(IFG2&UCA0RXIFG){
    //string1[i++] = UCA0RXBUF;
    if(UCA0RXBUF==0x55){
      IE2 |= UCA0TXIE;    // TX -> RXed character
      i = 0;
    }                 // Store RX'ed UART data
  }
  
  if(IFG2&UCB0RXIFG)
    _NOP();                // Store RX'ed SPI data
}