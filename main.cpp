#include <msp430.h> 
#include "LoRa.h"
#include <stdint.h>

LoRaClass Dragino;

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                   //Stop watchDog timer

    PMMCTL0_H = PMMPW_H;                        //Unlock PMM registers
    PM5CTL0 &= ~LOCKLPM5;                       //Unlock I/O pins
    PMMCTL0_H = 0x00;                           //Lock PMM registers

    // Configure GPIO
      P1OUT = 0x00;
      P1DIR = 0xFF;

      P2OUT = 0;
      P2DIR = 0xFF;

      P3OUT = 0;
      P3DIR = 0xFF;

      P4OUT = 0;
      P4DIR = 0xFF;

      P5OUT = 0;
      P5DIR = 0xFF;

      P6OUT = 0;
      P6DIR = 0xFF;

      P7OUT = 0;
      P7DIR = 0xFF;

      P8OUT = 0;
      P8DIR = 0xFF;

      P9OUT = 0;
      P9DIR = 0xFF;

      P10OUT = 0;
      P10DIR = 0xFF;

      PJOUT = 0;
      PJSEL0 = BIT4 | BIT5;                     // For XT1
      PJDIR = 0xFFFF;

    // Configure GPIO for pushbutton interrupt
    P1OUT |= BIT1;                             // Pull-up resistor on P1.1
    P1REN |= BIT1;                             // Select Pullup or pulldown resistor enabled
    P1DIR &= ~BIT1;                             // Set all but P1.1 to output direction
    P1IES = BIT1;                             // P1.1 high-to-low transition
    P1IFG = 0;                                // Clear all P1 interrupt flags
    P1IE = BIT1;                              // P1.1 interrupt enabled

    // Configure GPIO for LoRa interrupt
    P2OUT &= ~BIT0 + ~BIT3 + ~BIT4;                            // Pull-down resistor on P2.3
    P2REN |= BIT0 + BIT3 + BIT4;                             // Select Pullup or pulldown resistor enabled
    P2DIR &= ~BIT0 + ~BIT3 + ~BIT4;                             // Set all but P2.3 to output direction
    P2IES &= ~BIT0 + ~BIT3 + ~BIT4;                             // P2.3 low-to-high transition
    P2IFG &= ~BIT0 + ~BIT3 + ~BIT4;                            // Clear P2.3 interrupt flags
    P2IE |= BIT0 + BIT3 + BIT4;                              // P2.3 interrupt enabled


    //Left LED configuration
    P1DIR |= BIT0;
    P9DIR |= BIT7;

    //SPI CONFIGURATION------------------------------

    //SPI pinOut configuration
    P1OUT |= BIT5;                              // Set P1.5 HIGH (LoRa chip is selected)
    P1DIR |= BIT5;                              // Set P1.5 as OUTPUT for CS
    P1SEL0 |= BIT4 + BIT6 + BIT7;                // P1.4 = CLK, P1.6 = MOSI, P1.7=MISO
    P1SEL1 &= ~(BIT4 + BIT6 + BIT7);            // P1.4 = CLK, P1.6 = MOSI, P1.7=MISO

    //USCI module configuration for SPI
    UCB0CTLW0 = UCSWRST;                        //USCI configuration
    UCB0CTLW0|= UCMSB + UCMST + UCSYNC;         // 3-pin, 8-bit SPI master
    UCB0CTL1 |= UCSSEL_2;
    UCB0BR0 = 2;                                // Clock divider by 1
    UCB0BR1 = 0;                                //
    UCB0CTLW0 &= ~UCSWRST;                      //USCI Back to operation

    //LORA CONFIGURATION-----------------------------
    P2OUT |= BIT6;                              //Set P2.6 HIGH (LOW == LoRa chip is in reset mode)
    P2DIR |= BIT6;                              //Set P2.6 as OUTPUT for LoRa reset
    P2OUT &= ~BIT6;                             //Set P2.6 LOW (LOW == LoRa chip is in reset mode)
    P2OUT |= BIT6;                              //Set P2.6 HIGH (LOW == LoRa chip is in reset mode)

    Dragino.begin(868100000,12,125000,5,20);            //begin(frequency, SF, bandwidth, CodingRate, Pout)

    Dragino.writeRegister(0x40, 0X00);                  //DIO mapping 1
    Dragino.writeRegister(0x41, 0X00);                  //DIO mapping 2
    Dragino.writeRegister(0x12, 0Xff);                  //Clear RegIrq Flags
    P2IFG = 0;                                          // Clear all P1 interrupt flags
    P2IFG = 0;                                          // Clear all P2 interrupt flags

    Dragino.CAD();

    //Main loop--------------------------------------
    while (1)
    {
        __bis_SR_register(LPM0_bits | GIE);     // Enter LPM0 w/interrupt
        __no_operation();                       // For debugger
        Dragino.CAD();
    }
}

// Port 1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{

  P1OUT |= BIT0;    // P1.0 LED = ON

  Dragino.beginPacket();
  Dragino.write("0123456789", 11);
  Dragino.endPacket();

  P1OUT &= ~BIT0;    // P1.0 LED = OFF

  P1IFG &= ~BIT1;                           // Clear P1.1 IFG

  __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
}


// Port 2 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(P2IV, P2IV_P2IFG7))     //switch(__even_in_range(P2IV, P2IV_P2IFG3))
    {
       case P2IV_P2IFG3:     // LORA CADDONE INTERRUPT :
              P1OUT |= BIT0;    // P1.0 LED = ON

              P2IFG &= ~BIT3;                           // Clear P2.3 IFG
              Dragino.writeRegister(0x12, Dragino.readRegister(0x12) | 0x04);   //CLEAR REGIRQ CADDONE

              P1OUT &= ~BIT0;    // P1.0 LED = OFF

              __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
              break;



      case P2IV_P2IFG4:     // LORA CADDETECTED INTERRUPT :
          P9OUT |= BIT7;                            // P9.7 LED = ON

          Dragino.RxSingle();

          P2IFG &= ~BIT4;                           // Clear P2.4 IFG
          Dragino.writeRegister(0x12, Dragino.readRegister(0x12) | 0x01);   //CLEAR REGIRQ CADDETECTED

          P9OUT &= ~BIT7;                            // P9.7 LED = OFF

          __bis_SR_register_on_exit(LPM0_bits);     // Enter LPM0 while waiting for RXDONE
          break;



      case P2IV_P2IFG0:     // LORA RX_DONE INTERRUPT :

              P1OUT |= BIT0;    // P1.0 LED = ON
              P9OUT |= BIT7;    // P9.7 LED = ON

              int packetSize = Dragino.parsePacket();
                   if (packetSize > 0) {
                           char payload;
                           while (Dragino.available() > 0) {
                               char inputChar  =(char)Dragino.read();
                               payload += inputChar;
                           }
                         }

          P2IFG &= ~BIT0;                           // Clear P2.0 IFG
          Dragino.writeRegister(0x12, 0x00);        //CLEAR REGIRQ

          P1OUT &= ~BIT0;    // P1.0 LED = OFF
          P9OUT &= ~BIT7;    // P9.7 LED = OFF

          __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
          break;

    }
}


