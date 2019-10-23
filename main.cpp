#include <msp430.h> 
#include "LoRaWan.h"
#include <stdint.h>

//--- LORA ABP KEYS parameters----------------------------------------------------------------------------------------------//
// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Device Address (MSB)
uint8_t DevAddr[4] = { 0x11, 0x22, 0x33, 0x44 };
//--------------------------------------------------------------------------------------------------------------------------//

LoRaWan LoraRadio;

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                   // Stop WatchDog timer

    // Configure GPIO
    P1OUT = 0; P1DIR = 0xFF;
    P2OUT = 0; P2DIR = 0xFF;
    P3OUT = 0; P3DIR = 0xFF;
    P4OUT = 0; P4DIR = 0xFF;
    P5OUT = 0; P5DIR = 0xFF;
    P6OUT = 0; P6DIR = 0xFF;
    P7OUT = 0; P7DIR = 0xFF;
    P8OUT = 0; P8DIR = 0xFF;
    P9OUT = 0; P9DIR = 0xFF;
    P10OUT = 0;P10DIR = 0xFF;

    PJOUT = 0;
    PJSEL0 = BIT4 | BIT5;                     // For XT1 - LFXT 32Khz Osc
    PJDIR = 0xFFFF;

    PMMCTL0_H = PMMPW_H;                      // Unlock PMM registers
    PM5CTL0 &= ~LOCKLPM5;                     // Unlock I/O pins
    PMMCTL0_H = 0x00;                         // Lock PMM registers

    // Configure GPIO for pushbutton interrupt
    P1OUT |= BIT1;                            // Pull-up resistor on P1.1
    P1REN |= BIT1;                            // Select Pullup or pulldown resistor enabled
    P1DIR &= ~BIT1;                           // Set all but P1.1 to output direction
    P1IES = BIT1;                             // P1.1 high-to-low transition
    P1IFG = 0;                                // Clear all P1 interrupt flags
    P1IE = BIT1;                              // P1.1 interrupt enabled

    // LED configuration
    P1DIR |= BIT0;
    P9DIR |= BIT7;

    // Start LoRa Radio
    LoraRadio.begin( _EU863, CH0, SF12BW125, 15, false);
    //LoraRadio.lora.CAD();

    //Main loop--------------------------------------
    while (1)
    {
        __bis_SR_register(LPM0_bits | GIE);     // Enter LPM0 w/interrupt
        __no_operation();                       // For debugger
        //LoraRadio.lora.CAD();
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
    P1IFG &= ~BIT1;   // Clear P1.1 IFG
    P1OUT |= BIT0;    // P1.0 LED = ON

    LoraRadio.sendData( 80, "0123456789", 11 );

    P1OUT &= ~BIT0;    // P1.0 LED = OFF
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
            P1OUT |= BIT0;   // P1.0 LED = ON
            P2IFG &= ~BIT3;  // Clear P2.3 IFG

            LoraRadio.lora.clearCADDONE();

            P1OUT &= ~BIT0;  // P1.0 LED = OFF
            __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
            break;

       case P2IV_P2IFG4:     // LORA CADDETECTED INTERRUPT
            P9OUT |= BIT7;   // P9.7 LED = ON
            P2IFG &= ~BIT4;  // Clear P2.4 IFG

            // Clear CAD Irq and Start RX packet
            LoraRadio.lora.clearCADDETECTED();
            LoraRadio.lora.RxSingle();

            P9OUT &= ~BIT7;  // P9.7 LED = OFF
            __bis_SR_register_on_exit(LPM0_bits);     // Enter LPM0 while waiting for RXDONE
            break;

        case P2IV_P2IFG0:     // LORA RX_DONE INTERRUPT :
            P2IFG &= ~BIT0;   // Clear P2.0 IFG
            P1OUT |= BIT0;    // P1.0 LED = ON
            P9OUT |= BIT7;    // P9.7 LED = ON

            // Get Lora Packet Payload
            int packetSize = LoraRadio.lora.parsePacket();
               if (packetSize > 0) {
                       char payload;
                       while (LoraRadio.lora.available() > 0) {
                           char inputChar = (char)LoraRadio.lora.read();
                           payload += inputChar;
                       }
                     }
            LoraRadio.lora.clearRXDONE();

            P1OUT &= ~BIT0;    // P1.0 LED = OFF
            P9OUT &= ~BIT7;    // P9.7 LED = OFF
            __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
            break;
        }
}
