#include "SPI.h"
#include <msp430.h>


LoRaSPIClass::LoRaSPIClass()
{
    LoRaSPIClass(1, 2);
};

LoRaSPIClass::LoRaSPIClass(uint8_t cs_gpio, uint8_t usci_module)
{
    _cs_gpio = cs_gpio;
    _usci_module = usci_module;
};

void LoRaSPIClass::initSPI()
{
    //--- SPI pinOut Configuration ---//
    P1OUT  |= BIT5;                             // Set P1.5 HIGH (LoRa chip is selected)
    P1DIR  |= BIT5;                             // Set P1.5 as OUTPUT for CS
    P1SEL0 |= BIT4 + BIT6 + BIT7;               // P1.4 = CLK, P1.6 = MOSI, P1.7=MISO
    P1SEL1 &= ~(BIT4 + BIT6 + BIT7);            // P1.4 = CLK, P1.6 = MOSI, P1.7=MISO

    //--- USCI module configuration for SPI transfert ---//
    UCB0CTLW0 = UCSWRST;                        // USCI - configuration mode
    UCB0CTLW0|= UCMSB + UCMST + UCSYNC;         // SET SPI : 3-pin, 8-bit SPI master
    UCB0CTL1 |= UCSSEL_2;                       // Clock source Select
    UCB0BR0 = 2;                                // Clock divider by 1
    UCB0BR1 = 0;                                //
    UCB0CTLW0 &= ~UCSWRST;                      // USCI - SPI Activate
}

uint8_t LoRaSPIClass::readRegister(uint8_t address)
{
    return singleTransfer(address & 0x7f,0x00);
}

void LoRaSPIClass::writeRegister(uint8_t address, uint8_t value)
{
    singleTransfer(address | 0x80,value);
}

uint8_t LoRaSPIClass::singleTransfer(uint8_t address, uint8_t value)
{
    uint8_t response;

    P1OUT &= ~BIT5;                         // CS (NSS) LOW - Chip Select Lora Chip
    while (!(UCB0IFG & UCTXIFG));           // USCI_B0 TX buffer ready?
        UCB0TXBUF = address;                // Send address over SPI to Slave
    while (!(UCB0IFG & UCRXIFG));           // USCI_B0 RX Received?
        response = UCB0RXBUF;       // Store received data

    while (!(UCB0IFG & UCTXIFG));           // USCI_B0 TX buffer ready?
        UCB0TXBUF = value;          // Send value over SPI to Slave
    while (!(UCB0IFG & UCRXIFG));           // USCI_B0 RX Received?
        response = UCB0RXBUF;
    P1OUT |= BIT5;                          // CS (NSS) HIGH - Release Lora Chip

    return response;
}
