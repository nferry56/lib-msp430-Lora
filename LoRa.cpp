#include <msp430.h>
#include "LoRa.h"
#include <stdint.h>

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

LoRaClass::LoRaClass() :
    _reset(LORA_DEFAULT_RESET_PIN),
    _implicitHeaderMode(0),
    _frequency(0)
{

};

int LoRaClass::begin(long frequency, int sf, long sbw, int CR, int Pout)
{
    //Check version
    uint8_t checkVersion = readRegister(REG_VERSION);
    if (checkVersion != 0x12){
        return 0;
    }

    sleep();                                                    //0x09 by default (Mode STBY and LF mode ON)
    uint8_t checkMode = readRegister(REG_OP_MODE);
    if (checkMode != 0x80){
        return 0;
    }

    setSignalBandwidth(sbw);                                    //125kHz by default
    setFrequency(frequency);                                    //433MHz by default

    setSpreadingFactor(sf);                                     //7 by default
    uint8_t checkSF = readRegister(REG_MODEM_CONFIG_2) & 0xf0;
    if (checkSF != sf << 4){
        return 0;
    }

    setCodingRate4(CR);                                         //CR 4/5 by default
    uint8_t checkCR = readRegister(REG_MODEM_CONFIG_1) & 0x0e;
    if (checkCR != (CR -4) << 1){
        return 0;
    }

    setTxPower(Pout);                                          //

    //set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0x80);               //0x80 by default
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);               //0x00 by default

    //set LNA boost HF
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);     //bits 1-0 0x00 by default (not boosted)

    writeRegister(REG_MODEM_CONFIG_3, readRegister(REG_MODEM_CONFIG_3) | 0x04);                  //0x00 by default(Gain auto)

    idle();

    return 1;

}

void LoRaClass::setCodingRate4(int denominator)
{
    if (denominator < 5){
        denominator = 5;
    } else if (denominator > 8) {
        denominator = 8;
    }

    int cr = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr <<1));
}

void LoRaClass::setSpreadingFactor(int sf)
{
    uint8_t reg;

    if (sf <6){
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }

    if (sf == 6) {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    reg = readRegister(REG_MODEM_CONFIG_2) & 0x0f;
    reg = reg | ((sf << 4) & 0xf0);
    writeRegister(REG_MODEM_CONFIG_2, reg);
    setLdoFlag();
}

void LoRaClass::setLdoFlag()
{
    //Section 4.1.1.5
    long symbolDuration = 1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()) );

    //Section 4.1.1.6

    bool ldoOn = 0;

    if (symbolDuration > 16) {
        ldoOn = 1;
    }

    uint8_t ldoOnMask = 0x00;
    if (ldoOn) {
        ldoOnMask = 0x08;
    }

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    config3 = config3 | ldoOnMask;
    writeRegister(REG_MODEM_CONFIG_3, config3);
    readRegister(REG_MODEM_CONFIG_3);
}

int LoRaClass::getSpreadingFactor()
{
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

long LoRaClass::getSignalBandwidth()
{
    uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);
    switch (bw) {
    case 0: return 7800;
    case 1: return 10400;
    case 2: return 15600;
    case 3: return 20800;
    case 4: return 31250;
    case 5: return 41700;
    case 6: return 62500;
    case 7: return 125000;
    case 8: return 250000;
    case 9: return 500000;
    }

    return -1;
}


uint8_t LoRaClass::readRegister(uint8_t address)
{
    return singleTransfer(address & 0x7f,0x00);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
    uint8_t response;

    P1OUT &= ~BIT5;                         //SPI CS Dragino
    while (!(UCB0IFG & UCTXIFG));           // USCI_B0 TX buffer ready?
                UCB0TXBUF = address;        // Send address over SPI to Slave
    while (!(UCB0IFG & UCRXIFG));           // USCI_B0 RX Received?
                response = UCB0RXBUF;       // Store received data

    while (!(UCB0IFG & UCTXIFG));           // USCI_B0 TX buffer ready?
                UCB0TXBUF = value;          // Send value over SPI to Slave
    while (!(UCB0IFG & UCRXIFG));           // USCI_B0 RX Received?
                response = UCB0RXBUF;
    P1OUT |= BIT5;                          //SPI CS Dragino

    return response;
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
    singleTransfer(address | 0x80,value);
}

void LoRaClass::setFrequency(long frequency)
{
    _frequency = frequency;
    uint64_t frf = ((uint64_t)frequency << 19) /32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRaClass::setSignalBandwidth(long sbw)
{
    int bw;
    uint8_t reg;

    if (sbw <= 7800){
        bw = 0;
    } else if (sbw <= 10400){
        bw = 1;
    } else if (sbw <= 15600){
        bw = 2;
    } else if (sbw <= 20800){
        bw = 3;
    } else if (sbw <= 31250){
        bw = 4;
    } else if (sbw <= 41700){
        bw = 5;
    } else if (sbw <= 62500){
        bw = 6;
    } else if (sbw <= 125000){
        bw = 7;
    } else if (sbw <= 250000){
        bw = 8;
    } else /*if (sbw > 250000)*/ {
        bw = 9;
    }

    reg = readRegister(REG_MODEM_CONFIG_1) & 0x0f;
    reg = reg | (bw << 4);
    writeRegister(REG_MODEM_CONFIG_1, reg);
    setLdoFlag();

}

void LoRaClass::setTxPower(int Pout, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin){
        //RFO
        if(Pout < 0){
            Pout = 0;
        } else  if (Pout >15){
            Pout=15;
        }
        writeRegister(REG_PA_CONFIG, 0x70 | Pout);

    } else {
        //PA BOOST
        if (Pout >17){
            if (Pout>20){
                Pout = 20;
            }

            // subtract 3 from Pout, so 18 - 20 maps to 15 - 17
            Pout -= 3;

            //High Power +20 dBm Operation
            writeRegister(REG_PA_DAC, 0x87);
            setOCP(140);
        } else{
            if (Pout <2){
                Pout = 2;
            }
            //Default value PA_HF/LF or +17dBm
            writeRegister(REG_PA_DAC, 0x84);
            setOCP(100);
        }
        writeRegister(REG_PA_CONFIG, PA_BOOST | (Pout - 2));
    }
}

void LoRaClass::setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
        ocpTrim = (mA - 45) / 5;
    } else if (mA <=240){
        ocpTrim = (mA + 30) / 10;
    }

    readRegister(REG_OCP);
    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
    readRegister(REG_OCP);
}

void LoRaClass::idle()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaClass::FSTx()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | 0x02);
}

void LoRaClass::Tx()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void LoRaClass::RxSingle()
{
    if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
        //not currently in RX mode
        writeRegister(REG_FIFO_ADDR_PTR, 0);
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
}

void LoRaClass::CAD()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

int LoRaClass::beginPacket(int implicitHeader)
{
    if(isTransmitting()){
        return 0;
    }

    //put in standby mode
    idle();

    if (implicitHeader){
        implicitHeaderMode();
    } else{
        explicitHeaderMode();
    }

    //resetFIFO address and payload lenght
    writeRegister(REG_FIFO_ADDR_PTR, 0x80);
    writeRegister(REG_PAYLOAD_LENGTH, 0);

    return 1;
}

bool LoRaClass::isTransmitting()
{
    if((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX){
        return true;
    }

    if(readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK){
        //clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return false;
}

void LoRaClass::explicitHeaderMode()
{
    _implicitHeaderMode = 0;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
    _implicitHeaderMode = 1;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

int LoRaClass::endPacket(bool async)
{
    //clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

    //put in TX mode
    readRegister(REG_OP_MODE);
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    readRegister(REG_OP_MODE);

    if (async){
        //grace time is required for the radio
        _delay_cycles(150);     //delayMicroseconds(150);
    } else {
        //wait for TX done
        while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0){
            //yield(); TO CHECK WITH NICOLAS IF REALLY USEFULL
        }
        //clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 1;
}

int LoRaClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  }

  return packetLength;
}

int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  readRegister(REG_FIFO_ADDR_PTR);

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  readRegister(REG_PAYLOAD_LENGTH);

  return size;
}
