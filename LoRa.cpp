#include <msp430.h>
#include "LoRa.h"
#include <stdint.h>

// SX127x - registers map
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
#define REG_RX_TIMEOUT           0x1f
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_SYNC_CONFIG          0x27
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_DIO_MAPPING_2        0x41
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_FSTX                0x02
#define MODE_TX                  0x03
#define MODE_FSRX                0x04
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_CAD_DETECTED_MASK      0x01
#define IRQ_FSSH_CHANGE_MASK       0x02
#define IRQ_CAD_DONE_MASK          0x04
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_VALID_HEADER_MASK      0x10
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_RX_TIMEOUT_MASK        0x80

#define MAX_PKT_LENGTH              255



LoRaRadioClass::LoRaRadioClass()
{
    _reset_pin = LORA_DEFAULT_RESET_PIN;
    _spi = LoRaSPIClass();

    // LoRa Mode
    _mode = 0;
    _restoreMode = 0;

    // LoRa Configuration parameters
    _frqMSB = 0; _frqMID = 0; _frqLSB = 0;
    _spreadingFactor = 12;
    _bandwidth = 7;
    _codingRate = 1;
    _Pout = 14;
    _lnaBoost = 1;

    // LoRa Low-Level - Packet Parameters
    _implicitHeaderMode = 0;
    _packetIndex = 0;
};

void LoRaRadioClass::initGPIO() {
    // Initialize All GPIO needed for DIO IRQ
    // Configure GPIO for LoRa DIO interrupts (here on) : P2.0, P2.3, P2.4
    P2OUT &= ~BIT0 + ~BIT3 + ~BIT4;             // Pull-down resistor on
    P2REN |=  BIT0 +  BIT3 +  BIT4;             // Select Pullup or pulldown resistor enabled
    P2DIR &= ~BIT0 + ~BIT3 + ~BIT4;             // Set Input Direction
    P2IES &= ~BIT0 + ~BIT3 + ~BIT4;             // Interrupt low-to-high transition
    P2IFG &= ~BIT0 + ~BIT3 + ~BIT4;             // Clear Interrupt flags
    P2IE  |=  BIT0 +  BIT3 +  BIT4;             // Interrupt enabled
}

void LoRaRadioClass::reset() {
    // Reset LORA Module
    P2OUT |= BIT6;                              // Set P2.6 HIGH (LOW == LoRa chip is in reset mode)
    P2DIR |= BIT6;                              // Set P2.6 as OUTPUT (LoRa reset pin)

    P2OUT &= ~BIT6;                             // Set P2.6 LOW (LOW == LoRa chip is in reset mode)
    _delay_cycles(1);
    P2OUT |= BIT6;                              // Set P2.6 HIGH (HIGH == Active mode)
    _delay_cycles(5);
}

int LoRaRadioClass::begin(long frequency, int sf, long sbw, int CR, int Pout)
{
    // Start and configure SPI module
    _spi.initSPI();
    // Init LoRa Radio module via reset line
    reset();

    // Check SPI + Lora version register
    uint8_t checkVersion = _spi.readRegister(REG_VERSION);
    if( checkVersion != 0x12 )
        return -1;

    // Init Radio in LoRa Long Range Mode - SLEEP Mode (Configuration mode)
    // by default 0x09 (Mode FSK - STBY and LF mode ON)
    sleep();
    uint8_t checkMode = _spi.readRegister(REG_OP_MODE);
    if (checkMode != 0x80){
        return -1;
    }

    // Configure LORA Radio Parameters
    setFrequency(frequency);                                    // 868MHz by default
    setSignalBandwidth(sbw);                                    // 125kHz by default
    setSpreadingFactor(sf);                                     // 7 by default
    setCodingRate(CR);                                          // CR 4/5 by default

    // Set Tx Output Power (and Pin mode)
    setTxPower(Pout, PA_OUTPUT_RFO_PIN);
    // Rx Timeout set to 37 symbols (Default = 64 symbols)
    _spi.writeRegister(REG_RX_TIMEOUT, 0x25);
    //Preamble length set to 8 symbols (Default = 8 symbols)
    // 0x08 + 4 = 12
    _spi.writeRegister(REG_PREAMBLE_MSB,0x00);
    _spi.writeRegister(REG_PREAMBLE_LSB,0x08);
    //Set LoRa - Sync Word (0x12=Private Network, or 0x34=LoRaWan Networks)
    //_spi.writeRegister(REG_SYNC_CONFIG, 0x17);   // (Default value = 0x17 Sync on)
    //_spi.writeRegister(REG_SYNC_WORD,   0x34);   // (Default value = 0x12 private)

    // set FIFO base addresses
    _spi.writeRegister(REG_FIFO_TX_BASE_ADDR, 0x80);            // 0x80 by default
    _spi.writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);            // 0x00 by default

    //set LNA boost HF
    _spi.writeRegister(REG_LNA, _spi.readRegister(REG_LNA) | 0x03);  //bits 1-0 0x00 by default (not boosted)

    // Set LNA Gain adjusted by the AGC loop (Default = 0)
    _spi.writeRegister(REG_MODEM_CONFIG_3, _spi.readRegister(REG_MODEM_CONFIG_3) | 0x04);    // 0x00 by default (Gain auto)

    // Set IRQ Mapping and Clear DIO
    _spi.writeRegister(REG_DIO_MAPPING_1, 0x00);                // DIO mapping 1
    _spi.writeRegister(REG_DIO_MAPPING_1, 0x00);                // DIO mapping 2
    _spi.writeRegister(REG_IRQ_FLAGS, 0xFF);                    // Clear RegIrq Flags

    // Keep in sleep mode
    sleep();
    return 0;
}

void LoRaRadioClass::setCodingRate(int denominator)
{
    if (denominator < 5)
        denominator = 5;
    else if (denominator > 8)
        denominator = 8;

    int cr = denominator - 4;
    _spi.writeRegister(REG_MODEM_CONFIG_1, (_spi.readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr <<1));
}

void LoRaRadioClass::setSpreadingFactor(int sf)
{
    uint8_t reg;

    if (sf <6)
        sf = 6;
    else if (sf > 12)
        sf = 12;

    if (sf == 6) {
        _spi.writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        _spi.writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        _spi.writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        _spi.writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    reg = _spi.readRegister(REG_MODEM_CONFIG_2) & 0x0f;
    reg = reg | ((sf << 4) & 0xf0);
    _spi.writeRegister(REG_MODEM_CONFIG_2, reg);

    setLdoFlag();
}

void LoRaRadioClass::setLdoFlag()
{
    //Section 4.1.1.5
    long symbolDuration = 1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()) );

    //Section 4.1.1.6
    uint8_t ldoOnMask = 0x00;
    if( symbolDuration > 16 )
        ldoOnMask = 0x08;

    uint8_t config3 = _spi.readRegister(REG_MODEM_CONFIG_3);
    config3 = config3 | ldoOnMask;
    _spi.writeRegister(REG_MODEM_CONFIG_3, config3);
    _spi.readRegister(REG_MODEM_CONFIG_3);
}

int LoRaRadioClass::getSpreadingFactor()
{
    return _spi.readRegister(REG_MODEM_CONFIG_2) >> 4;
}

long LoRaRadioClass::getSignalBandwidth()
{
    uint8_t bw = (_spi.readRegister(REG_MODEM_CONFIG_1) >> 4);
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

void LoRaRadioClass::setFrequency(long frequency)
{
    uint64_t frf = ((uint64_t)frequency << 19) /32000000;
    _frqMSB = (uint8_t)(frf >> 16);
    _frqMID = (uint8_t)(frf >> 8);
    _frqLSB = (uint8_t)(frf >> 0);
    _spi.writeRegister(REG_FRF_MSB, _frqMSB);
    _spi.writeRegister(REG_FRF_MID, _frqMID);
    _spi.writeRegister(REG_FRF_LSB, _frqLSB);
}

void LoRaRadioClass::setFrequency(uint8_t frqMSB, uint8_t frqMID, uint8_t frqLSB)
{
    _frqMSB = frqMSB;
    _frqMID = frqMID;
    _frqLSB = frqLSB;
    _spi.writeRegister(REG_FRF_MSB, _frqMSB);
    _spi.writeRegister(REG_FRF_MID, _frqMID);
    _spi.writeRegister(REG_FRF_LSB, _frqLSB);
}

void LoRaRadioClass::setSignalBandwidth(long sbw)
{
    uint8_t reg;

    if (sbw <= 7800){
        _bandwidth = 0;
    } else if (sbw <= 10400){
        _bandwidth = 1;
    } else if (sbw <= 15600){
        _bandwidth = 2;
    } else if (sbw <= 20800){
        _bandwidth = 3;
    } else if (sbw <= 31250){
        _bandwidth = 4;
    } else if (sbw <= 41700){
        _bandwidth = 5;
    } else if (sbw <= 62500){
        _bandwidth = 6;
    } else if (sbw <= 125000){
        _bandwidth = 7;
    } else if (sbw <= 250000){
        _bandwidth = 8;
    } else /*if (sbw > 250000)*/ {
        _bandwidth = 9;
    }

    reg = _spi.readRegister(REG_MODEM_CONFIG_1) & 0x0f;
    reg = reg | (_bandwidth << 4);
    _spi.writeRegister(REG_MODEM_CONFIG_1, reg);

    setLdoFlag();
}

void LoRaRadioClass::setTxPower(int Pout, int outputPin)
{
    if( PA_OUTPUT_RFO_PIN == outputPin ) {
        //RFO
        if( Pout < 0 )
            Pout = 0;
        else if( Pout > 15 )
            Pout=15;

        _spi.writeRegister(REG_PA_CONFIG, 0x70 | Pout);
    } else {
        //PA BOOST
        if( Pout > 17 ) {
            if( Pout > 20 )
                Pout = 20;

            // subtract 3 from Pout, so 18 - 20 maps to 15 - 17
            Pout -= 3;

            //High Power +20 dBm Operation
            _spi.writeRegister(REG_PA_DAC, 0x87);
            setOCP(140);
        } else {
            if( Pout < 2 )
                Pout = 2;

            //Default value PA_HF/LF or +17dBm
            _spi.writeRegister(REG_PA_DAC, 0x84);
            setOCP(100);
        }
        _spi.writeRegister(REG_PA_CONFIG, PA_BOOST | (Pout - 2));
    }
}

void LoRaRadioClass::setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
        ocpTrim = (mA - 45) / 5;
    } else if (mA <=240){
        ocpTrim = (mA + 30) / 10;
    }

    _spi.writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRaRadioClass::idle()
{
    _mode = MODE_LONG_RANGE_MODE | MODE_STDBY;
    _spi.writeRegister(REG_OP_MODE, _mode);
}

void LoRaRadioClass::sleep()
{
    _mode = MODE_LONG_RANGE_MODE | MODE_SLEEP;
    _spi.writeRegister(REG_OP_MODE, _mode);
}

void LoRaRadioClass::FSTx()
{
    _mode = MODE_LONG_RANGE_MODE | MODE_FSTX;
    _spi.writeRegister(REG_OP_MODE, _mode);
}

void LoRaRadioClass::Tx()
{
    _mode = MODE_LONG_RANGE_MODE | MODE_TX;
    _spi.writeRegister(REG_OP_MODE, _mode);

}

void LoRaRadioClass::RxSingle()
{
    if (_spi.readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
        // not currently in RX Single mode
        _mode = MODE_LONG_RANGE_MODE | MODE_RX_SINGLE;
        _spi.writeRegister(REG_FIFO_ADDR_PTR, 0);
        _spi.writeRegister(REG_OP_MODE, _mode);
    }
}

void LoRaRadioClass::CAD()
{
    _mode = MODE_LONG_RANGE_MODE | MODE_CAD;
    _spi.writeRegister(REG_OP_MODE, _mode);

}

int LoRaRadioClass::beginPacket(int implicitHeader)
{
    if( isTransmitting() )
        return 0;

    _restoreMode = _mode;
    if( implicitHeader )
        implicitHeaderMode();
    else
        explicitHeaderMode();

    // reset FIFO address and payload lenght
    _spi.writeRegister(REG_PAYLOAD_LENGTH, 0);
    _spi.writeRegister(REG_FIFO_ADDR_PTR, 0x80);
    _spi.writeRegister(REG_PAYLOAD_LENGTH, 0);

    // Put in standby mode (to enable FIFO)
    idle();
    return 1;
}

bool LoRaRadioClass::isTransmitting()
{
    if((_spi.readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX){
        return true;
    }

    if(_spi.readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK){
        //clear IRQ's
        _spi.writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return false;
}

void LoRaRadioClass::explicitHeaderMode()
{
    _implicitHeaderMode = 0;
    _spi.writeRegister(REG_MODEM_CONFIG_1, _spi.readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaRadioClass::implicitHeaderMode()
{
    _implicitHeaderMode = 1;
    _spi.writeRegister(REG_MODEM_CONFIG_1, _spi.readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

int LoRaRadioClass::endPacket(bool async)
{
    //clear IRQ's TX_DONE
    clearTXDONE();

    // put in TX mode
    Tx();

    if (async){
        //grace time is required for the radio
        _delay_cycles(150);     //delayMicroseconds(150);
    } else {
        //wait for TX done
        while ((_spi.readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0){
             // yield();        // Yield() to give an OS schedule
        }
        //clear IRQ's TX_DONE
        _spi.writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    // Restore Mode before beginPacket
    _mode = _restoreMode;
    _spi.writeRegister(REG_OP_MODE, _restoreMode);
    return 1;
}

int LoRaRadioClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = _spi.readRegister(REG_IRQ_FLAGS);

  if( size > 0 ) {
    implicitHeaderMode();
    _spi.writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  _spi.writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if( _implicitHeaderMode )
      packetLength = _spi.readRegister(REG_PAYLOAD_LENGTH);
    else
      packetLength = _spi.readRegister(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    _spi.writeRegister(REG_FIFO_ADDR_PTR, _spi.readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  }

  return packetLength;
}

int LoRaRadioClass::available()
{
  return( _spi.readRegister(REG_RX_NB_BYTES) - _packetIndex );
}

int LoRaRadioClass::read()
{
  if( !available() )
    return -1;

  _packetIndex++;
  return _spi.readRegister(REG_FIFO);
}

size_t LoRaRadioClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = _spi.readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
      _spi.writeRegister(REG_FIFO, buffer[i]);
  }

  _spi.readRegister(REG_FIFO_ADDR_PTR);

  // update length
  _spi.writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  _spi.readRegister(REG_PAYLOAD_LENGTH);

  return size;
}

void LoRaRadioClass::clearRXDONE() {
    _spi.writeRegister(REG_IRQ_FLAGS, _spi.readRegister(REG_IRQ_FLAGS) | IRQ_RX_DONE_MASK);   //CLEAR REGIRQ RX_DONE
}

void LoRaRadioClass::clearTXDONE() {
    _spi.writeRegister(REG_IRQ_FLAGS, _spi.readRegister(REG_IRQ_FLAGS) | IRQ_TX_DONE_MASK);   //CLEAR REGIRQ TX_DONE
}

void LoRaRadioClass::clearCADDONE() {
    _spi.writeRegister(REG_IRQ_FLAGS, _spi.readRegister(REG_IRQ_FLAGS) | 0x04);   //CLEAR REGIRQ CAD_DONE
}

void LoRaRadioClass::clearCADDETECTED() {
    _spi.writeRegister(REG_IRQ_FLAGS, _spi.readRegister(REG_IRQ_FLAGS) | 0x04);   //CLEAR REGIRQ CAD_DETECTED
}

void LoRaRadioClass::clearALL() {
    _spi.writeRegister(REG_IRQ_FLAGS, 0xFF);   //CLEAR ALL IRQ
}
