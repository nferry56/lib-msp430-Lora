#ifndef LORA_H_
#define LORA_H_
#include "SPI.h"
#include <stdint.h>
#include <stddef.h>


#define PA_OUTPUT_RFO_PIN       0
#define PA_OUTPUT_PA_BOOST_PIN  1


class LoRaRadioClass{
  private:
    // Hardware LORA Chip Interface
    LoRaSPIClass _spi;
    int _reset_pin;

    // LoRa Mode
    uint8_t _mode;
    uint8_t _restoreMode;

    // LoRa Configuration parameters
    uint8_t _frqMSB, _frqMID, _frqLSB;
    uint8_t _spreadingFactor;
    uint8_t _bandwidth;
    uint8_t _codingRate;
    uint8_t _Pout;
    uint8_t _lnaBoost;

    // LoRa Low-Level - Packet Parameters
    int _implicitHeaderMode;
    int _packetIndex;

  private:
    void explicitHeaderMode();
    void implicitHeaderMode();
    bool isTransmitting();

  public:
    LoRaRadioClass();
    void initGPIO();
    void reset();
    int  begin(long frequency, int sf, long sbw, int CR, int Pout);

    //--- Change LORA Mode ------------------------------------------------//
    void sleep();
    void idle();
    void FSTx();
    void Tx();
    void RxSingle();
    void CAD();

    //--- Set LORA Parameters : FREQ, SF, BW, CR --------------------------//
    void setFrequency(long frequency);
    void setFrequency(uint8_t frqMSB, uint8_t frqMID, uint8_t frqLSB);
    void setSpreadingFactor(int sf);
    void setSignalBandwidth(long sbw);
    void setCodingRate(int denominator);
    void setTxPower(int Pout, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
    void setOCP(uint8_t mA);            //Over Current Protection control
    void setLdoFlag();

    //--- Get LORA Parameters --------------------------------------------//
    long getFrequency();
    int  getSpreadingFactor();
    long getSignalBandwidth();
    int  getCodingRate();
    int  getTxPower();

    //--- Handle Send Packet Stream --------------------------------------//
    int beginPacket(int size = 0);
    int endPacket(bool async = false);
    virtual size_t write(const uint8_t *buffer, size_t size);

    //--- Handle Read Packet Stream -------------------------------------//
    int parsePacket(int size = 0);
    int available();
    int read();

    //--- Handle Lora IRQ - Interface -----------------------------------//
    void clearRXDONE();
    void clearTXDONE();
    void clearCADDONE();
    void clearCADDETECTED();
    void clearALL();
};

#endif /* LORA_H_ */
