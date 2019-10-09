#ifndef LORA_H_
#define LORA_H_
#include <stdint.h>
#include <stddef.h>

//#define LORA_DEFAULT_SPI           SPI
//#define LORA_DEFAULT_SPI_FREQUENCY 8E6
//#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
//#define LORA_DEFAULT_DIO0_PIN      2

#define PA_OUTPUT_RFO_PIN       0
#define PA_OUTPUT_PA_BOOST_PIN  1

class LoRaClass{
public:
    LoRaClass();
    int begin(long frequency, int sf, long sbw, int CR, int Pout);

    void sleep();
    void idle();
    void FSTx();
    void Tx();
    void RxSingle();
    void CAD();

    void setFrequency(long frequency);
    void setTxPower(int Pout, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
    void setSpreadingFactor(int sf);
    void setLdoFlag();

    void setOCP(uint8_t mA);            //Over Current Protection control
    int beginPacket(int size = 0);
    int endPacket(bool async = false);
    virtual size_t write(const uint8_t *buffer, size_t size);
    long getSignalBandwidth();
    int getSpreadingFactor();
    void setSignalBandwidth(long sbw);
    void setCodingRate4(int denominator);
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);

    int parsePacket(int size = 0);
    int available();
    int read();


private:
    void explicitHeaderMode();
    void implicitHeaderMode();
    bool isTransmitting();
    uint8_t singleTransfer(uint8_t address, uint8_t value);



private:
    int _reset;
    long _frequency;
    int _implicitHeaderMode;
    int _packetIndex;
};

#endif /* LORA_H_ */
