#ifndef LORA_SPI_H_
#define LORA_SPI_H_
#include <stdint.h>

#define LORA_DEFAULT_RESET_PIN  9
#define LORA_DEFAULT_USCI_SPI   B0


class LoRaSPIClass{
  private:
    int _cs_gpio;
    int _usci_module;

  protected:
    uint8_t singleTransfer(uint8_t address, uint8_t value);

  public:
    LoRaSPIClass();
    LoRaSPIClass(uint8_t cs_gpio, uint8_t usci_module);

    void    initSPI();
    uint8_t readRegister(uint8_t address);
    void    writeRegister(uint8_t address, uint8_t value);
};

#endif /* LORA_SPI_H_ */
