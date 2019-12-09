#ifndef LORAWAN_H_
#define LORAWAN_H_

#include "LoRa.h"
#include <stdint.h>
//#define DEBUG

//--- LoRA Regional params ------------------------------------------------------------//
#define EU863       // Used in Europe
//#define US902     // Used in USA, Canada and South America
//#define AU915     // Used in Australia
//#define AS920     // Used in Asia


typedef enum lrw_regions
{
  _EU863,
  _US902,
  _AU915,
  _AS920,
  _LAST_REGION,
} lrw_region_t;

typedef enum lrw_class
{
  CLASS_A,
  CLASS_B,
  CLASS_C,
} lrw_class_t;

/*** RFM channels options */
typedef enum lrw_channels
{
  CH0,
  CH1,
  CH2,
  CH3,
  CH4,
  CH5,
  CH6,
  CH7,
  MULTI,
} lrw_channel_t;

/** RFM fixed datarate, dependent on region */
typedef enum lrw_datarates
{
  SF12BW125,
  SF11BW125,
  SF10BW125,
  SF9BW125,
  SF8BW125,
  SF7BW125,
  SF7BW250,
  FSCK,
} lrw_datarate_t;


/**************************************************************************/
/*!
    @brief  LoRaWan Class
*/
/**************************************************************************/
class LoRaWan
{
    public:
        LoRaWan();
        bool begin();
        bool begin( lrw_region_t region, lrw_channel_t channel, lrw_datarate_t datarate, uint8_t txpower, bool confirmUplink );
        int  setChannel(lrw_channel_t channel = CH0);
        int  setDatarate(lrw_datarate_t datarate = SF7BW125);
        int  setPower(int8_t txpower = 14);
        void sendData(uint8_t Frame_Port, const uint8_t *Data, const uint8_t Data_Length );

    public:
        LoRaRadioClass lora;

    private:
        uint8_t _class;
        uint8_t _channel;
        uint8_t _datarate;
        uint8_t _txpower;
        bool    _isMultiChan;
        bool    _isAdaptativeDatarate;

        uint16_t _frameCounter;  /// frame counter
        uint8_t  _txrandomNum;

        static const unsigned char LoRa_Frequency[8][3];
        static const unsigned char S_Table[16][16];

    protected:
        void send_Packet(unsigned char *buffer, unsigned char size);

        //--- XOR, CRC, MIC functions -----------------------------------------------------------//
        void Encrypt_Payload(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction);
        void Calculate_MIC(unsigned char *Data, unsigned char *Final_MIC, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction);
        void Generate_Keys(unsigned char *K1, unsigned char *K2);
        void Shift_Left(unsigned char *Data);
        void XOR(unsigned char *New_Data, unsigned char *Old_Data);

        //--- AES Encryption functions -----------------------------------------------------------//
        void AES_HW_Encrypt(unsigned char *Data, unsigned char *Key);
        void AES_Encrypt(unsigned char *Data, unsigned char *Key);
        void AES_Add_Round_Key(unsigned char *Round_Key, unsigned char(*State)[4]);
        unsigned char AES_Sub_Byte(unsigned char Byte);
        void AES_Shift_Rows(unsigned char(*State)[4]);
        void AES_Mix_Collums(unsigned char(*State)[4]);
        void AES_Calculate_Round_Key(unsigned char Round, unsigned char *Round_Key);
};

#endif /* LORAWAN_H_ */
