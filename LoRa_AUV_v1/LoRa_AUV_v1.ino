uint8_t RXPacketL;                              //used to store the received packet length
uint8_t RXBUFFER[100];                           //create the buffer that received packets are copied into

uint8_t buff[] = "L-11.1111O22.2222D1U1G1P1R32W23Y32E";                        //the message to send, set one Arduino to Ping, the other to Pong
String uartData;
#define Program_Version "V1.0"
#include <SPI.h>
#include <SX127XLT.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "displayUtils.h"
#include "Settings.h"
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2x(U8G2_R0);
uint32_t TXPacketCount, startmS, endmS;
SX127XLT LT;
uint32_t RXpacketCount;
uint32_t errors;

int8_t PacketRSSI;
int8_t PacketSNR;
const byte disp_row[5] = {0, 15, 31, 47, 63};
char raw_packet[40];
uint8_t int_raw_packet[40];
uint8_t uint8_packet[sizeof(raw_packet)];
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 60000; // 1 minute period

void convertCharToUint8(const char* charArray, int length, uint8_t* uint8Array) {
  for (int i = 0; i < length; ++i) {
    uint8Array[i] = static_cast<uint8_t>(charArray[i]);
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin(SCK, MISO, MOSI, NSS);
  u8g2x.setColorIndex(1);
  u8g2x.begin();
  u8g2x.setBitmapMode(1);
  displayMainScreen(u8g2x);

  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE)) {
    Serial.println(F("LoRa Device found"));
    displayLoRaDetected(u8g2x);
  } else {
    Serial.println(F("No device responding"));
    while (1) {
      displayLoRaNotDetected(u8g2x);
    }
  }

  LT.setMode(MODE_STDBY_RC);
  LT.setPacketType(PACKET_TYPE_LORA);
  LT.setRfFrequency(Frequency, Offset);
  LT.calibrateImage(0);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);
  LT.setBufferBaseAddress(0x00, 0x00);
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
  LT.setHighSensitivity();
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RX_DONE, 0, 0);

  Serial.println();
  LT.printModemSettings();
  Serial.println();
  LT.printOperatingSettings();
  Serial.println();
  LT.printRegisters(0x00, 0x4F);
  Serial.println();
  Serial.println();

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();
}

void loop()
{
  LT.printASCIIPacket(buff, sizeof(buff));                        //print the buffer (the sent packet) as ASCII
  if (!LT.transmit(buff, sizeof(buff), 10000, TXpower, WAIT_TX))  //will return 0 if transmit error
  {
    Serial.print(F("Transmit failed"));
    LT.printIrqStatus();                                          //print IRQ status, indicates why packet transmit fail
    Serial.println();
  }
  if (LT.receive(RXBUFFER, sizeof(RXBUFFER), 2000, WAIT_RX))      //wait for a packet to arrive with 2 second (2000mS) timeout
  {
    RXPacketL = LT.readRegister(REG_RXNBBYTES);
    LT.printASCIIPacket(RXBUFFER, RXPacketL);                     //print the packet as ASCII characters
    Serial.println();
  }
  else
  {
    Serial.print(F("Receive failed"));
    LT.printIrqStatus();                                          //print IRQ status, why did packet receive fail
    Serial.println();
  }
}
