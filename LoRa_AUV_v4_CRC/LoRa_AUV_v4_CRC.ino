#include <SPI.h>
#include <SX127XLT.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "displayUtils.h"
#include "Settings.h"

#define Program_Version "V1.0"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2x(U8G2_R0);

uint8_t buff_jet[100];                          // the buffer to store the received UART input for transmission
int buff_jet_length = 0;                        // length of the current data in buff_jet

uint8_t RXPacketL;                              // used to store the received packet length
uint8_t RXBUFFER[100];                          // create the buffer that received packets are copied into
uint32_t TXPacketCount, startmS, endmS;
SX127XLT LT;
uint32_t RXpacketCount;
uint32_t errors;

int8_t PacketRSSI;
int8_t PacketSNR;

void txpacket_is_OK()
{
  uint16_t localCRC;
  Serial.print(F("  BytesSent,"));
  Serial.print(buff_jet_length);                    //print transmitted packet length
  localCRC = LT.CRCCCITT(buff_jet, buff_jet_length, 0xFFFF);
  Serial.print(F("  CRC,"));
  Serial.print(localCRC, HEX);                      //print CRC of transmitted packet
  Serial.print(F("  TransmitTime,"));
  Serial.print(endmS - startmS);                    //print transmit time of packet
  Serial.print(F("mS"));
  Serial.print(F("  PacketsSent,"));
  Serial.print(TXPacketCount);                      //print total of packets sent OK
  Serial.println();
}

void txpacket_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(buff_jet_length);                    //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                     //print IRQ status
  LT.printIrqStatus();                              //prints the text of which IRQs set
  Serial.println();
}

void packet_is_OK()
{
  uint16_t IRQStatus, localCRC;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register
  RXpacketCount++;
  Serial.print(millis() / 1000.0);                  //print elapsed time to Serial Monitor
  Serial.print(F("s "));
  LT.printASCIIPacket(RXBUFFER, RXPacketL);         //print the packet as ASCII characters
  localCRC = LT.CRCCCITT(RXBUFFER, RXPacketL, 0xFFFF);  //calculate the CRC, this is the external CRC calculation of the RXBUFFER
  Serial.print(F(",CRC,"));
  Serial.print(localCRC, HEX);
  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  Serial.println();
}

void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register
  Serial.print(millis() / 1000.0);                  //print elapsed time to Serial Monitor
  Serial.print(F("s "));
  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
    Serial.println();
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the device packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();                            //print the names of the IRQ registers set
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin(SCK, MISO, MOSI, NSS);
  u8g2x.setColorIndex(1);
  u8g2x.begin();
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
}

void loop() {
  // Read data from UART
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    Serial.print(receivedChar);  // Debugging: print each received character
    if (buff_jet_length < sizeof(buff_jet) - 1) {  // Ensure buffer doesn't overflow
      buff_jet[buff_jet_length] = receivedChar;
      buff_jet_length++;
    }
    // Check if the packet is complete and ready to transmit
    if (receivedChar == '*') {
      buff_jet[buff_jet_length] = '\0';  // Null-terminate the buffer to make it a valid string
      Serial.println("\nPacket received, sending...");  // Debugging: print when a packet is ready to send
      // Print and transmit the packet
      startmS = millis();
      if (LT.transmit(buff_jet, buff_jet_length, 10000, TXpower, WAIT_TX)) {  // Will return 0 if transmit error
        endmS = millis();
        TXPacketCount++;
        txpacket_is_OK();
      } else {
        txpacket_is_Error();
      }
      buff_jet_length = 0;  // Reset the buffer length after transmission
    }
  }

  // Receiving packets
  RXPacketL = LT.receive(RXBUFFER, sizeof(RXBUFFER), 60000, WAIT_RX);  // Wait for a packet to arrive with 2 second (2000ms) timeout
  PacketRSSI = LT.readPacketRSSI();  //read the received RSSI value
  PacketSNR = LT.readPacketSNR();    //read the received SNR value
  if (RXPacketL == 0) {  //if the LT.receive() function detects an error, RXpacketL is 0
    packet_is_Error();
  } else {
    packet_is_OK();
  }
}
