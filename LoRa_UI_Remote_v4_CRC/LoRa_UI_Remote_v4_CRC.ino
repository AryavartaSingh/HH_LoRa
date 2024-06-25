//libraries
#include <Keypad.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ezButton.h>
#include "displayUtils.h"
#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
#include <TinyGPS.h>
SX127XLT LT;
//definitions
#define ROWS 4
#define COLS 4
void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);
char keyMap[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'.', '0', '-', 'D'}
};
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;
HardwareSerial GPSPort(1);
int16_t PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;
int bufferIndex = 0;
uint8_t RXPacketL;                              //used to store the received packet length
uint8_t RXBUFFER[100];                           //create the buffer that received packets are copied into
const byte disp_row[5] = {0, 15, 31, 47, 63};
ezButton but_abort(36);
ezButton but_rtb(35);
ezButton but_send(15);
char raw_packet[40];
uint8_t int_raw_packet[40];
int A_val = 0;
float L_base_val = 28.123456;
float O_base_val = 77.123456;
int S_val = 5;
int F_val = 8;
int new_A_val;
float new_L_base_val;
float new_O_base_val;
int new_S_val;
int new_F_val;
float new_latitude;
float new_longitude;
String input = "";
bool sweepInputMode = false;
bool freqInputMode = false;
bool abortInputMode = false;
bool rtbInputMode = false;
bool latitudeInputMode = false;
bool longitudeInputMode = false;
int TXPacketL;
uint32_t TXPacketCount, startmS, endmS;
//receiving vars
float L_auv_val = 12.123456;
float O_auv_val = 66.123456;
int dvl_stat = 0;
int usbl_stat = 0;
int gps_stat = 0;
int pa_stat = 0;
int roll;
int pitch;
int yaw;
uint8_t uint8_packet[sizeof(raw_packet)];

uint32_t RXpacketCount;
uint32_t errors;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 60000; // 1 minute period

//pins
//not 21 22 27 5 18 26 14 19
byte rowPins[ROWS] = {4, 13, 16, 17};
byte colPins[COLS] = {23, 25, 32, 33}; //15 hata
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2x(U8G2_R0);
TinyGPS gps;
const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];

void getgps() {
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPSPort.available())
    {
      char c = GPSPort.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}
void getvals() {
  char *L_index = strchr(rxBuffer, 'L'); // LAT
  char *O_index = strchr(rxBuffer, 'O'); // LONG
  char *D_index = strchr(rxBuffer, 'D'); // DVL
  char *U_index = strchr(rxBuffer, 'U'); // USBL
  char *G_index = strchr(rxBuffer, 'G'); // GPS
  char *P_index = strchr(rxBuffer, 'P'); // PA
  char *R_index = strchr(rxBuffer, 'R'); // ROLL
  char *W_index = strchr(rxBuffer, 'W'); // PITCH
  char *Y_index = strchr(rxBuffer, 'Y'); // YAW
  char *E_index = strchr(rxBuffer, 'E'); // End
  if (L_index != NULL && O_index != NULL && D_index != NULL && U_index != NULL && G_index != NULL && P_index != NULL && R_index != NULL && W_index != NULL && Y_index != NULL && E_index != NULL)
  {
    String inlat = L_index + 1;
    L_auv_val = inlat.toFloat();
    String inlong = O_index + 1;
    O_auv_val = inlong.toFloat();
    dvl_stat = atoi(D_index + 1);
    usbl_stat = atoi(U_index + 1);
    gps_stat = atoi(G_index + 1);
    pa_stat = atoi(P_index + 1);
    roll = atoi(R_index + 1);
    pitch = atoi(W_index + 1);
    yaw = atoi(Y_index + 1);
    Serial.print(L_auv_val);
    Serial.print('_');
    Serial.print(O_auv_val);
    Serial.print('_');
    Serial.print(dvl_stat);
    Serial.print('_');
    Serial.print(usbl_stat);
    Serial.print('_');
    Serial.print(gps_stat);
    Serial.print('_');
    Serial.print(pa_stat);
    Serial.print('_');
    Serial.print(roll);
    Serial.print('_');
    Serial.print(pitch);
    Serial.print('_');
    Serial.print(yaw);
    Serial.println('_');

  }
}
void txpacket_is_OK()
{
  //if here packet has been sent OK
  uint16_t localCRC;

  Serial.print(F("  BytesSent,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  localCRC = LT.CRCCCITT(uint8_packet, TXPacketL, 0xFFFF);
  Serial.print(F("  CRC,"));
  Serial.print(localCRC, HEX);                         //print CRC of transmitted packet
  Serial.print(F("  TransmitTime,"));
  Serial.print(endmS - startmS);                       //print transmit time of packet
  Serial.print(F("mS"));
  Serial.print(F("  PacketsSent,"));
  Serial.print(TXPacketCount);                         //print total of packets sent OK
  Serial.println();
}
void txpacket_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                      //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                        //print IRQ status
  LT.printIrqStatus();                                 //prints the text of which IRQs set
  Serial.println();
}
void packet_is_OK()
{
  uint16_t IRQStatus, localCRC;

  IRQStatus = LT.readIrqStatus();                 //read the LoRa device IRQ status register

  RXpacketCount++;

  printElapsedTime();                             //print elapsed time to Serial Monitor
  Serial.print(F("  "));
  LT.printASCIIPacket(RXBUFFER, RXPacketL);       //print the packet as ASCII characters

  localCRC = LT.CRCCCITT(RXBUFFER, RXPacketL, 0xFFFF);  //calculate the CRC, this is the external CRC calculation of the RXBUFFER
  Serial.print(F(",CRC,"));                       //contents, not the LoRa device internal CRC
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

  printElapsedTime();                               //print elapsed time to Serial Monitor

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
void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}
void convertReceivedPacketToBuffer(uint8_t* RXPacket, uint8_t RXPacketL) {
  // Clear the buffer before storing the new packet
  memset(rxBuffer, 0, BUFFER_SIZE);
  bufferIndex = 0;

  // Copy the received packet into rxBuffer
  for (int i = 0; i < RXPacketL && bufferIndex < BUFFER_SIZE - 1; i++) {
    rxBuffer[bufferIndex++] = static_cast<char>(RXPacket[i]);
  }
  rxBuffer[bufferIndex] = '\0';
}
void updatePacket() {
  sprintf(raw_packet, "A%dL%.6fO%.6fS%dF%d",
          A_val, L_base_val, O_base_val, S_val, F_val);
}
void convertCharToUint8(const char* charArray, int length, uint8_t* uint8Array) {
  for (int i = 0; i < length; ++i) {
    uint8Array[i] = static_cast<uint8_t>(charArray[i]);
  }
}

void setup() {
  Serial.begin(115200);
  GPSPort.begin(9600, SERIAL_8N1, 2, 0);
  Serial.println("Remote_ESP_init!");
  SPI.begin(SCK, MISO, MOSI, NSS);
  u8g2x.setColorIndex(1);
  u8g2x.begin();
  u8g2x.setBitmapMode(1);
  but_abort.setDebounceTime(25);
  but_rtb.setDebounceTime(25);
  but_send.setDebounceTime(25);
  displayMainScreen(u8g2x);
  LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE);
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);
  LT.setMode(MODE_STDBY_RC);                              //got to standby mode to configure device
  LT.setPacketType(PACKET_TYPE_LORA);                     //set for LoRa transmissions
  LT.setRfFrequency(Frequency, Offset);                   //set the operating frequency
  LT.calibrateImage(0);                                   //run calibration after setting frequency
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);  //set LoRa modem parameters
  LT.setBufferBaseAddress(0x00, 0x00);                    //where in the SX buffer packets start, TX and RX
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);  //set packet parameters
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);              //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
  LT.setHighSensitivity();                                //set for highest sensitivity at expense of slightly higher LNA current
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);   //set for IRQ on RX done
}

void recevent() {
  if (RXPacketL = LT.receive(RXBUFFER, sizeof(RXBUFFER), 5000, WAIT_RX)) { //wait for a packet to arrive with 2 second (2000mS) timeout
    PacketRSSI = LT.readPacketRSSI();  //read the received RSSI value
    PacketSNR = LT.readPacketSNR();    //read the received SNR value
    if (RXPacketL == 0) {  //if the LT.receive() function detects an error, RXpacketL is 0
      packet_is_Error();
    } else {
      packet_is_OK();
      convertReceivedPacketToBuffer(RXBUFFER, RXPacketL);
      Serial.println(rxBuffer);
      getvals();
      displayStatus(u8g2x, S_val, F_val, dvl_stat, usbl_stat, gps_stat, pa_stat, L_auv_val, O_auv_val, roll, pitch, yaw);
    }
  }
}


void loop() {
  but_abort.loop();
  but_rtb.loop();
  but_send.loop();
  char key1 = keypad.getKey();
  Serial.print(key1);
  if (Serial.available() > 0) {
    char key = Serial.read();
    if (key) {
      if (key == 'A') {
        A_val = 0;
        sprintf(raw_packet, "A%dL%.6fO%.6fS%dF%dE",
                A_val, L_base_val, O_base_val, S_val, F_val);
        Serial.println(raw_packet);
        displayMainScreen(u8g2x);
        displayStatus(u8g2x, S_val, F_val, dvl_stat, usbl_stat, gps_stat, pa_stat, L_auv_val, O_auv_val, roll, pitch, yaw);
        //getpacketevent
      }
      else if (key == 'B') {
        A_val = 3;
        input = "";
        sweepInputMode = true;
        freqInputMode = false;
        abortInputMode = false;
        rtbInputMode = false;
        latitudeInputMode = false;
        longitudeInputMode = false;
        displayMainScreen(u8g2x);
        displaySweep(u8g2x);
      }
      else if (key == 'C') {
        A_val = 2;
        input = "";
        freqInputMode = true;
        sweepInputMode = false;
        abortInputMode = false;
        rtbInputMode = false;
        latitudeInputMode = false;
        longitudeInputMode = false;
        displayMainScreen(u8g2x);
        displayFreq(u8g2x);
      }
      else if (key == 'D') {
        A_val = 4;
        input = "";
        freqInputMode = false;
        sweepInputMode = false;
        abortInputMode = false;
        rtbInputMode = false;
        latitudeInputMode = true;
        longitudeInputMode = false;
        displayMainScreen(u8g2x);
        displayLatitude(u8g2x);
      }
      else if (key == 'S') {
        updatePacket();
        Serial.print(TXpower);                                       //print the transmit power defined
        Serial.print(F("dBm "));
        Serial.print(F("Packet> "));
        Serial.flush();
        TXPacketL = sizeof(raw_packet);
        raw_packet[TXPacketL - 1] = 'E';
        convertCharToUint8(raw_packet, TXPacketL, uint8_packet);
        LT.printASCIIPacket(uint8_packet, TXPacketL);
        startmS = millis();
        if (LT.transmit(uint8_packet, TXPacketL, 500, TXpower, WAIT_TX)) { //will return packet length sent if OK, otherwise 0 if transmit error
          endmS = millis();
          TXPacketCount++;
          txpacket_is_OK();
          recevent();
        }
        else {
          txpacket_is_Error();
        }
      }
      else if (key == 'Q'){ //abport
            A_val = 5;
    displayMainScreen(u8g2x);
    u8g2x.setCursor(0, 50);
    u8g2x.print("Confirm Abort!");
    freqInputMode = false;
    sweepInputMode = false;
    abortInputMode = true;
    rtbInputMode = false;
    u8g2x.sendBuffer();
        }
      else if (key == 'W'){
            A_val = 1;
    displayMainScreen(u8g2x);
    u8g2x.setCursor(0, 50);
    u8g2x.print("Confirm Return to Base!");
    freqInputMode = false;
    sweepInputMode = false;
    abortInputMode = false;
    rtbInputMode = true;
    getgps();
    u8g2x.sendBuffer();
        }
      else if (latitudeInputMode || longitudeInputMode) {
        if ((key >= '0' && key <= '9') || key == '.' || key == '-') {
          input += key;
          displayMainScreen(u8g2x);
          if (latitudeInputMode) {
            displayLatitude(u8g2x);
          } else {
            displayLongitude(u8g2x);
          }
          u8g2x.setCursor(0, 55);
          u8g2x.print(input);
          u8g2x.sendBuffer();
        }
      }
      else if (sweepInputMode) {
        if (key >= '0' && key <= '9') {
          input += key;
          displayMainScreen(u8g2x);
          displaySweep(u8g2x);
          u8g2x.setCursor(0, 55);
          u8g2x.print(input);
          u8g2x.sendBuffer();
        }
      }
      else if (freqInputMode) {
        if (key >= '0' && key <= '9') {
          input += key;
          displayMainScreen(u8g2x);
          displayFreq(u8g2x);
          u8g2x.setCursor(0, 55);
          u8g2x.print(input);
          u8g2x.sendBuffer();
        }
      }
    }
  }

if (but_send.isPressed()) {
    if (sweepInputMode) {
      new_S_val = input.toInt();
      S_val = new_S_val;
      A_val = 3;
      sweepInputMode = false;
      input = "";
      displayMainScreen(u8g2x);
      u8g2x.setCursor(0, 40);
      u8g2x.print("Sweep time updated");
      u8g2x.sendBuffer();
    }
    else if (freqInputMode) {
      new_F_val = input.toInt();
      F_val = new_F_val;
      A_val = 2;
      freqInputMode = false;
      input = "";
      displayMainScreen(u8g2x);
      u8g2x.setCursor(0, 40);
      u8g2x.print("Freq time updated");
      u8g2x.sendBuffer();
    }
    else if (latitudeInputMode) {
      A_val = 4;
      new_latitude = input.toFloat();
      L_base_val = new_latitude;
      updatePacket();
      displayMainScreen(u8g2x);
      latitudeInputMode = false;
      longitudeInputMode = true;
      input = "";
      displayMainScreen(u8g2x);
      displayLongitude(u8g2x);
    }
    else if (longitudeInputMode) {
      A_val = 4;
      new_longitude = input.toFloat();
      O_base_val = new_longitude;
      updatePacket();
      displayMainScreen(u8g2x);
      longitudeInputMode = false;
    }
    else if (abortInputMode) {
      A_val = 5;
      abortInputMode = false;
      input = "";
      displayMainScreen(u8g2x);
      u8g2x.sendBuffer();
    }
    else if (rtbInputMode) {
      A_val = 1;
      rtbInputMode = false;
      input = "";
      displayMainScreen(u8g2x);
      u8g2x.setCursor(0, 40);
      u8g2x.print("Returning to Base!");
      //update status event - return back to status page and show return operation
      u8g2x.sendBuffer();
    }

    else {
      updatePacket();

      Serial.print(TXpower);                                       //print the transmit power defined
      Serial.print(F("dBm "));
      Serial.print(F("Packet> "));
      PacketRSSI = LT.readPacketRSSI();              //read the recived RSSI value
      PacketSNR = LT.readPacketSNR();                //read the received SNR value
      Serial.print(PacketSNR);
      Serial.print(PacketRSSI);
      Serial.flush();
      TXPacketL = sizeof(raw_packet);
      raw_packet[TXPacketL - 1] = 'E';
      convertCharToUint8(raw_packet, TXPacketL, uint8_packet);
      LT.printASCIIPacket(uint8_packet, TXPacketL);
      startmS = millis();
      if (LT.transmit(uint8_packet, TXPacketL, 500, TXpower, WAIT_TX)) { //will return packet length sent if OK, otherwise 0 if transmit error
        endmS = millis();
        TXPacketCount++;
        txpacket_is_OK();
        recevent();
      }
      else {
        txpacket_is_Error();
      }
      displayMainScreen(u8g2x);
      u8g2x.sendBuffer();
  }
 }
}
