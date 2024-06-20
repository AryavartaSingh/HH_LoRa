//libraries
#include <Keypad.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ezButton.h>
#include "displayUtils.h"
#include <SPI.h>                                                                                       
#include <SX127XLT.h>                                          
#include "Settings.h"                                          
SX127XLT LT;                                                  
//definitions
#define ROWS 4
#define COLS 4
char keyMap[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'.','0','-','D'}
};
int bufferIndex = 0;
uint8_t RXPacketL;                              //used to store the received packet length
uint8_t RXBUFFER[100];                           //create the buffer that received packets are copied into
const byte disp_row[5] = {0, 15, 31, 47, 63};
ezButton but_abort(4);
ezButton but_rtb(35);
ezButton but_send(13);
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
//pins
uint8_t rowPins[ROWS] = {32,27,26,25};
uint8_t colPins[COLS] = {19,18,17,16};
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2x(U8G2_R0);

const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];

void getvals(){
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

  //***************************************************************************************************
  //Setup LoRa 
  //***************************************************************************************************
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
  //***************************************************************************************************
}

void loop() {
  but_abort.loop();
  but_rtb.loop();
  but_send.loop();
  //char key = keypad.getKey(); 
   if (Serial.available() > 0) {
    char key = Serial.read();
  
  if (key) {
    if (key == 'A') {
      sprintf(raw_packet, "A%dL%.6fO%.6fS%dF%dE",
              A_val, L_base_val, O_base_val, S_val, F_val);
      Serial.println(raw_packet);
      displayMainScreen(u8g2x);
      displayStatus(u8g2x, S_val, F_val, dvl_stat, usbl_stat, gps_stat, pa_stat, L_auv_val, O_auv_val, roll, pitch, yaw);  
      //getpacketevent
    }
    else if (key == 'B') {
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
        }}
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

  // Check button presses
  if (but_abort.isPressed()) {
    displayMainScreen(u8g2x);
    u8g2x.setCursor(0, 50);
    u8g2x.print("Confirm Abort!");
    freqInputMode = false;
    sweepInputMode = false;
    abortInputMode = true;
    rtbInputMode = false;
    u8g2x.sendBuffer();
  }

  if (but_rtb.isPressed()) {
    displayMainScreen(u8g2x);
    u8g2x.setCursor(0, 50);
    u8g2x.print("Confirm Return to Base!");
    freqInputMode = false;
    sweepInputMode = false;
    abortInputMode = false;
    rtbInputMode = true;
    u8g2x.sendBuffer();
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
      displayStatus(u8g2x, S_val, F_val, dvl_stat, usbl_stat, gps_stat, pa_stat, L_auv_val, O_auv_val, roll, pitch, yaw);
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
      displayStatus(u8g2x, S_val, F_val, dvl_stat, usbl_stat, gps_stat, pa_stat, L_auv_val, O_auv_val, roll, pitch, yaw);
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
       Serial.flush();
      TXPacketL = sizeof(raw_packet);
      raw_packet[TXPacketL - 1] = 'E';
      convertCharToUint8(raw_packet, TXPacketL, uint8_packet);
      LT.printASCIIPacket(uint8_packet, TXPacketL);
      if (!LT.transmit(uint8_packet, TXPacketL, 5000, TXpower, WAIT_TX)){ //will return packet length sent if OK, otherwise 0 if transmit error
      Serial.print(F("Transmit failed"));
      LT.printIrqStatus();                                          //print IRQ status, indicates why packet transmit fail
      Serial.println();
      }
        if (LT.receive(RXBUFFER, sizeof(RXBUFFER), 2000, WAIT_RX))      //wait for a packet to arrive with 2 second (2000mS) timeout
  {
    RXPacketL = LT.readRegister(REG_RXNBBYTES);
    convertReceivedPacketToBuffer(RXBUFFER, RXPacketL);
    Serial.println(rxBuffer);
    getvals();
    //LT.printASCIIPacket(RXBUFFER, RXPacketL);                     //print the packet as ASCII characters
    Serial.println();
  }
  else
  {
    Serial.print(F("Receive failed"));
    LT.printIrqStatus();                                          //print IRQ status, why did packet receive fail
    Serial.println();
  }
     // receiveLoRaPacket();
      displayMainScreen(u8g2x);
     // u8g2x.setCursor(0, 50);
     // u8g2x.print("Packet Sent");
     // delay(1000);
      displayStatus(u8g2x, S_val, F_val, dvl_stat, usbl_stat, gps_stat, pa_stat, L_auv_val, O_auv_val, roll, pitch, yaw);
      u8g2x.sendBuffer();
    }
   }
}
