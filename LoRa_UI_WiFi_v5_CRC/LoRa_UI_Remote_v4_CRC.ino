#include <U8g2lib.h>
#include <Wire.h>
#include "displayUtils.h"
#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
#include <TinyGPS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

unsigned long previousMillis = 0;
String inlong;
String inlat;
const char* ssid = "Redminib";
const char* password = "aryavarta";
String webpage = "<!DOCTYPE html><html><head> <title>HyperHorizon</title> <style> body { background-color: #000000; color: #ffffff; font-family: Arial, sans-serif; margin: 0; padding: 0; } #content { display: flex; flex-direction: row; justify-content: space-between; } #main { flex: 3; padding: 20px; } #sidebar { flex: 1; padding: 20px; background-color: #1a1a1a; border-left: 2px solid #333; } h1 { color: #00ff00; text-align: center; } .status-indicator { display: flex; align-items: center; margin-bottom: 10px; } .status-bubble { width: 15px; height: 15px; border-radius: 50%; margin-right: 10px; } .status-green { background-color: #00ff00; } .status-red { background-color: #ff0000; } .input-group { display: flex; align-items: center; margin-bottom: 10px; } .input-group input, .input-group button { margin-right: 5px; } button { background-color: #333; color: #fff; border: 1px solid #555; padding: 5px 10px; cursor: pointer; } button:hover { background-color: #444; } </style></head><body> <div id='content'> <div id='main'> <h1>HyperHorizon</h1> <p>Sweep Time: <span id='freq'>-</span></p> <p>Resurfacing Frequency: <span id='sweep'>-</span></p> <p>Received Latitude: <span id='rec_lat'>-</span></p> <p>Received Longitude: <span id='rec_long'>-</span></p> <p>Roll: <span id='roll'>-</span></p> <p>Pitch: <span id='pitch'>-</span></p> <p>Yaw: <span id='yaw'>-</span></p> <p>Base Station Latitude: <span id='base_lat'>-</span></p> <p>Base Station Longitude: <span id='base_long'>-</span></p> <p>A_val: <span id='A_val'>-</span></p> <p><button type='button' id='BTN_SEND_BACK'>Send info to ESP32</button></p> <p><button type='button' id='BTN_SEND_PACKET'>Send Packet</button></p> <div class='input-group'> <button type='button' id='BTN_SET_FREQ'>Set Frequency</button> <input type='text' id='freq_input' style='display: none;' placeholder='Enter Frequency' /> <button type='button' id='BTN_SEND_FREQ' style='display: none;'>Send Frequency</button> </div> <div class='input-group'> <button type='button' id='BTN_SET_SWEEP'>Set Sweep Time</button> <input type='text' id='sweep_input' style='display: none;' placeholder='Enter Sweep Time' /> <button type='button' id='BTN_SEND_SWEEP' style='display: none;'>Send Sweep Time</button> </div> <p><button type='button' id='BTN_RTB'>Return to Base</button> <button type='button' id='BTN_CONFIRM_RTB' style='display: none;'>Confirm Return to Base</button></p> <p><button type='button' id='BTN_ABORT'>Abort Mission</button> <button type='button' id='BTN_CONFIRM_ABORT' style='display: none;'>Confirm Abort Mission</button></p> <div class='input-group'> <button type='button' id='BTN_ADD_WAYPOINT'>Add Waypoint</button> <input type='text' id='waypoint_lat' style='display: none;' placeholder='Enter Waypoint Latitude' /> <input type='text' id='waypoint_long' style='display: none;' placeholder='Enter Waypoint Longitude' /> <button type='button' id='BTN_SEND_WAYPOINT' style='display: none;'>Send Waypoint</button> </div> </div> <div id='sidebar'> <h2>Status Indicators</h2> <div class='status-indicator'> <div id='dvl_bubble' class='status-bubble'></div> <p>DVL Status: <span id='dvl_stat'>-</span></p> </div> <div class='status-indicator'> <div id='USBL_bubble' class='status-bubble'></div> <p>USBL Status: <span id='USBL_stat'>-</span></p> </div> <div class='status-indicator'> <div id='gps_bubble' class='status-bubble'></div> <p>GPS Status: <span id='gps_stat'>-</span></p> </div> <div class='status-indicator'> <div id='pa_bubble' class='status-bubble'></div> <p>PA Status: <span id='pa_stat'>-</span></p> </div> </div> </div> <script> var Socket; document.getElementById('BTN_SEND_BACK').addEventListener('click', button_send_back); document.getElementById('BTN_SEND_PACKET').addEventListener('click', button_send_packet); document.getElementById('BTN_SET_FREQ').addEventListener('click', button_show_freq_input); document.getElementById('BTN_SEND_FREQ').addEventListener('click', button_send_freq); document.getElementById('BTN_SET_SWEEP').addEventListener('click', button_show_sweep_input); document.getElementById('BTN_SEND_SWEEP').addEventListener('click', button_send_sweep); document.getElementById('BTN_RTB').addEventListener('click', button_show_confirm_rtb); document.getElementById('BTN_CONFIRM_RTB').addEventListener('click', button_confirm_rtb); document.getElementById('BTN_ABORT').addEventListener('click', button_show_confirm_abort); document.getElementById('BTN_CONFIRM_ABORT').addEventListener('click', button_confirm_abort); document.getElementById('BTN_ADD_WAYPOINT').addEventListener('click', button_show_waypoint_input); document.getElementById('BTN_SEND_WAYPOINT').addEventListener('click', button_send_waypoint); function init() { Socket = new WebSocket('ws://' + window.location.hostname + ':81/'); Socket.onmessage = function (event) { processCommand(event); }; } function button_send_back() { var msg = { brand: 'HyperHorizon', type: 'AUV', year: 2024, color: 'white' }; Socket.send(JSON.stringify(msg)); } function button_send_packet() { Socket.send(JSON.stringify({ A_val: 0 })); } function button_show_freq_input() { document.getElementById('freq_input').style.display = 'inline'; document.getElementById('BTN_SEND_FREQ').style.display = 'inline'; } function button_send_freq() { var freq_value = document.getElementById('freq_input').value; Socket.send(JSON.stringify({ freq: freq_value, A_val: 2 })); } function button_show_sweep_input() { document.getElementById('sweep_input').style.display = 'inline'; document.getElementById('BTN_SEND_SWEEP').style.display = 'inline'; } function button_send_sweep() { var sweep_value = document.getElementById('sweep_input').value; Socket.send(JSON.stringify({ sweep: sweep_value, A_val: 3 })); } function button_show_confirm_rtb() { document.getElementById('BTN_CONFIRM_RTB').style.display = 'inline'; } function button_confirm_rtb() { Socket.send(JSON.stringify({ A_val: 1 })); } function button_show_confirm_abort() { document.getElementById('BTN_CONFIRM_ABORT').style.display = 'inline'; } function button_confirm_abort() { Socket.send(JSON.stringify({ A_val: 5 })); } function button_show_waypoint_input() { document.getElementById('waypoint_lat').style.display = 'inline'; document.getElementById('waypoint_long').style.display = 'inline'; document.getElementById('BTN_SEND_WAYPOINT').style.display = 'inline'; } function button_send_waypoint() { var waypoint_lat = parseFloat(document.getElementById('waypoint_lat').value).toFixed(10); var waypoint_long = parseFloat(document.getElementById('waypoint_long').value).toFixed(10); Socket.send(JSON.stringify({ waypoint_lat: waypoint_lat, waypoint_long: waypoint_long, A: 4, A_val: 4 })); } function processCommand(event) { var obj = JSON.parse(event.data); document.getElementById('freq').innerHTML = obj.freq; document.getElementById('sweep').innerHTML = obj.sweep; document.getElementById('rec_lat').innerHTML = obj.rec_lat; document.getElementById('rec_long').innerHTML = obj.rec_long; document.getElementById('dvl_stat').innerHTML = obj.dvl_stat; document.getElementById('USBL_stat').innerHTML = obj.USBL_stat; document.getElementById('gps_stat').innerHTML = obj.gps_stat; document.getElementById('pa_stat').innerHTML = obj.pa_stat; document.getElementById('roll').innerHTML = obj.roll; document.getElementById('pitch').innerHTML = obj.pitch; document.getElementById('yaw').innerHTML = obj.yaw; document.getElementById('base_lat').innerHTML = obj.base_lat; document.getElementById('base_long').innerHTML = obj.base_long; document.getElementById('A_val').innerHTML = obj.A_val; updateStatusIndicator('dvl_bubble', obj.dvl_stat); updateStatusIndicator('USBL_bubble', obj.USBL_stat); updateStatusIndicator('gps_bubble', obj.gps_stat); updateStatusIndicator('pa_bubble', obj.pa_stat); } function updateStatusIndicator(elementId, status) { var element = document.getElementById(elementId); if (status === '1') { element.className = 'status-bubble status-green'; } else { element.className = 'status-bubble status-red'; } } window.onload = function (event) { init(); }; </script></body></html>";
String freq = "0";
String sweep = "0";
int interval = 1000;
String base_lat = "12.1234567890";
String base_long = "-112.1234567890";
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
SX127XLT LT;
void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);
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
    inlat = L_index + 1;
    L_auv_val = inlat.toFloat();
    inlong = O_index + 1;
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
  WiFi.begin(ssid, password);
  Serial.println("Remote_ESP_init!");
  Serial.println("Establishing connection to WiFi with SSID: " + String(ssid));
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.print("Connected to network with IP address: ");
    Serial.println(WiFi.localIP());

    server.on("/", []() {
        server.send(200, "text/html", webpage);
    });
    server.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
  SPI.begin(SCK, MISO, MOSI, NSS);
  u8g2x.setColorIndex(1);
  u8g2x.begin();
  u8g2x.setBitmapMode(1);
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
  GPSPort.begin(9600, SERIAL_8N1, 2, 0);
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

void sendevent(){
  updatePacket();
        Serial.print(TXpower);                                       
      Serial.print(F("dBm "));
      Serial.print(F("Packet> "));
      PacketRSSI = LT.readPacketRSSI();             
      PacketSNR = LT.readPacketSNR();               
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
  }

void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("Client " + String(num) + " disconnected");
            break;
        case WStype_CONNECTED:
            Serial.println("Client " + String(num) + " connected");
            break;
        case WStype_TEXT: {
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, payload, length);
            if (error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
                return;
            }

            if (doc.containsKey("freq")) {
                F_val = doc["freq"];
                freq = String(F_val);
                A_val = 2;
                Serial.println("Received frequency: " + freq);
                sendevent();
            } else if (doc.containsKey("sweep")) {
                S_val = doc["sweep"];
                sweep = String(S_val);
                A_val = 3;
                Serial.println("Received sweep time: " + sweep);
                sendevent();
            } else if (doc.containsKey("waypoint_lat") && doc.containsKey("waypoint_long")) {
                L_base_val = doc["waypoint_lat"];
                O_base_val = doc["waypoint_long"];
                int A = doc["A"];
                A_val = 4;
                sendevent();
                Serial.println("Received waypoint: Lat " + String(L_base_val, 10) + ", Long " + String(O_base_val, 10) + ", A: " + String(A));
            } else if (doc.containsKey("A_val")) {
                A_val = doc["A_val"];
                Serial.println("Received A_val: " + String(A_val));
                sendevent();
            } else {
                String msg = String((char*) payload);
                Serial.println("Message from user: " + msg);
                if (msg == "RTB") {
                    A_val = 1;
                    Serial.println("Received Return to Base command");
                    sendevent();
                } else if (msg == "ABORT") {
                    A_val = 5;
                    Serial.println("Received Abort Mission command");
                    sendevent();
                }
            }
            break;
        }
        default:
            break;
    }
}
void loop() {
  server.handleClient();
  webSocket.loop();
unsigned long now = millis();
    if ((unsigned long)(now - previousMillis) > interval) {
        String jsonString = "";
        StaticJsonDocument<400> doc;
        JsonObject object = doc.to<JsonObject>();
        object["freq"] = S_val;
        object["sweep"] = F_val;
        object["rec_lat"] = O_auv_val;
        object["rec_long"] = L_auv_val;
        object["dvl_stat"] = dvl_stat; 
        object["USBL_stat"] = usbl_stat; 
        object["gps_stat"] = gps_stat; 
        object["pa_stat"] = pa_stat; 
        object["roll"] = roll;
        object["pitch"] = pitch; 
        object["yaw"] = yaw; 
        object["base_lat"] = base_lat;
        object["base_long"] = base_long;
        object["A_val"] = A_val;
        serializeJson(doc, jsonString);
        Serial.println(jsonString);
        webSocket.broadcastTXT(jsonString);
        
        previousMillis = now;
    }
  //currentMillis = millis();
//
//  if (currentMillis - startMillis >= period) {
//    startMillis = currentMillis;
//    getgps();
//    
//  }

  
}
