//libraries
#include <Keypad.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ezButton.h>
//definitions
#define ROWS 4
#define COLS 4
char keyMap[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'.','0','-','D'}
};
const byte disp_row[5] = {0, 15, 31, 47, 63};
ezButton but_abort(4);
ezButton but_rtb(35);
ezButton but_send(13);
char raw_packet[30];
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

float new_latitude;
float new_longitude;


//pins
uint8_t rowPins[ROWS] = {32,27,26,25};
uint8_t colPins[COLS] = {19,18,17,16};
Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0); //constructor for displa

//bmp
#define logo_width 40
#define logo_height 40
const unsigned char PROGMEM logo_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x81, 
  0x01, 0x00, 0x00, 0xF0, 0x81, 0x0F, 0x00, 0x00, 0xF0, 0x00, 0x0F, 0x00, 
  0x00, 0xF8, 0xFF, 0x1F, 0x00, 0x00, 0xF8, 0xFF, 0x1F, 0x00, 0x00, 0xFC, 
  0xFF, 0x3F, 0x00, 0x00, 0xFC, 0xFF, 0x3F, 0x00, 0x00, 0x3E, 0x00, 0x7C, 
  0x00, 0x00, 0x1F, 0x00, 0xF8, 0x00, 0x00, 0x0F, 0x00, 0xF0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 
  0xF0, 0x00, 0x00, 0x1F, 0x00, 0xF8, 0x00, 0x00, 0x3E, 0x00, 0x7C, 0x00, 
  0x00, 0xFC, 0xFF, 0x3F, 0x00, 0x00, 0xFC, 0xFF, 0x3F, 0x00, 0x00, 0xF8, 
  0xFF, 0x1F, 0x00, 0x00, 0xF8, 0xFF, 0x1F, 0x00, 0x00, 0xF0, 0x00, 0x0F, 
  0x00, 0x00, 0xF0, 0x81, 0x0F, 0x00, 0x00, 0x80, 0x81, 0x01, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

void updatePacket() {
  sprintf(raw_packet, "A%dL%.6fO%.6fS%dF%dE",
          A_val, L_base_val, O_base_val, S_val, F_val);
}

void displayMainScreen() {
  if (A_val == 5){
  u8g2.clearBuffer();
  u8g2.drawXBMP(44, -10, logo_width, logo_height, logo_bits);
  u8g2.setFont(u8g2_font_profont10_tf );
  u8g2.setCursor (20, 30);
  u8g2.printf("Mission Aborted");
  u8g2.sendBuffer();
    }
  else{
  u8g2.clearBuffer();
  u8g2.drawXBMP(44, -10, logo_width, logo_height, logo_bits);
  u8g2.setFont(u8g2_font_profont10_tf );
  u8g2.sendBuffer();
}}

void displayStatus() {
  u8g2.setFont(u8g2_font_profont10_tf );
  u8g2.setCursor(0, 6);
  u8g2.printf("Sweep:%d", S_val);
  u8g2.setCursor(0, 14);
  u8g2.printf("Freq:%d", F_val);
  u8g2.setCursor(96, 6);
  u8g2.printf("DVL:%d", dvl_stat);
  u8g2.setCursor(96, 14);
  u8g2.printf("USBL:%d", usbl_stat);
  u8g2.setCursor(96, 22);
  u8g2.printf("GPS:%d", gps_stat);
  u8g2.setCursor(96, 30);
  u8g2.printf("PA:%d", pa_stat);
  u8g2.setCursor(0, 50);
  u8g2.printf("Lat: %f", L_auv_val);
  u8g2.setCursor(0, 57);
  u8g2.printf("Long: %f", O_auv_val);
  u8g2.setCursor(0, 64);
  u8g2.printf("R:%d P:%d Y:%d", roll, pitch, yaw);
  u8g2.sendBuffer();
  //add functionality for last time at which data was received
}

void displaySweep() {
  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.setCursor(0, 40);
  u8g2.printf("Enter new Sweep time:");
  u8g2.sendBuffer();
}

void displayFreq() {
  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.setCursor(0, 40);
  u8g2.printf("Enter new Resurf. freq:");
  u8g2.sendBuffer();
}
void displayLatitude() {
  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.setCursor(0, 40);
  u8g2.printf("Enter new Latitude:");
  u8g2.sendBuffer();
}

void displayLongitude() {
  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.setCursor(0, 40);
  u8g2.printf("Enter new Longitude:");
  u8g2.sendBuffer();
}
String input = "";
bool sweepInputMode = false;
bool freqInputMode = false;
bool abortInputMode = false;
bool rtbInputMode = false;
bool latitudeInputMode = false;
bool longitudeInputMode = false;
void setup() {
  Serial.begin(115200);
  Serial.println("Remote_ESP_init!");
  u8g2.setColorIndex(1);
  u8g2.begin();
  u8g2.setBitmapMode(1);
  but_abort.setDebounceTime(25);
  but_rtb.setDebounceTime(25);
  but_send.setDebounceTime(25);
  displayMainScreen();
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
      displayMainScreen();
      displayStatus();
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
      displayMainScreen();
      displaySweep();
    }
    else if (key == 'C') {
      input = "";
      freqInputMode = true;
      sweepInputMode = false;
      abortInputMode = false;
      rtbInputMode = false;
      latitudeInputMode = false;
      longitudeInputMode = false;
      displayMainScreen();
      displayFreq();
    }
    else if (key == 'D') {
        input = "";
        freqInputMode = false;
        sweepInputMode = false;
        abortInputMode = false;
        rtbInputMode = false;
        latitudeInputMode = true;
        longitudeInputMode = false;
        displayMainScreen();
        displayLatitude();
    }
    else if (latitudeInputMode || longitudeInputMode) {
        if ((key >= '0' && key <= '9') || key == '.' || key == '-') {
          input += key;
          displayMainScreen();
          if (latitudeInputMode) {
            displayLatitude();
          } else {
            displayLongitude();
          }
          u8g2.setCursor(0, 55);
          u8g2.print(input);
          u8g2.sendBuffer();
        }}
    else if (sweepInputMode) {
      if (key >= '0' && key <= '9') {
        input += key;
        displayMainScreen();
        displaySweep();
        u8g2.setCursor(0, 55);
        u8g2.print(input);
        u8g2.sendBuffer();
      }
    }
    else if (freqInputMode) {
      if (key >= '0' && key <= '9') {
        input += key;
        displayMainScreen();
        displayFreq();
        u8g2.setCursor(0, 55);
        u8g2.print(input);
        u8g2.sendBuffer();
      }
    }
  }
  }

  // Check button presses
  if (but_abort.isPressed()) {
    displayMainScreen();
    u8g2.setCursor(0, 50);
    u8g2.print("Confirm Abort!");
    freqInputMode = false;
    sweepInputMode = false;
    abortInputMode = true;
    rtbInputMode = false;
    u8g2.sendBuffer();
  }

  if (but_rtb.isPressed()) {
    displayMainScreen();
    u8g2.setCursor(0, 50);
    u8g2.print("Confirm Return to Base!");
    freqInputMode = false;
    sweepInputMode = false;
    abortInputMode = false;
    rtbInputMode = true;
    u8g2.sendBuffer();
  }

  if (but_send.isPressed()) {
    if (sweepInputMode) {
      new_S_val = input.toInt();
      S_val = new_S_val;
      A_val = 3;
      sweepInputMode = false;
      input = "";
      displayMainScreen();
      u8g2.setCursor(0, 40);
      u8g2.print("Sweep time updated");
      u8g2.sendBuffer();
    } 
    else if (freqInputMode) {
      new_F_val = input.toInt();
      F_val = new_F_val;
      A_val = 2;
      freqInputMode = false;
      input = "";
      displayMainScreen();
      u8g2.setCursor(0, 40);
      u8g2.print("Freq time updated");
      u8g2.sendBuffer();
    }
     else if (latitudeInputMode) {
      A_val = 4;
      new_latitude = input.toFloat();
      L_base_val = new_latitude;
      updatePacket();
      displayMainScreen();
      displayStatus();
      latitudeInputMode = false;
      longitudeInputMode = true;
      input = "";
      displayMainScreen();
      displayLongitude();
    }
    else if (longitudeInputMode) {
      A_val = 4;
      new_longitude = input.toFloat();
      O_base_val = new_longitude;
      updatePacket();
      displayMainScreen();
      displayStatus();
      longitudeInputMode = false;
    }
    else if (abortInputMode) {
      A_val = 5;
      abortInputMode = false;
      input = "";
      displayMainScreen();
      u8g2.sendBuffer();
    }
    else if (rtbInputMode) {
      A_val = 1;
      rtbInputMode = false;
      input = "";
      displayMainScreen();
      u8g2.setCursor(0, 40);
      u8g2.print("Returning to Base!");
      //update status event - return back to status page and show return operation
      u8g2.sendBuffer();
    }
    
    else {
      updatePacket();
      displayMainScreen();
      u8g2.setCursor(0, 50);
      u8g2.print("Packet Sent");
      u8g2.sendBuffer();
    }}

  //delay(10);
}
