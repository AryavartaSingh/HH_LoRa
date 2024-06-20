#include <U8g2lib.h>

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

void displayMainScreen(U8G2 &u8g2x) {
  u8g2x.clearBuffer();
  u8g2x.drawXBMP(44, -10, logo_width, logo_height, logo_bits);
  u8g2x.setFont(u8g2_font_profont10_tf );
  u8g2x.sendBuffer();
}

void displayStatus(U8G2 &u8g2x, int S_val, int F_val, int dvl_stat, int usbl_stat, int gps_stat, int pa_stat, float L_auv_val, float O_auv_val, int roll, int pitch, int yaw) {
  u8g2x.setFont(u8g2_font_profont10_tf );
  u8g2x.setCursor(0, 6);
  u8g2x.printf("Sweep:%d", S_val);
  u8g2x.setCursor(0, 14);
  u8g2x.printf("Freq:%d", F_val);
  u8g2x.setCursor(96, 6);
  u8g2x.printf("DVL:%d", dvl_stat);
  u8g2x.setCursor(96, 14);
  u8g2x.printf("USBL:%d", usbl_stat);
  u8g2x.setCursor(96, 22);
  u8g2x.printf("GPS:%d", gps_stat);
  u8g2x.setCursor(96, 30);
  u8g2x.printf("PA:%d", pa_stat);
  u8g2x.setCursor(0, 50);
  u8g2x.printf("Lat: %f", L_auv_val);
  u8g2x.setCursor(0, 57);
  u8g2x.printf("Long: %f", O_auv_val);
  u8g2x.setCursor(0, 64);
  u8g2x.printf("R:%d P:%d Y:%d", roll, pitch, yaw);
  u8g2x.sendBuffer();
}
void displayLoRaDetected(U8G2 &u8g2x) {
  u8g2x.setFont(u8g2_font_helvR08_tr);
  u8g2x.setCursor(0, 40);
  u8g2x.printf("LoRa Detected");
  u8g2x.sendBuffer();
}
void displayRXTimeout(U8G2 &u8g2x) {
  u8g2x.setFont(u8g2_font_helvR08_tr);
  u8g2x.setCursor(0, 40);
  u8g2x.printf("Timeout");
  u8g2x.sendBuffer();
}

void displayPacketError(U8G2 &u8g2x) {
  u8g2x.setFont(u8g2_font_helvR08_tr);
  u8g2x.setCursor(0, 40);
  u8g2x.printf("Packet Error");
  u8g2x.sendBuffer();
}
void displayPacketFine(U8G2 &u8g2x) {
  u8g2x.setFont(u8g2_font_helvR08_tr);
  u8g2x.setCursor(0, 40);
  u8g2x.printf("Packet Fine");
  u8g2x.sendBuffer();
}


void displayLoRaNotDetected(U8G2 &u8g2x) {
  u8g2x.setFont(u8g2_font_helvR08_tr);
  u8g2x.setCursor(0, 40);
  u8g2x.printf("LoRaNotFound");
  u8g2x.sendBuffer();
}

void displayLatitude(U8G2 &u8g2x) {
  u8g2x.setFont(u8g2_font_helvR08_tr);
  u8g2x.setCursor(0, 40);
  u8g2x.printf("Enter new Latitude:");
  u8g2x.sendBuffer();
}

void displayLongitude(U8G2 &u8g2x) {
  u8g2x.setFont(u8g2_font_helvR08_tr);
  u8g2x.setCursor(0, 40);
  u8g2x.printf("Enter new Longitude:");
  u8g2x.sendBuffer();
}