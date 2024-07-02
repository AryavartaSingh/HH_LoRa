//#include <SoftwareSerial.h>
#include "uWAVE_ALib.h"

/*  
 * For compiling this sketch uses uWAVE_ALib library, that can be found on github:
 * https://github.com/ucnl/uWAVE_ALib
 *   
 * Hardware list:
 *  
 *    1. Arduino UNO/Nano compatible board - 1 pcs
 *    2. uWAVE modem - at least 1 pcs
 *      
 *  In this sketch the hardware serial (Serial) is used to communicate with a PC. Both Arduino and uWave modem are powered from PC's usb port.
 *  
 *  uWAVE's cable cores should be soldered to the Arduino as follows:
 *  
 *  uWAVE Core     | Arduino Nano Pin
 *  *********************************           
 *  GND            | GND
 *  VCC            | VIN
 *  TX             | D2
 *  RX             | D3
 *  CMD            | D4
 *   
 *   
 *  After power up, scetch will attempt to send different queries to the modem:
 *  - ask the modem for device information
 *  - try to change Ambient data settings
 *  - query for packet mode settings
 *  - sending periodic remote requests
 *   
 * For more information about uWave interfacing protocol, please refer: 
 * https://docs.unavlab.com/documentation/EN/uWAVE/uWAVE_Protocol_Specification_en.html
 */


// Configuration of this sketch ------------------------------------------------------------------------------------
#define SPEED_OF_SOUND     (1500)

#define USE_PKT_MODE

#define REMOTE_MODEM_TX_ID (0)
#define REMOTE_MODEM_RX_ID (1)
#define REMOTE_PKT_ADDR    (0)
#define REMOTE_PKT_DID     (DID_DPT)

#define REMOTE_REQUEST_PERIOD_MS (5000)

#define AMB_IS_PRESSURE    (true)
#define AMB_IS_TEMPERATURE (true)
#define AMB_IS_DEPTH       (true)
#define AMB_IS_VOLTAGE     (true)
#define AMB_PERIOD_MS      (1) // 0 - do not send, 1 - after any modem output, other value - period in milliseconds
//

// Configuration of modem's connection ----------------------------------------------------------------------------
//#define uWAVE_RX_PIN  (3)
//#define uWAVE_TX_PIN  (2)
#define uWAVE_CMD_PIN (4)

// Variables section ----------------------------------------------------------------------------------------------
byte pkt[uWAVE_PKT_MAX_SIZE];
byte pkt_size;

long rem_req_ts = 0;


HardwareSerial uWavePort(1);

uWAVE uWrapper(&uWavePort, uWAVE_CMD_PIN);
uWAVE_EVENT_Enum result;

void setup() 
{  
  uWavePort.begin(9600, SERIAL_8N1, 2, 0); // TX RX
  Serial.begin(9600);    // Start hardware serial

  uWrapper.enable();     // set the CMD pin to a High level (Command mode on)
}

void loop() {
  result = uWrapper.process(); // This function should be called as frequent as possible

  if (result != uWAVE_NONE)    // If something is happend in the uWrapper...
  {
    if (result & uWAVE_ACK_RECEIVED) // Aan ACK received from the modem
    {
      Serial.println(F("[ACK]"));
      
      Serial.print(F("sntID="));
      Serial.println((char)uWrapper.getLast_request_sntID());  // ID of the sentence that caused the ACK
      Serial.print(F("result="));
      Serial.println(uWrapper.getACK_result());                // Result
    }   
    else if (result & uWAVE_REMOTE_RESPONSE_RECEIVED) // 
    {
      Serial.println(F("[REMOTE RESPONSE]"));
      
      Serial.print(F("requestID="));
      Serial.println(uWrapper.getRem_requested_rcID());
      Serial.print(F("requested txID="));
      Serial.println(uWrapper.getRem_requested_txID());
      Serial.print(F("requested rxID="));
      Serial.println(uWrapper.getRem_requested_rxID());
      Serial.print(F("propagation time, s="));
      Serial.println(uWrapper.getRem_propTime_s());
      Serial.print(F("Slant range, m="));
      Serial.println(uWrapper.getRem_propTime_s() * SPEED_OF_SOUND);
      Serial.print(F("MSR, dB="));
      Serial.println(uWrapper.getRem_msr_dB());
      Serial.print(F("value="));
      Serial.println(uWrapper.getRem_value());

      float f = uWrapper.getRem_azimuth_deg();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
        Serial.print("Azimuth, °=");
        Serial.println(f);
      }
    }    
    else if (result & uWAVE_DEVICE_INFO_RECEIVED)
    {      
      Serial.println(F("[DINFO]"));
      Serial.print(F("S/N="));
      Serial.write(uWrapper._serial, 24);
      Serial.print(F("\r\nCore moniker="));
      Serial.write(uWrapper._core_moniker, 32);
      Serial.print(F("\r\nCore version="));
      Serial.print(uWrapper._core_version >> 0x08);
      Serial.print(F("."));
      Serial.println(uWrapper._core_version & 0xff);
      Serial.print(F("System moniker="));
      Serial.write(uWrapper._system_moniker, 32);
      Serial.print(F("\r\nSystem version="));
      Serial.print(uWrapper._system_version >> 0x08);
      Serial.print(F("."));
      Serial.println(uWrapper._system_version & 0xff);
      Serial.print(F("Acoustic baudrate, bit/s="));
      Serial.println(uWrapper.getAcousticBaudrate(), 2);
      Serial.print(F("Tx channel ID="));
      Serial.println(uWrapper.getTxChID());
      Serial.print(F("Rx channel ID="));
      Serial.println(uWrapper.getRxChID());
      Serial.print(F("Total channels="));
      Serial.println(uWrapper.getTotalCodeChannels());
      Serial.print(F("Salinity, PSU="));
      Serial.println(uWrapper.getSalinityPSU(), 1);
      Serial.print(F("PTS present="));
      Serial.println(uWrapper.isPTSPresent());
      Serial.print(F("Command mode by default="));
      Serial.println(uWrapper.isCmdModeByDefault());
    }
    else if (result & uWAVE_ASYNC_IN_RECEIVED)
    {
      Serial.println("[ASYNC IN]");

      Serial.print("rem_async_in=");
      Serial.println(uWrapper.getRem_async_in_rc());
      Serial.print("rem_async_msr, dB=");
      Serial.print(uWrapper.getRem_msr_dB());

      float f = uWrapper.getRem_azimuth_deg();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
        Serial.print("Azimuth, °=");
        Serial.println(f);
      }
    }
    else if (result & uWAVE_AMB_DATA_RECEIVED)
    {
      Serial.println("[AMB DATA]");

      float f;
      
      f = uWrapper.getAmb_pressure_mBar();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
         Serial.print("Pressure, mBar=");
         Serial.println(f);
      }

      f = uWrapper.getAmb_temperature_C();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
         Serial.print("Temperature, °C=");
         Serial.println(f);
      }

      f = uWrapper.getAmb_depth_m();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
         Serial.print("Depth, m=");
         Serial.println(f);
      }

      f = uWrapper.getAmb_voltage_V();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
         Serial.print("Voltage, V=");
         Serial.println(f);
      }
    }
    else if (result & uWAVE_PKT_SETTINGS_RECEIVED)
    {
      Serial.println(F("[PACKET MODE SETTINGS]"));

      Serial.print(F("isPktMode="));
      Serial.println((bool)uWrapper.isPktMode());
      Serial.print(F("pktModeLocalAddress="));
      Serial.println(uWrapper.pktModeLocalAddress());      
    }
    else if (result & uWAVE_PKT_DELIVERED)
    {
      Serial.println(F("[PACKET DELIVERED]"));

      Serial.print(F("pkt_tries_taken="));
      Serial.println(uWrapper.getPkt_tries_taken());
      Serial.print(F("pkt_target_address="));
      Serial.println(uWrapper.getPkt_target_address());

      float f = uWrapper.getRem_azimuth_deg();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
        Serial.print("Azimuth, °=");
        Serial.println(f);
      }

      Serial.print(F("Packet: "));
      uWrapper.getPkt_delivered(pkt, &pkt_size);
      for (int i = 0; i < pkt_size; i++)
      {
        Serial.print(DIGIT_2HEX(pkt[i] / 16));
        Serial.print(DIGIT_2HEX(pkt[i] % 16));
      }
      Serial.println();
    }
    else if (result & uWAVE_PKT_RECEIVED)
    {
      Serial.println(F("[PACKET RECEIVED]"));

      Serial.print(F("pkt_sender_address="));
      Serial.println(uWrapper.getPkt_sender_address());

      float f = uWrapper.getRem_azimuth_deg();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
        Serial.print("Azimuth, °=");
        Serial.println(f);
      }

      Serial.print(F("Packet: "));
      uWrapper.getPkt_received(pkt, &pkt_size);
      for (int i = 0; i < pkt_size; i++)
      {
        Serial.print(DIGIT_2HEX(pkt[i] / 16));
        Serial.print(DIGIT_2HEX(pkt[i] % 16));
      }
      Serial.println();      
    }
    else if (result & uWAVE_PKT_SEND_FAILED)
    {
      Serial.println(F("[PACKET SENDING FAILED]"));

      Serial.print(F("pkt_tries_taken="));
      Serial.println(uWrapper.getPkt_tries_taken());
      Serial.print(F("pkt_target_address="));
      Serial.println(uWrapper.getPkt_target_address());

      Serial.print(F("Packet: "));
      uWrapper.getPkt_failed(pkt, &pkt_size);
      for (int i = 0; i < pkt_size; i++)
      {
        Serial.print(DIGIT_2HEX(pkt[i] / 16));
        Serial.print(DIGIT_2HEX(pkt[i] % 16));
      }
      Serial.println();
    }
    else if (result & uWAVE_PKT_ITG_RESULT_RECEIVED)
    {      
      Serial.println(F("[PACKET ITG RESULT RECEIVED]"));      
      Serial.print(F("Target address="));
      Serial.println(uWrapper.getPkt_target_address());
      Serial.print(F("Requested data ID="));
      Serial.println(uWrapper.getPkt_ITG_DataID());

      float f;

      f = uWrapper.getPkt_ITG_DataValue();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
         Serial.print("Data value=");
         Serial.println(f);        
      }

      Serial.print(F("Propagation time, s="));
      Serial.println(uWrapper.getRem_propTime_s());

      f = uWrapper.getRem_azimuth_deg();
      if (uWAVE_IF_FLOAT_VALID(f))
      {
         Serial.print("Azimuth=");
         Serial.println(f);        
      }  
    }
    
    if (result & uWAVE_LOCAL_TIMEOUT)
    {
      Serial.println(F("[LOCAL TIMEOUT]"));      
      Serial.print(F("SntID="));
      Serial.println((char)uWrapper.getLast_request_sntID());
    }

    if (result & uWAVE_REMOTE_TIMEOUT)
    {
      Serial.println(F("[REMOTE TIMEOUT]"));
      
      Serial.print(F("requested txID="));
      Serial.println(uWrapper.getRem_requested_txID());
      Serial.print(F("requested rxID="));
      Serial.println(uWrapper.getRem_requested_rxID());
      Serial.print(F("requested cmdID="));
      Serial.println(uWrapper.getRem_requested_rcID());      
    }

    if (result & uWAVE_PKT_ACK_TIMEOUT)
    {
      Serial.println(F("[PACKET ACK TIMEOUT]"));

      Serial.print(F("Target address="));
      Serial.println(uWrapper.getPkt_target_address());
    }

    if (result & uWAVE_PKT_ITG_TIMEOUT)
    {
      Serial.println(F("[PACKET ITG TIMEOUT]"));

      Serial.print(F("Target address="));
      Serial.println(uWrapper.getPkt_target_address());
      Serial.print(F("Requested data ID="));
      Serial.println(uWrapper.getPkt_ITG_DataID());
    }

    if (result & uWAVE_PKT_ITG_TIMEOUT_RECEIVED)
    {
      Serial.println(F("[PACKET ITG TIMEOUT RECEIVED]"));

      Serial.print(F("Target address="));
      Serial.println(uWrapper.getPkt_target_address());
      Serial.print(F("Requested data ID="));
      Serial.println(uWrapper.getPkt_ITG_DataID());
    }    
  }
  else
  {
    if (!uWrapper.isWaitingLocal())
    {
      if (!uWrapper.deviceInfoUpdated())
      {
        if (uWrapper.queryForDeviceInfo())
           Serial.println(F("Querying device info..."));
        else
           Serial.println(F("Failed to query device info!"));
      }
      else if (!uWrapper.ambConfigUpdated())
      {
        if (uWrapper.queryForAmbientDataConfig(false, AMB_PERIOD_MS, AMB_IS_PRESSURE, AMB_IS_TEMPERATURE, AMB_IS_DEPTH, AMB_IS_VOLTAGE))
           Serial.println(F("Querying ambient data configuration..."));
        else
           Serial.println(F("Failed to config ambient data!"));
      }
      else if (!uWAVE_IF_DEFINED_VAL(uWrapper.isPktMode()))
      {
        if (uWrapper.queryForPktModeSettings())
           Serial.println(F("Querying packet mode settings..."));
        else
           Serial.println(F("Failed to query packet mode settings!"));    
      }

#ifndef USE_PKT_MODE
      
      else if (!uWrapper.isWaitingRemote() && !uWrapper.isWaitingPacketACK() && (millis() - rem_req_ts >= REMOTE_REQUEST_PERIOD_MS))
      {
        if (uWrapper.queryRemoteModem(REMOTE_MODEM_RX_ID, REMOTE_MODEM_TX_ID, RC_DPT_GET))
        {
           Serial.println(F("Querying a remote modem..."));
           rem_req_ts = millis();
        }
        else
           Serial.println(F("Failed to query a remote modem!"));
      }
      
#else

      else if (!uWrapper.isWaitingRemote() && !uWrapper.isWaitingPacketACK() && !uWrapper.isWaitingPacketITG() && (millis() - rem_req_ts >= REMOTE_REQUEST_PERIOD_MS))
      {
        if (uWrapper.queryForPktITG(REMOTE_PKT_ADDR, REMOTE_PKT_DID))
        {
           Serial.println(F("Querying a remote modem..."));
           rem_req_ts = millis();
        }
        else
           Serial.println(F("Failed to query a remote modem!"));
      }

#endif



      
    }
  }
}
