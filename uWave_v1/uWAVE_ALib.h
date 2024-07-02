#ifndef _UWAVE_ALIB_H_
#define _UWAVE_ALIB_H_

#include <Arduino.h>
//#include <SoftwareSerial.h>

/*
 * @file uWAVE_ALib.h
 * @author Alek Dikarev
 *
 * @class uWAVE
 * @brief Provides functionality to interface with uWAVE underwater acoustic modems.
 */

#define uWAVE_UNDEFINED_VAL       (-1)
#define uWAVE_IF_DEFINED_VAL(v)   ((v) != uWAVE_UNDEFINED_VAL)

#define uWAVE_UNDEFINED_FLOAT_VAL (-32768)
#define uWAVE_IF_FLOAT_VALID(f)   ((f) != uWAVE_UNDEFINED_FLOAT_VAL)

#define uWAVE_OUT_BUFFER_SIZE     (128)
#define uWAVE_IN_BUFFER_SIZE      (128)
#define uWAVE_PKT_MAX_SIZE        (64)

#define uWAVE_LOC_TIMEOUT_MS      (3000)
#define uWAVE_REM_TIMEOUT_MS      (6000)
#define uWAVE_PKT_SEND_TIMEOUT_MS (uWAVE_REM_TIMEOUT_MS * 255)

#define PUWV_SIGNATURE            (0x50555756)

#define HEX_DIGIT2B(b)            ((b) >= 0x41 ? ((b) - 0x37) : ((b) - 0x30))
#define DIGIT_2HEX(h)             ((h) > 9     ? ((h) + 0x37) : ((h) + 0x30))

#define C2B(b)                    ((b - '0'))
#define CC2B(b1, b2)              ((10 * (b1 - '0') + (b2 - '0')))
#define CCC2B(b1, b2, b3)         ((100 * (b1 - '0') + 10 * (b2 - '0') + (b3 - '0')))

#define NMEA_SNT_STR              '$'
#define NMEA_SNT_END              '\n'
#define NMEA_SNT_END1             '\r'
#define NMEA_PAR_SEP              ','
#define NMEA_CHK_SEP              '*'

#define IC_D2H_ACK                '0'        // $PUWV0,cmdID,errCode
#define IC_H2D_SETTINGS_WRITE     '1'        // $PUWV1,rxChID,txChID,styPSU,isCmdMode,isACKOnTXFinished,gravityAcc
#define IC_H2D_RC_REQUEST         '2'        // $PUWV2,txChID,rxChID,rcCmdID
#define IC_D2H_RC_RESPONSE        '3'        // $PUWV3,txChID,rcCmdID,propTime_seс,snr,[value],[azimuth]
#define IC_D2H_RC_TIMEOUT         '4'        // $PUWV4,txChID,rcCmdID
#define IC_D2H_RC_ASYNC_IN        '5'        // $PUWV5,rcCmdID,snr,[azimuth]
#define IC_H2D_AMB_DTA_CFG        '6'        // $PUWV6,isSaveInFlash,periodMs,isPrs,isTemp,isDpt,isBatV
#define IC_D2H_AMB_DTA            '7'        // $PUWV7,prs_mBar,temp_C,dpt_m,batVoltage_V

// packet mode
#define IC_H2D_PT_SETTINGS_READ   'D'        // $PUWVD,reserved
#define IC_D2H_PT_SETTINGS        'E'        // $PUWVE,isPTMode,ptAddress
#define IC_H2H_PT_SETTINGS_WRITE  'F'        // $PUWVF,isSaveInFlash,isPTMode,ptAddress

#define IC_H2D_PT_SEND            'G'        // $PUWVG,tareget_ptAddress,[maxTries],dataPacket
#define IC_D2H_PT_FAILED          'H'        // $PUWVH,tareget_ptAddress,triesTaken,dataPacket
#define IC_D2H_PT_DLVRD           'I'        // $PUWVI,tareget_ptAddress,triesTaken,dataPacket
#define IC_D2H_PT_RCVD            'J'        // $PUWVJ,sender_ptAddress,dataPacket

#define IC_H2D_PT_ITG             'K'        // $PUWVK,target_ptAddress,pt_itg_dataID
#define IC_D2H_PT_TMO             'L'        // $PUWVL,target_ptAddress,pt_itg_dataID
#define IC_D2H_PT_ITG_RESP        'M'        // $PUWVM,target_ptAddress,pt_itg_dataID,[dataValue],pTime,[azimuth]
//

#define IC_H2D_DINFO_GET          '?'        // $PUWV?,reserved
#define IC_D2H_DINFO              '!'        // $PUWV!,serialNumber,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,isCmdMode

#define IC_D2H_UNKNOWN            '-'



enum uWAVE_EVENT_Enum : int
{
  uWAVE_NONE                      = 0,
  uWAVE_ACK_RECEIVED              = 2,
  uWAVE_LOCAL_TIMEOUT             = 4,
  uWAVE_REMOTE_RESPONSE_RECEIVED  = 8,
  uWAVE_REMOTE_TIMEOUT            = 16,
  uWAVE_DEVICE_INFO_RECEIVED      = 32,
  uWAVE_ASYNC_IN_RECEIVED         = 64,
  uWAVE_AMB_DATA_RECEIVED         = 128,
  uWAVE_PKT_SETTINGS_RECEIVED     = 256,
  uWAVE_PKT_DELIVERED             = 512,
  uWAVE_PKT_RECEIVED              = 1024,
  uWAVE_PKT_SEND_FAILED           = 2048,
  uWAVE_PKT_ACK_TIMEOUT           = 4096,
  uWAVE_PKT_ITG_TIMEOUT           = 8192,
  uWAVE_PKT_ITG_RESULT_RECEIVED   = 16384,
  uWAVE_PKT_ITG_TIMEOUT_RECEIVED  = 32768
};

enum uWAVE_RC_CODES_Enum : int
{
  RC_PING        = 0,
  RC_PONG        = 1,
  RC_DPT_GET     = 2,
  RC_TMP_GET     = 3,
  RC_BAT_V_GET   = 4,
  RC_ERR_NSUP    = 5,
  RC_ACK         = 6,
  RC_USR_CMD_000 = 7,
  RC_USR_CMD_001 = 8,
  RC_USR_CMD_002 = 9,
  RC_USR_CMD_003 = 10,
  RC_USR_CMD_004 = 11,
  RC_USR_CMD_005 = 12,
  RC_USR_CMD_006 = 13,
  RC_USR_CMD_007 = 14,
  RC_USR_CMD_008 = 15,
  RC_MSG_ASYNC   = 16,
  RC_INVALID
};

enum DataID_Enum : int
{
  DID_DPT = 0,
  DID_TMP = 1,
  DID_BAT = 2,
  DID_INVALID

};


enum uWAVE_ERR_CODES_Enum : int
{
  LOC_ERR_NO_ERROR              = 0,
  LOC_ERR_INVALID_SYNTAX        = 1,
  LOC_ERR_UNSUPPORTED           = 2,
  LOC_ERR_TRANSMITTER_BUSY      = 3,
  LOC_ERR_ARGUMENT_OUT_OF_RANGE = 4,
  LOC_ERR_INVALID_OPERATION     = 5,
  LOC_ERR_UNKNOWN_FIELD_ID      = 6,
  LOC_ERR_VALUE_UNAVAILABLE     = 7,
  LOC_ERR_RECEIVER_BUSY         = 8,
  LOC_ERR_TX_BUFFER_OVERRUN     = 9,
  LOC_ERR_CHKSUM_ERROR          = 10,
  LOC_ERR_TX_FINISHED           = 11,
  LOC_ACK_BEFORE_STANDBY        = 12,
  LOC_ACK_AFTER_WAKEUP          = 13,
  LOC_ERR_SVOLTAGE_TOO_HIGH     = 14,
  LOC_ERR_UNKNOWN
};

class uWAVE 
{
  public:
    uWAVE(HardwareSerial* port, int cmd_pin);

    uWAVE_EVENT_Enum process();

    void enable();
    void disable();       

    bool deviceInfoUpdated();
    bool ambConfigUpdated();

    bool isEnabled();
    int isPktMode();
    int pktModeLocalAddress();

    int isPTSPresent();

    int getTxChID();
    int getRxChID();
    int getTotalCodeChannels();

    int isCmdModeByDefault();
    int isACKOnTXFinished();

    float getAcousticBaudrate();
    float getSalinityPSU();

    bool isWaitingLocal();
    bool isWaitingRemote();
    bool isWaitingPacketACK();
    bool isWaitingPacketITG();

    // refactor
    byte _serial[24];
    byte _core_moniker[32];
    byte _system_moniker[32];
    int _core_version;
    int _system_version;
    //
    
    uWAVE_ERR_CODES_Enum getACK_result();  
    byte getLast_request_sntID();
        
    uWAVE_RC_CODES_Enum getRem_requested_rcID();
    byte getRem_requested_txID();
    byte getRem_requested_rxID();
    float getRem_propTime_s();
    float getRem_msr_dB();
    float getRem_value();
    float getRem_azimuth_deg();

    uWAVE_RC_CODES_Enum getRem_async_in_rc();
    
    
    byte getPkt_tries_taken();
    byte getPkt_target_address();
    byte getPkt_sender_address();
    void getPkt_delivered(byte* data, byte* dataSize);
    void getPkt_received(byte* data, byte* dataSize);
    void getPkt_failed(byte* data, byte* dataSize);

    DataID_Enum getPkt_ITG_DataID();
    float getPkt_ITG_DataValue();

    float getAmb_pressure_mBar();
    float getAmb_depth_m();
    float getAmb_temperature_C();
    float getAmb_voltage_V();
    
    
    bool queryForDeviceInfo();
    bool queryForSettingsUpdate(byte txChID, byte rxChID, float salinityPSU, bool isCmdModeByDefault, bool isACKOnTxFinished, float gravityAcc);
    bool queryForAmbientDataConfig(bool isSaveToFlash, int periodMs, bool isPressure, bool isTemperature, bool isDepth, bool isVCC);
    bool queryRemoteModem(byte txChID, byte rxChID, uWAVE_RC_CODES_Enum cmdID);
    
    bool queryForPktModeSettings();
    bool queryForPktModeSettingsUpdate(bool isSaveToFlash, bool isPktMode, byte localAddress);
    bool queryForPktAbortSend();
    bool queryForPktSend(byte targetPktAddress, byte* data, byte dataSize);
    bool queryForPktSend(byte targetPktAddress, byte maxTries, byte* data, byte dataSize);    
    bool queryForPktITG(byte targetPktAddress, DataID_Enum dataID);
    
  private:
    HardwareSerial *_port;
    int _cmd_pin;
    
    bool _enabled;
    int _isPktMode;
    int _pktModeLocalAddress;

    int _isPTSPresent;

    int _txChID;
    int _rxChID;
    int _totalCodeChannels;

    int _isCmdModeByDefault;
    int _isACKOnTXFinished;

    float _acousticBaudrate;
    float _salinityPSU;

    bool _deviceInfoUpdated;
    bool _ambConfigUpdated;

    bool _isWaitingLocal;
    bool _isWaitingRemote;
    bool _isWaitingPacketACK;
    bool _isWaitingPacketITG;

    byte _out_buffer_idx;
    byte _out_buffer[uWAVE_OUT_BUFFER_SIZE];
    byte _in_buffer[uWAVE_IN_BUFFER_SIZE];

    // ACK data
    byte _ACK_sntID;
    uWAVE_ERR_CODES_Enum _ACK_result;

    // Local timeout data
    byte _last_request_sntID;
    long _last_request_ts;

    // Remote request data
    long _rem_request_ts;
    uWAVE_RC_CODES_Enum _rem_requested_rcID;
    byte _rem_requested_txID;
    byte _rem_requested_rxID;
    float _rem_propTime_s;
    float _rem_msr_dB;
    float _rem_value;
    float _rem_azimuth_deg;

    // Rem async in
    uWAVE_RC_CODES_Enum _rem_async_in_rc;

    // Pkt data
    long _pkt_sent_ts;
    byte _pkt_tries_taken;
    byte _pkt_target_address;
    byte _pkt_sender_address;
    long _pt_itg_ts;
    
    float _pkt_itg_dataValue;
    DataID_Enum _pkt_itg_dataID;
    
    byte _pkt_out_packet[uWAVE_PKT_MAX_SIZE];
    byte _pkt_out_size;
    
    byte _pkt_in_size;
    byte _pkt_in_packet[uWAVE_PKT_MAX_SIZE];
    
    // AMB data
    float _amb_pressure_mBar;
    float _amb_depth_m;
    float _amb_temperature_C;
    float _amb_voltage_V;    

    // parser
    bool NMEA_Process(byte);
    byte  _in_buffer_idx;
    
    bool  _in_buffer_isReady;
    bool  _in_buffer_isStarted;
  
    byte  _in_buffer_chk_act;
    byte  _in_buffer_chk_dcl;
    byte  _in_buffer_chk_dcl_idx;
    bool  _in_buffer_chk_present;
      
    long  _in_buffer_sign;


    void Str_WriterInit(byte* buffer, byte* srcIdx, byte bufferSize);
    void Str_WriteByte(byte* buffer, byte* srcIdx, byte c);
    void Str_WriteIntDec(byte* buffer, byte* srcIdx, int src, byte zPad);
    void Str_WriteFloat(byte* buffer, byte* srcIdx, float f, byte dPlaces, byte zPad);
    void Str_WriteHexByte(byte* buffer, byte* srcIdx, byte c);
    void Str_WriteHexArray(byte* buffer, byte* srcIdx, byte* src, byte srcSize);
    void Str_WriteHexStr(byte* buffer, byte* srcIdx, byte* src, byte srcSize);
    void Str_WriteStr(byte* buffer, byte* srcIdx, byte* src);
    void NMEA_CheckSum_Update(byte* buffer, byte size);
    
    float Str_ParseFloat(const byte* buffer, byte stIdx, byte ndIdx);
    int Str_ParseIntDec(const byte* buffer, byte stIdx, byte ndIdx);
    byte Str_ParseHexByte(const byte* buffer, byte stIdx);
    int Str_ReadHexStr(const byte* buffer, byte stIdx, byte ndIdx, byte* out_buffer, byte out_buffer_size, byte* out_size);
    void Str_ReadString(const byte* src_buffer, byte* dst_buffer, byte stIdx, byte ndIdx);
    
    bool NMEA_GetNextParam(const byte* buffer, byte fromIdx, byte bufferSize, byte* stIdx, byte* ndIdx);

    bool ACK_Parse();
    bool RC_RESPONSE_Parse();
    bool RC_TIMEOUT_Parse();
    bool RC_ASYNC_IN_Parse();
    bool AMD_DTA_Parse();
    bool PT_SETTINGS_Parse();
    bool PT_FAILED_Parse();
    bool PT_DLVRD_Parse();
    bool PT_RCVD_Parse();
    bool PT_ITG_TMO_Parse();
    bool PT_ITG_RES_Parse();
    bool D_INFO_Parse();
};


#endif
