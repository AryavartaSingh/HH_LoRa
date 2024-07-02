#include "uWAVE_ALib.h"


void uWAVE::Str_WriterInit(byte* buffer, byte* srcIdx, byte bufferSize)
{
  *srcIdx = 0;
  for (int i = 0; i < bufferSize; i++)
    buffer[i] = 0;
}

void uWAVE::Str_WriteByte(byte* buffer, byte* srcIdx, byte c)
{
  buffer[*srcIdx] = c;
  (*srcIdx)++;
}

void uWAVE::Str_WriteIntDec(byte* buffer, byte* srcIdx, int src, byte zPad)
{
  int x = src, len = 0, i;

  do { x /= 10; len++; } while (x >= 1);

  x = 1;
  for (i = 1; i < len; i++) x *= 10;

  if (zPad > 0) i = zPad;
  else i = len;

  do
  {
    if (i > len) buffer[*srcIdx] = '0';
    else
    {
      buffer[*srcIdx] = (byte)((src / x) + '0');
      src -= (src / x) * x;
      x /= 10;
    }
    (*srcIdx)++;
  } while (--i > 0);
}

void uWAVE::Str_WriteFloat(byte* buffer, byte* srcIdx, float f, byte dPlaces, byte zPad)
{
  float ff = f;

  if (ff < 0)
  {
    Str_WriteByte(buffer, srcIdx, '-');
    ff = -f;
  }

  int dec = (int)ff, mult = 1, i;
  for (i = 0; i < dPlaces; i++) mult *= 10;
  int frac = (int)((ff - dec) * (float)mult);

  Str_WriteIntDec(buffer, srcIdx, dec, zPad);
  Str_WriteByte(buffer, srcIdx, '.');
  Str_WriteIntDec(buffer, srcIdx, frac, dPlaces);
}

void uWAVE::Str_WriteStr(byte* buffer, byte* srcIdx, byte* src)
{
  byte c;
  c = *src;
  while (c != '\0') 
  { 
    buffer[(*srcIdx)++] = c;
    c = *++src; 
  }
}

void uWAVE::Str_WriteHexByte(byte* buffer, byte* srcIdx, byte c)
{  
  buffer[*srcIdx] = DIGIT_2HEX(c / 16);
  (*srcIdx)++;
  buffer[*srcIdx] = DIGIT_2HEX(c % 16);
  (*srcIdx)++;
}


void uWAVE::Str_WriteHexArray(byte* buffer, byte* srcIdx, byte* src, byte srcSize)
{
  int i;
  Str_WriteStr(buffer, srcIdx, ((byte*)"0x\0"));
  for (i = 0; i < srcSize; i++) { Str_WriteHexByte(buffer, srcIdx, src[i]); }
}

void uWAVE::Str_WriteHexStr(byte* buffer, byte* srcIdx, byte* src, byte srcSize)
{
  int i;
  for (i = 0; i < srcSize; i++) { Str_WriteHexByte(buffer, srcIdx, src[i]); }
}




void uWAVE::NMEA_CheckSum_Update(byte* buffer, byte size)
{
  byte i;
  byte acc = 0, b1, b2;
  for (i = 0; i < size; i++)
  {
    if (buffer[i] == NMEA_SNT_STR)
    {
      acc = 0;
    }
    else if (buffer[i] == NMEA_CHK_SEP)
    {
      b1 = acc / 16;
      if (b1 > 9)
      {
        b1 += ('A' - 10);
      }
      else
      {
        b1 += '0';      
      }
      b2 = acc % 16;
      
      if (b2 > 9)
      {
        b2 += ('A' - 10);
      }
      else
      {
        b2 += '0';
      }
      
      buffer[i + 1] = b1;
      buffer[i + 2] = b2;
    }
    else
    {
      acc ^= buffer[i];
    }
  }
}




float uWAVE::Str_ParseFloat(const byte* buffer, byte stIdx, byte ndIdx)
{
  int i, dotIdx = ndIdx + 1;
  float sign = 1.0f, fract = 0.0f;

  if (buffer[stIdx] == '-')
  {
    sign = -1.0f;
    stIdx++;
  }

  for (i = stIdx; i <= ndIdx; i++)
  {
    if (buffer[i] == '.')
    {
      dotIdx = i;
    }
  }

  float result = 0.0f;
  float multiplier = 1.0f;

  for (i = dotIdx - 1; i >= stIdx; i--)
  {
    result += ((float)((buffer[i] - '0'))) * multiplier;
    multiplier *= 10.0f;
  }

  multiplier = 0.1f;

  for (i = dotIdx + 1; i <= ndIdx; i++)
  {
    fract += ((float)((buffer[i] - '0'))) * multiplier;
    multiplier /= 10.0f;
  }

  result += fract;
  return result * sign;
}

int uWAVE::Str_ParseIntDec(const byte* buffer, byte stIdx, byte ndIdx)
{
  byte i;
  int sign = 1;

  if (buffer[stIdx] == '-')
  {
    sign = -1;
    stIdx++;
  }

  int result = 0;
  int multiplier = 1;

  for (i = ndIdx; i >= stIdx; i--)
  {
    result += ((int)((buffer[i] - '0'))) * multiplier;
    multiplier *= 10;
  }

  return result;
}

byte uWAVE::Str_ParseHexByte(const byte* buffer, byte stIdx)
{
  return HEX_DIGIT2B(buffer[stIdx]) * 16 + HEX_DIGIT2B(buffer[stIdx + 1]);
}

int uWAVE::Str_ReadHexStr(const byte* buffer, byte stIdx, byte ndIdx, byte* out_buffer, byte out_buffer_size, byte* out_size)
{
  int result = 0;  
  for (int i = 0; i < out_buffer_size; i++)
     out_buffer[i] = 0;
  
  *out_size = (ndIdx - stIdx - 1);
  if ((*out_size < 0) || ((*out_size) % 2 != 0))
  {
    result = 1;
  }
  else
  {
    *out_size /= 2;
    if ((buffer[stIdx] != '0') || (buffer[stIdx + 1] != 'x'))
      result = 2;
    else
    {
      int idx = 0;
      while (idx < *out_size)
      {
        out_buffer[idx] = Str_ParseHexByte(buffer, stIdx + idx * 2 + 2);
        idx++;
      }
    }
  }

  return result;
}

void uWAVE::Str_ReadString(const byte* src_buffer, byte* dst_buffer, byte stIdx, byte ndIdx)
{
  for (byte i = stIdx; i <= ndIdx; i++) 
  {
    dst_buffer[i - stIdx] = src_buffer[i];
  }
}



bool uWAVE::NMEA_GetNextParam(const byte* buffer, byte fromIdx, byte bufferSize, byte* stIdx, byte* ndIdx)
{
  byte i = fromIdx + 1;
  *stIdx = fromIdx;
  *ndIdx = *stIdx;

  while ((i <= bufferSize) && (*ndIdx == *stIdx))
  {
    if ((buffer[i] == NMEA_PAR_SEP) ||
        (buffer[i] == NMEA_CHK_SEP) ||
        (buffer[i] == NMEA_SNT_END1) ||
        (i == bufferSize))
    {
      *ndIdx = i;
    }
    else
    {
      i++;
    }
  }
  (*stIdx)++;
  (*ndIdx)--;
  
  return ((buffer[i] != NMEA_CHK_SEP) && (i != bufferSize) && (buffer[i] != NMEA_SNT_END1));
}

bool uWAVE::ACK_Parse()
{
  // $PUWV0,sndID,errCode
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  uWAVE_ERR_CODES_Enum errCode = LOC_ERR_UNKNOWN;
  char sntID = IC_D2H_UNKNOWN;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          sntID = char(_in_buffer[stIdx]);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          errCode = (uWAVE_ERR_CODES_Enum)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
    }
    
    pIdx++;
  } while (result && isNotLastParam); 

  if (result)
  {
    _ACK_sntID = sntID;
    _ACK_result = errCode;
    
    if (errCode == LOC_ERR_NO_ERROR)
    {    
      if (sntID == IC_H2D_RC_REQUEST)
      {
        _isWaitingRemote = true;
        _rem_request_ts = millis();    
      }
      else if (sntID == IC_H2D_AMB_DTA_CFG)
      {
        _ambConfigUpdated = true;
      }
      else if (sntID == IC_H2D_PT_SEND)
      {        
        if (_pkt_out_size == 0)
           _isWaitingPacketACK = false; // ACK for AbortSend
        else
        {
          _isWaitingPacketACK = true; // ACK for packet send
          _pkt_sent_ts = millis();
        }
      }
      else if (sntID == IC_H2D_PT_ITG)
      {
        _isWaitingPacketITG = true;
        _pt_itg_ts = millis();
      }
    }
  }

  return result;
}

bool uWAVE::RC_RESPONSE_Parse()
{
  // $PUWV3,txChID,rcCmdID,propTime_seс,msr,[value],[azimuth]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;
  byte txChID = 255;
  uWAVE_RC_CODES_Enum rcCmdID = RC_INVALID;
  float pTime = uWAVE_UNDEFINED_FLOAT_VAL;
  float msr = uWAVE_UNDEFINED_FLOAT_VAL;
  float value = uWAVE_UNDEFINED_FLOAT_VAL;
  float azimuth = uWAVE_UNDEFINED_FLOAT_VAL;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          txChID = (byte)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          rcCmdID = (uWAVE_RC_CODES_Enum)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          result = false;
        else
          pTime = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 4:
        if (ndIdx < stIdx)
          result = false;
        else
          msr = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 5:
        if (ndIdx > stIdx)
          value = Str_ParseFloat(_in_buffer, stIdx, ndIdx);        
        break;
      case 6:
        if (ndIdx > stIdx)
          azimuth = Str_ParseFloat(_in_buffer, stIdx, ndIdx);        
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  if (result)
  {
    _rem_requested_rcID = rcCmdID;
    _rem_propTime_s = pTime;
    _rem_msr_dB = msr;
    _rem_value = value;
    _rem_azimuth_deg = azimuth;
  }

  return result;
}

bool uWAVE::RC_TIMEOUT_Parse()
{
  // $PUWV4,txChID,rcCmdID
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  byte txChID = 255;
  uWAVE_RC_CODES_Enum rcCmdID = RC_INVALID;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          txChID = (byte)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          //rcCmdID = Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
          rcCmdID = static_cast<uWAVE_RC_CODES_Enum>(Str_ParseIntDec(_in_buffer, stIdx, ndIdx));

        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  if (result)
  {
    _rem_requested_txID = txChID;
    _rem_requested_rcID = rcCmdID;
  }

  return result;
}

bool uWAVE::RC_ASYNC_IN_Parse()
{
  // $PUWV5,rcCmdID,msr,[azimuth]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;
  
  uWAVE_RC_CODES_Enum rcCmdID = RC_INVALID;
  float msr = uWAVE_UNDEFINED_FLOAT_VAL;
  float azimuth = uWAVE_UNDEFINED_FLOAT_VAL;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          rcCmdID = (uWAVE_RC_CODES_Enum)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          msr = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          result = false;
        else
          azimuth = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
    }

    pIdx++;
  } while (result && isNotLastParam); 

  if (result)
  {
    _rem_async_in_rc = rcCmdID;
    _rem_msr_dB = msr;
    _rem_azimuth_deg = azimuth;
  }

  return result;
}

bool uWAVE::AMD_DTA_Parse()
{
  // $PUWV5,rcCmdID,msr,[azimuth]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;
  
  float prs = uWAVE_UNDEFINED_FLOAT_VAL;
  float dpt = uWAVE_UNDEFINED_FLOAT_VAL;
  float tmp = uWAVE_UNDEFINED_FLOAT_VAL;
  float vcc = uWAVE_UNDEFINED_FLOAT_VAL;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);
    switch (pIdx)
    {
      case 1:
        if (ndIdx > stIdx)
          prs = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx > stIdx)
          tmp = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx > stIdx)
          dpt = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 4:
        if (ndIdx > stIdx)
          vcc = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;  
    }

    pIdx++;
  } while (result && isNotLastParam); 

  if (result)
  {
    _amb_pressure_mBar = prs;
    _amb_depth_m = dpt;
    _amb_temperature_C = tmp;
    _amb_voltage_V = vcc;
  }

  return result;
}

bool uWAVE::PT_SETTINGS_Parse()
{
  // $PUWVE,isPTMode,ptAddress
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  bool isPTMode = false;
  byte ptAddress = 255;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          isPTMode = bool(_in_buffer[stIdx]);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          ptAddress = (byte)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
    }
    
    pIdx++;
  } while (result && isNotLastParam); 

  if (result)
  {
    _isPktMode = isPTMode;
    _pktModeLocalAddress = ptAddress;
  }

  return result;
}

bool uWAVE::PT_FAILED_Parse()
{
  // $PUWVH,target_ptAddress,triesTaken,dataPacket
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  byte target_ptAddress = 255;
  byte triesTaken = 255;
    
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          target_ptAddress = byte(_in_buffer[stIdx]);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          triesTaken = (byte)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 3:
        if ((ndIdx < stIdx) || (Str_ReadHexStr(_in_buffer, stIdx, ndIdx, _pkt_out_packet, uWAVE_PKT_MAX_SIZE, &_pkt_out_size) != 0))
          result = false;
        break;
    }
    
    pIdx++;
  } while (result && !isNotLastParam); 

  if (result)
  {
    _pkt_tries_taken = triesTaken;
    _pkt_target_address = target_ptAddress;
  }

  return result;
}

bool uWAVE::PT_DLVRD_Parse()
{
  // $PUWVI,tareget_ptAddress,[azimuth],triesTaken,dataPacket
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  byte target_ptAddress = 255;
  byte triesTaken = 255;
  float azimuth = uWAVE_UNDEFINED_FLOAT_VAL;
    
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          target_ptAddress = byte(_in_buffer[stIdx]);
        break;
      case 2:
        if (ndIdx > stIdx)
          azimuth = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          result = false;
        else
          triesTaken = (byte)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 4:
        if ((ndIdx < stIdx) || (Str_ReadHexStr(_in_buffer, stIdx, ndIdx, _pkt_out_packet, uWAVE_PKT_MAX_SIZE, &_pkt_out_size) != 0))
          result = false;
        break;
    }
    
    pIdx++;
  } while (result && isNotLastParam); 

  if (result)
  {
    _pkt_tries_taken = triesTaken;
    _pkt_target_address = target_ptAddress;

    if (azimuth != uWAVE_UNDEFINED_FLOAT_VAL)
       _rem_azimuth_deg = azimuth;
  }

  return result;
}

bool uWAVE::PT_RCVD_Parse()
{
  // $PUWVJ,sender_ptAddress,[azimuth],dataPacket
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  byte sender_ptAddress = 255;
  float azimuth = uWAVE_UNDEFINED_FLOAT_VAL;
    
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          sender_ptAddress = byte(_in_buffer[stIdx]);
        break;
      case 2:
        if (ndIdx > stIdx)
          azimuth = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
         break;
      case 3:
        if ((ndIdx < stIdx) || (Str_ReadHexStr(_in_buffer, stIdx, ndIdx, _pkt_in_packet, uWAVE_PKT_MAX_SIZE, &_pkt_in_size) != 0))
          result = false;
        break;
    }
    
    pIdx++;
  } while (result && isNotLastParam); 

  if (result)
  {
    _pkt_sender_address = sender_ptAddress;

    if (azimuth != uWAVE_UNDEFINED_FLOAT_VAL)
       _rem_azimuth_deg = azimuth;
  }

  return result;
}

bool uWAVE::PT_ITG_TMO_Parse()
{
  // $PUWVL,target_ptAddress,pt_itg_dataID
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  byte target_ptAddress = 255;
  DataID_Enum dataID = DID_INVALID;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          target_ptAddress = (byte)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          dataID = static_cast<DataID_Enum>(Str_ParseIntDec(_in_buffer, stIdx, ndIdx));
          
        break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  if (result)
  {
    _pkt_target_address = target_ptAddress;
    _pkt_itg_dataID = dataID;
  }

  return result;
}

bool uWAVE::PT_ITG_RES_Parse()
{
  // $PUWVM,target_ptAddress,pt_itg_dataID,[dataValue],pTime,[azimuth]
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;

  byte target_ptAddress = 255;
  DataID_Enum dataID = DID_INVALID;
  float dataValue = uWAVE_UNDEFINED_FLOAT_VAL;
  float pTime = uWAVE_UNDEFINED_FLOAT_VAL;
  float azimuth = uWAVE_UNDEFINED_FLOAT_VAL;
  
  do
  {
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {
      case 1:
        if (ndIdx < stIdx)
          result = false;
        else
          target_ptAddress = (byte)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
      break;
      
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          dataID = static_cast<DataID_Enum>(Str_ParseIntDec(_in_buffer, stIdx, ndIdx));

      break;

      case 3:
        if (ndIdx >= stIdx)          
          dataValue = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
      break;

      case 4:
        if (ndIdx < stIdx)
          result = false;
        else
          pTime = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
      break;

      case 5:
      if (ndIdx >= stIdx)
          azimuth = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
      break;
    }

    pIdx++;
  } while (result && isNotLastParam);

  if (result)
  {
    _pkt_target_address = target_ptAddress;
    _pkt_itg_dataID = dataID;

    if (dataValue != uWAVE_UNDEFINED_FLOAT_VAL)
      _pkt_itg_dataValue = dataValue;

    _rem_propTime_s = pTime;

    if (azimuth != uWAVE_UNDEFINED_FLOAT_VAL)
      _rem_azimuth_deg = azimuth;
  }

  return result;
}






bool uWAVE::D_INFO_Parse()
{
  // $PUWV!,serialNumber,sys_moniker,sys_version,core_moniker [release],core_version,acBaudrate,rxChID,txChID,totalCh,salinityPSU,isPTS,isCmdMode
  byte stIdx = 0, ndIdx = 0, pIdx = 0;
  bool result = true, isNotLastParam;  
  
  do
  {         
    isNotLastParam = NMEA_GetNextParam(_in_buffer, ndIdx + 1, _in_buffer_idx, &stIdx, &ndIdx);    
    switch (pIdx)
    {      
      case 1:        
        if (ndIdx < stIdx)
          result = false;
        else
        {
          Str_ReadString(_in_buffer, _serial, stIdx, ndIdx);          
        }
        break;
      case 2:
        if (ndIdx < stIdx)
          result = false;
        else
          Str_ReadString(_in_buffer, _core_moniker, stIdx, ndIdx);
        break;
      case 3:
        if (ndIdx < stIdx)
          result = false;
        else
          _core_version = Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 4:
        if (ndIdx < stIdx)
          result = false;
        else
          Str_ReadString(_in_buffer, _system_moniker, stIdx, ndIdx);
        break;
      case 5:
        if (ndIdx < stIdx)
          result = false;
        else
          _system_version = Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 6:
        if (ndIdx < stIdx)
          result = false;
        else
          _acousticBaudrate = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 7:
        if (ndIdx < stIdx)
          result = false;
        else
          _txChID = Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 8:
        if (ndIdx < stIdx)
          result = false;
        else
          _rxChID = Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 9:
        if (ndIdx < stIdx)
          result = false;
        else
          _totalCodeChannels = Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 10:
        if (ndIdx < stIdx)
          result = false;
        else
          _salinityPSU = Str_ParseFloat(_in_buffer, stIdx, ndIdx);
        break;
      case 11:
        if (ndIdx < stIdx)
          result = false;
        else
          _isPTSPresent = (bool)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      case 12:
        if (ndIdx < stIdx)
          result = false;
        else 
          _isCmdModeByDefault = (bool)Str_ParseIntDec(_in_buffer, stIdx, ndIdx);
        break;
      default:
        break;
    }    
    
    pIdx++;
  } while (result && isNotLastParam);
  

  return result;
}


bool uWAVE::NMEA_Process(byte newByte)
{
  bool result = false;
  
  if (!_in_buffer_isReady) 
  {
    if (newByte == NMEA_SNT_STR)
    {      
      _in_buffer_isStarted = true;
            
      for (int i = 0; i < uWAVE_IN_BUFFER_SIZE; i++)
        _in_buffer[i] = 0;
  
      _in_buffer_chk_act     = 0;
      _in_buffer_chk_dcl     = 0;
      _in_buffer_chk_dcl_idx = 0;
      _in_buffer_idx         = 0;
        
      _in_buffer_sign        = 0;
  
      _in_buffer[_in_buffer_idx] = newByte;
      _in_buffer_idx++;
    }
    else
    {
      if (_in_buffer_isStarted)
      {
        _in_buffer[_in_buffer_idx] = newByte;
        _in_buffer_idx++;
        
        if (newByte == NMEA_SNT_END)
        {
          _in_buffer_isStarted = false;
          _in_buffer_isReady = true;
          result = true;          
        }
        else if (newByte == NMEA_CHK_SEP)
        {
          _in_buffer_chk_dcl_idx = 1;
          _in_buffer_chk_present = true;          
        }
        else
        {
          if (_in_buffer_idx >= uWAVE_IN_BUFFER_SIZE)
          {
            _in_buffer_isStarted = false;
          }
          else
          {          
            if (_in_buffer_chk_dcl_idx == 0)
            {
              _in_buffer_chk_act ^= newByte;
  
              if (_in_buffer_idx == 2)      _in_buffer_sign = ((long)newByte) << 24;
              else if (_in_buffer_idx == 3) _in_buffer_sign |= ((long)newByte) << 16;
              else if (_in_buffer_idx == 4) _in_buffer_sign |= (((long)newByte) << 8);
              else if (_in_buffer_idx == 5) 
              {
                _in_buffer_sign |= ((long)newByte);

                if (_in_buffer_sign != PUWV_SIGNATURE)
                {
                   _in_buffer_isStarted = false;                   
                }
              }
            }
            else if (_in_buffer_chk_dcl_idx == 1)
            {              
              _in_buffer_chk_dcl = 16 * HEX_DIGIT2B(newByte);
              _in_buffer_chk_dcl_idx++;              
            }
            else if (_in_buffer_chk_dcl_idx == 2)
            {              
              _in_buffer_chk_dcl += HEX_DIGIT2B(newByte);
              if (_in_buffer_chk_act != _in_buffer_chk_dcl)
              {
                _in_buffer_isStarted = false;
              }              
              _in_buffer_chk_dcl_idx++;
            }            
          }
        }
      }
    }
  }

  return result;
}





uWAVE::uWAVE(HardwareSerial *port, int cmd_pin)
{
  _port = port;
  _cmd_pin = cmd_pin;
  
  pinMode(_cmd_pin, OUTPUT);

  _isPktMode = uWAVE_UNDEFINED_VAL;
  _pktModeLocalAddress = uWAVE_UNDEFINED_VAL;

  _isPTSPresent = uWAVE_UNDEFINED_VAL;

  _txChID = uWAVE_UNDEFINED_VAL;
  _rxChID = uWAVE_UNDEFINED_VAL;
  _totalCodeChannels = uWAVE_UNDEFINED_VAL;

  _isCmdModeByDefault = uWAVE_UNDEFINED_VAL;
  _isACKOnTXFinished = uWAVE_UNDEFINED_VAL;

  _acousticBaudrate = uWAVE_UNDEFINED_FLOAT_VAL;
  _salinityPSU = uWAVE_UNDEFINED_FLOAT_VAL;

  _deviceInfoUpdated = false;
  _ambConfigUpdated = false;

  _isWaitingLocal = false;
  _isWaitingRemote = false;

  _out_buffer_idx = 0;
  
  _ACK_sntID = IC_D2H_UNKNOWN;
  _ACK_result = LOC_ERR_UNKNOWN;

  _last_request_sntID = IC_D2H_UNKNOWN;
  _last_request_ts = 0;
    
  _rem_request_ts = 0;
  _rem_requested_rcID = RC_INVALID;
  _rem_requested_txID = 0;
  _rem_requested_rxID = 0;
  _rem_propTime_s = uWAVE_UNDEFINED_FLOAT_VAL;
  _rem_msr_dB = uWAVE_UNDEFINED_FLOAT_VAL;
  _rem_value = uWAVE_UNDEFINED_FLOAT_VAL;
  _rem_azimuth_deg = uWAVE_UNDEFINED_FLOAT_VAL;

  _rem_async_in_rc = RC_INVALID;

  _pkt_tries_taken = 0;
  _pkt_target_address = 0;
  _pkt_sender_address = 0;
    
  _pkt_out_size = 0;
  _pkt_in_size = 0;
        
  _amb_pressure_mBar = uWAVE_UNDEFINED_FLOAT_VAL;
  _amb_depth_m = uWAVE_UNDEFINED_FLOAT_VAL;
  _amb_temperature_C = uWAVE_UNDEFINED_FLOAT_VAL;
  _amb_voltage_V = uWAVE_UNDEFINED_FLOAT_VAL;

  _in_buffer_idx = 0;  
  _in_buffer_isReady = false;
  _in_buffer_isStarted = false;

  _in_buffer_chk_act = 0;
  _in_buffer_chk_dcl = 0;
  _in_buffer_chk_dcl_idx = 0;
  _in_buffer_chk_present = false;
  
  _in_buffer_sign = 0;
}

uWAVE_EVENT_Enum uWAVE::process()
{
  uWAVE_EVENT_Enum result = uWAVE_NONE;

  if (_enabled)
  {
    while (_port->available())
    {
       char c = _port->read();
       if (NMEA_Process(c))
       {
         switch ((char)_in_buffer[5])
         {
            case IC_D2H_ACK:
            if (ACK_Parse())
            {
             // result |= uWAVE_ACK_RECEIVED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_ACK_RECEIVED);   
              _isWaitingLocal = false;
            }
            break;
            
            case IC_D2H_RC_RESPONSE:
            if (RC_RESPONSE_Parse())
            {
             // result |= uWAVE_REMOTE_RESPONSE_RECEIVED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_REMOTE_RESPONSE_RECEIVED);   
              _isWaitingRemote = false;
            }
            break;
  
            case IC_D2H_RC_TIMEOUT:
            if (RC_TIMEOUT_Parse())
            {
            //  result |= uWAVE_REMOTE_TIMEOUT;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_REMOTE_TIMEOUT);   
              _isWaitingRemote = false;
            }
            break;
  
            case IC_D2H_RC_ASYNC_IN:
            if (RC_ASYNC_IN_Parse())
            {
             // result |= uWAVE_ASYNC_IN_RECEIVED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_ASYNC_IN_RECEIVED);            
            }
            break;
  
            case IC_D2H_AMB_DTA:
            if (AMD_DTA_Parse())
            {
              //result |= uWAVE_AMB_DATA_RECEIVED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_AMB_DATA_RECEIVED);
            }
            break;
  
            case IC_D2H_PT_SETTINGS:
            if (PT_SETTINGS_Parse())
            {
              //result |= uWAVE_PKT_SETTINGS_RECEIVED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_SETTINGS_RECEIVED);
              _isWaitingLocal = false;
            }
            break;
  
            case IC_D2H_PT_FAILED:
            if (PT_FAILED_Parse())
            {
             // result |= uWAVE_PKT_SEND_FAILED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_SEND_FAILED); 
              _isWaitingPacketACK = false;
            }
            break;
  
            case IC_D2H_PT_DLVRD:
            if (PT_DLVRD_Parse())
            {
//              result |= uWAVE_PKT_DELIVERED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_DELIVERED);      
              _isWaitingPacketACK = false;
            }
            break;
  
            case IC_D2H_PT_RCVD:
            if (PT_RCVD_Parse())
            {
              //result |= uWAVE_PKT_RECEIVED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_RECEIVED);      
            }
            break;
  
            case IC_D2H_DINFO:            
            if (D_INFO_Parse())
            {
              //result |= uWAVE_DEVICE_INFO_RECEIVED;   
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_DEVICE_INFO_RECEIVED);         
              _isWaitingLocal = false;
              _deviceInfoUpdated = true;
            }
            break;

            case IC_D2H_PT_TMO:
            if (PT_ITG_TMO_Parse())
            {
             // result |= uWAVE_PKT_ITG_TIMEOUT;

               result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_ITG_TIMEOUT);
              _isWaitingRemote = false;              
            }
            break;
            
            case IC_D2H_PT_ITG_RESP:
            if (PT_ITG_RES_Parse())
            {
              //result |= uWAVE_PKT_ITG_RESULT_RECEIVED;
              result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_ITG_RESULT_RECEIVED);
              
              _isWaitingRemote = false;
            }
            break;
            
            default:
               break;  
         }
         
         _in_buffer_isReady = false;
       }
    }

    long ts = millis();
    if (_isWaitingLocal)
    {
      if (ts - _last_request_ts > uWAVE_LOC_TIMEOUT_MS)
      {
       // result |= uWAVE_LOCAL_TIMEOUT;
        result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_LOCAL_TIMEOUT);
        _isWaitingLocal = false;
      }
    }

    if (_isWaitingRemote)
    {
      if (ts - _rem_request_ts > uWAVE_REM_TIMEOUT_MS)
      {
       // result |= uWAVE_REMOTE_TIMEOUT;
        result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_REMOTE_TIMEOUT);
        _isWaitingRemote = false;
      }
    }

    if (_isWaitingPacketACK)
    {
      if (ts - _pkt_sent_ts > uWAVE_PKT_SEND_TIMEOUT_MS)
      {
       // result |= uWAVE_PKT_ACK_TIMEOUT;
        result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_ACK_TIMEOUT);
        _isWaitingPacketACK = false;
      }
    }

    if (_isWaitingPacketITG)
    {      
      if (ts - _pt_itg_ts > uWAVE_REM_TIMEOUT_MS)
      {
        //result |= (uWAVE_PKT_ITG_TIMEOUT);
        result = static_cast<uWAVE_EVENT_Enum>(result | uWAVE_PKT_ITG_TIMEOUT);
        _isWaitingPacketITG = false;
      }
    }
  }

  return result;
}

void uWAVE::enable()
{  
  digitalWrite(_cmd_pin, HIGH);

  _isWaitingLocal = false;
  _isWaitingRemote = false;
    
  _enabled = true;
}

void uWAVE::disable()
{
   digitalWrite(_cmd_pin, LOW);

   _isWaitingLocal = false;
   _isWaitingRemote = false;
   
   _enabled = false;
}

int uWAVE::isPktMode()
{
  return _isPktMode;
}

int uWAVE::pktModeLocalAddress()
{
  return _pktModeLocalAddress;
}

int uWAVE::isPTSPresent()
{
  return _isPTSPresent;
}

int uWAVE::getTxChID()
{
  return _txChID;
}

int uWAVE::getRxChID()
{
  return _rxChID;
}

int uWAVE::getTotalCodeChannels()
{
  return _totalCodeChannels;
}

int uWAVE::isCmdModeByDefault()
{
  return _isCmdModeByDefault;
}

int uWAVE::isACKOnTXFinished()
{
  return _isACKOnTXFinished;
}

float uWAVE::getAcousticBaudrate()
{
  return _acousticBaudrate;
}

float uWAVE::getSalinityPSU()
{
  return _salinityPSU;
}

bool uWAVE::deviceInfoUpdated()
{
  return _deviceInfoUpdated;
}

bool uWAVE::ambConfigUpdated()
{
  return _ambConfigUpdated;
}

bool uWAVE::isWaitingLocal()
{
  return _isWaitingLocal;
}

bool uWAVE::isWaitingRemote()
{
  return _isWaitingRemote;
}

bool uWAVE::isWaitingPacketACK()
{
  return _isWaitingPacketACK;
}

bool uWAVE::isWaitingPacketITG()
{
  return _isWaitingPacketITG;
}


uWAVE_ERR_CODES_Enum uWAVE::getACK_result()
{
  return _ACK_result;
}

byte uWAVE::getLast_request_sntID()
{
  return _last_request_sntID;
}

uWAVE_RC_CODES_Enum uWAVE::getRem_requested_rcID()
{
  return _rem_requested_rcID;
}

byte uWAVE::getRem_requested_txID()
{
  return _rem_requested_txID;
}

byte uWAVE::getRem_requested_rxID()
{
  return _rem_requested_rxID;
}

float uWAVE::getRem_propTime_s()
{
  return _rem_propTime_s;
}

float uWAVE::getRem_msr_dB()
{
  return _rem_msr_dB;
}

float uWAVE::getRem_value()
{
  return _rem_value;
}

float uWAVE::getRem_azimuth_deg()
{
  return _rem_azimuth_deg;
}

uWAVE_RC_CODES_Enum uWAVE::getRem_async_in_rc()
{
  return _rem_async_in_rc;
}
    
byte uWAVE::getPkt_tries_taken()
{
  return _pkt_tries_taken;
}

byte uWAVE::getPkt_target_address()
{
  return _pkt_target_address;
}

byte uWAVE::getPkt_sender_address()
{
  return _pkt_sender_address;
}

void uWAVE::getPkt_delivered(byte* data, byte* dataSize)
{
  for (int i = 0; i < _pkt_out_size; i++)
  {
    data[i] = _pkt_out_packet[i];
  }

  *dataSize = _pkt_out_size;
}

void uWAVE::getPkt_failed(byte* data, byte* dataSize)
{
for (int i = 0; i < _pkt_out_size; i++)
  {
    data[i] = _pkt_out_packet[i];
  }

  *dataSize = _pkt_out_size;
}

void uWAVE::getPkt_received(byte* data, byte* dataSize)
{
  for (int i = 0; i < _pkt_in_size; i++)
  {
    data[i] = _pkt_in_packet[i];
  }

  *dataSize = _pkt_in_size;
}

DataID_Enum uWAVE::getPkt_ITG_DataID()
{
  return _pkt_itg_dataID;
}

float uWAVE::getPkt_ITG_DataValue()
{
  return _pkt_itg_dataValue;
}




float uWAVE::getAmb_pressure_mBar()
{
  return _amb_pressure_mBar;
}

float uWAVE::getAmb_depth_m()
{
  return _amb_depth_m;
}

float uWAVE::getAmb_temperature_C()
{
  return _amb_temperature_C;
}

float uWAVE::getAmb_voltage_V()
{
  return _amb_voltage_V;    
}



bool uWAVE::queryForDeviceInfo()
{
  if (_enabled && !_isWaitingLocal)
  {
    _port->write("$PUWV?,0*27\r\n");

    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2D_DINFO_GET;
    
    return true;
  }
  else
     return false;
}

bool uWAVE::queryForSettingsUpdate(byte txChID, byte rxChID, float salinityPSU, bool isCmdModeByDefault, bool isACKOnTxFinished, float gravityAcc)
{
  if (_enabled && !_isWaitingLocal)
  {
    Str_WriterInit(  _out_buffer, &_out_buffer_idx, uWAVE_OUT_BUFFER_SIZE);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, ((byte*)"$PUWV1,\0"));
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, rxChID, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, txChID, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteFloat(  _out_buffer, &_out_buffer_idx, salinityPSU, 0, 1);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isCmdModeByDefault, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isACKOnTxFinished, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteFloat(  _out_buffer, &_out_buffer_idx, gravityAcc, 0, 4);    
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_CHK_SEP);
    Str_WriteHexByte(_out_buffer, &_out_buffer_idx, 0);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, (byte*)"\r\n");
    NMEA_CheckSum_Update(_out_buffer, _out_buffer_idx);

    _port->write(_out_buffer, _out_buffer_idx);

    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2D_SETTINGS_WRITE;    
    
    return true;
  }
  else
     return false;
}

bool uWAVE::queryForAmbientDataConfig(bool isSaveToFlash, int periodMs, bool isPressure, bool isTemperature, bool isDepth, bool isVCC)
{
  if (_enabled && !_isWaitingLocal)
  {
    Str_WriterInit(  _out_buffer, &_out_buffer_idx, uWAVE_OUT_BUFFER_SIZE);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, ((byte*)"$PUWV6,\0"));
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isSaveToFlash, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, periodMs, 0);    
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isPressure, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isTemperature, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isDepth, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isVCC, 0);    
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_CHK_SEP);
    Str_WriteHexByte(_out_buffer, &_out_buffer_idx, 0);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, (byte*)"\r\n");
    NMEA_CheckSum_Update(_out_buffer, _out_buffer_idx);

    _port->write(_out_buffer, _out_buffer_idx);

    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2D_AMB_DTA_CFG;
    
    return true;
  }
  else
     return false;
}

bool uWAVE::queryRemoteModem(byte txChID, byte rxChID, uWAVE_RC_CODES_Enum cmdID)
{
  if (_enabled && !_isWaitingLocal && !_isWaitingRemote)
  {
    Str_WriterInit(  _out_buffer, &_out_buffer_idx, uWAVE_OUT_BUFFER_SIZE);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, ((byte*)"$PUWV2,\0"));
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, txChID, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, rxChID, 0);    
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, cmdID, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_CHK_SEP);
    Str_WriteHexByte(_out_buffer, &_out_buffer_idx, 0);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, (byte*)"\r\n");
    NMEA_CheckSum_Update(_out_buffer, _out_buffer_idx);

    _port->write(_out_buffer, _out_buffer_idx);

    _rem_requested_rcID = cmdID;
    _rem_requested_txID = rxChID;
    _rem_requested_rxID = txChID;
  
    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2D_RC_REQUEST;
      
    return true;
  }
  else
    return false;
}

bool uWAVE::queryForPktModeSettings()
{
  if (_enabled && !_isWaitingLocal)
  {
    _port->write("$PUWVD,0*5C\r\n");

    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2D_PT_SETTINGS_READ;
    
    return true;
  }
  else
     return false;
}

bool uWAVE::queryForPktModeSettingsUpdate(bool isSaveToFlash, bool isPktMode, byte localAddress)
{  
  if (_enabled && !_isWaitingLocal && (localAddress < 255))
  {
    Str_WriterInit(  _out_buffer, &_out_buffer_idx, uWAVE_OUT_BUFFER_SIZE);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, ((byte*)"$PUWVF,\0"));
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isSaveToFlash, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, isPktMode, 0);    
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, localAddress, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_CHK_SEP);
    Str_WriteHexByte(_out_buffer, &_out_buffer_idx, 0);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, (byte*)"\r\n");
    NMEA_CheckSum_Update(_out_buffer, _out_buffer_idx);

    _port->write(_out_buffer, _out_buffer_idx);

    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2H_PT_SETTINGS_WRITE;
    
    return true;
  }
  else
     return false;
}

bool uWAVE::queryForPktAbortSend()
{
  if (_enabled && !_isWaitingLocal)
    {
      _port->write("$PUWVG,0,0,*6F\r\n");
  
      _isWaitingLocal = true;
      _last_request_ts = millis();
      _last_request_sntID = IC_H2D_PT_SEND;

      _isWaitingPacketACK = false; // refactor
      
      return true;
    }
    else
       return false;
}

bool uWAVE::queryForPktSend(byte targetPktAddress, byte maxTries, byte* data, byte dataSize)
{
  if (_enabled && !_isWaitingLocal && !_isWaitingRemote && (dataSize <= uWAVE_PKT_MAX_SIZE))
  {
    Str_WriterInit(  _out_buffer, &_out_buffer_idx, uWAVE_OUT_BUFFER_SIZE);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, ((byte*)"$PUWVG,\0"));
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, targetPktAddress, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, maxTries, 0);    
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);    
    Str_WriteHexStr( _out_buffer, &_out_buffer_idx, data, dataSize);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_CHK_SEP);
    Str_WriteHexByte(_out_buffer, &_out_buffer_idx, 0);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, (byte*)"\r\n");
    NMEA_CheckSum_Update(_out_buffer, _out_buffer_idx);

    _port->write(_out_buffer, _out_buffer_idx);
    
    for (int i = 0; i < dataSize; i++)
    {
      _pkt_out_packet[i] = data[i];
    }

    _pkt_out_size = dataSize;
    _pkt_target_address = targetPktAddress;    

    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2D_PT_SEND;
    
    return true;
  }
  else
     return false;
}

bool uWAVE::queryForPktITG(byte targetPktAddress, DataID_Enum dataID)
{
  if (_enabled && !_isWaitingLocal && !_isWaitingRemote && !_isWaitingPacketITG && (dataID != DID_INVALID))
  {
    Str_WriterInit(  _out_buffer, &_out_buffer_idx, uWAVE_OUT_BUFFER_SIZE);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, ((byte*)"$PUWVK,\0"));
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, targetPktAddress, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_PAR_SEP);
    Str_WriteIntDec( _out_buffer, &_out_buffer_idx, dataID, 0);
    Str_WriteByte(   _out_buffer, &_out_buffer_idx, NMEA_CHK_SEP);
    Str_WriteHexByte(_out_buffer, &_out_buffer_idx, 0);
    Str_WriteStr(    _out_buffer, &_out_buffer_idx, (byte*)"\r\n");
    NMEA_CheckSum_Update(_out_buffer, _out_buffer_idx);

    _port->write(_out_buffer, _out_buffer_idx);
     
    _pkt_target_address = targetPktAddress;
    _pkt_itg_dataID = dataID;    

    _isWaitingLocal = true;
    _last_request_ts = millis();
    _last_request_sntID = IC_H2D_PT_ITG;
    
    return true;
  }
  else
     return false;
}
