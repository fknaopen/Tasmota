/*
  xdrv_49_BLE_EQ3_TRV.ino - EQ3 radiator valve sense and control via BLE_ESP32 support for Tasmota

  Copyright (C) 2020  Simon Hailes

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  --------------------------------------------------------------------------------------------
  Version yyyymmdd  Action    Description
  --------------------------------------------------------------------------------------------
  0.0.0.0 20201213  created - initial version
*/


/*

Commands:
e.g.
trv 001A22092EE0 settemp 22.5


trv reset
trv devlist - reprot seen devices.  Active scanning required, not passive, as it looks for names
trv scan - same as devlist
trv <mac> state - report general state (see below for MQTT)
trv <mac> raw <hex to send> - send a raw command
trv <mac> on - set temp to 30 -> display ON on EQ3
trv <mac> off - set temp to 4.5 -> display OFF on EQ3
trv <mac> boost - set boost
trv <mac> unboost - turn off boost
trv <mac> lock - manual lock of physical buttons
trv <mac> unlock - manual unlock of physical buttons
trv <mac> auto - set EQ3 to auto mode
trv <mac> manual - set EQ3 to manual mode
trv <mac> eco - set EQ3 to eco mode?
trv <mac> day - set EQ3 to day temp
trv <mac> night - set EQ3 to night temp
trv <mac> settemp 20.5 - set EQ3 to temp
trv <mac> settime - set time to Tasmota time (untested)
trv <mac> settime <hex as per esp32_mqtt_eq3> - set time
trv <mac> offset 1.5 - set offset temp
trv <mac> setdaynight 22 17.5 - set day and night mode temps
trv <mac> setwindowtempdur 12.5 30 - set window open temp and duration in mins

trv <mac> reqprofile <0-6> - request a profile for a day fo the week.
trv <mac> setprofile <0-6> 20.5-07:30,17-17:00,22.5-22:00,17-24:00 (up to 7 temp-HH:MM) - set a profile for a day fo the week.

Responses:
normal:
tele/tasmota_E89E98/EQ3 = {
  "trv":"00:1a:22:09:2e:e0",
  "blestate":DONENOTIFIED, - state of the command - FAILxxx | DONExxxx
  "raw":"02010900042C", - raw response in hex
  "temp":22.0, - temp currently set (NOT measured temp)
  "posn":0, - position of the valve (0-100);
  "mode":"manual", 
  "boost":"inactive",
  "dst":"set", - daylight savings time?
  "window":"closed",
  "state":"unlocked",
  "battery":"GOOD"
}

holiday:
as above, but adds ,"holidayend":"YY-MM-DD HH:MM"

when trv <mac> reqprofile is used:
tele/tasmota_E89E98/EQ3 = {"trv":"00:1a:22:09:2e:e0","blestate":DONENOTIFIED,"raw":"02010900042C",
  "profiledayN":"20.5-07:30,17.0-17:00,22.5-22:00,17.0-24:00"}
where N is the day (0-6).

when trv <mac> setprofile is used:
tele/tasmota_E89E98/EQ3 = {"trv":"00:1a:22:09:2e:e0","blestate":DONENOTIFIED,"raw":"02010900042C","profiledayset":N}
where N is the day (0-6).

on error:
tele/tasmota_E89E98/EQ3 = {"trv":"00:1a:22:09:2e:e0","blestate":"FAIL<xxxx>","retriesremain":<1-3>}
when retries exhausted:
tele/tasmota_E89E98/EQ3 = {"trv":"00:1a:22:09:2e:e0","blestate":"FAIL<xxxx>"}

The driver will try a command three times.

*/




//#define VSCODE_DEV

#ifdef VSCODE_DEV
#define ESP32
#define USE_BLE_ESP32
#define USE_EQ3_ESP32
#endif

// for testing of BLE_ESP32, we remove xsns_62_MI_ESP32.ino completely, and instead add this modified xsns_52_ibeacon_BLE_ESP32.ino
#ifdef USE_EQ3_ESP32
#ifdef ESP32                       // ESP32 only. Use define USE_HM10 for ESP8266 support
#ifdef USE_BLE_ESP32

#define XDRV_49                    49
#define D_CMND_EQ3 "trv"

#define EQ3_DEBUG

namespace EQ3_ESP32 {

void CmndTrv(void);

const char kEQ3_Commands[] PROGMEM = D_CMND_EQ3"|"
  "";

void (*const EQ3_Commands[])(void) PROGMEM = {
  &CmndTrv };


int EQ3GenericOpCompleteFn(BLE_ESP32::generic_sensor_t *pStruct);

const char EQ3_Svc[] PROGMEM = "3e135142-654f-9090-134a-a6ff5bb77046";
const char EQ3_rw_Char[] PROGMEM = "3fa4585a-ce4a-3bad-db4b-b8df8179ea09";
const char EQ3_notify_Char[] PROGMEM = "d0e8434d-cd29-0996-af41-6c90f4e0eb2a";

struct eq3_device_tag{
  uint8_t addr[6];
  int8_t RSSI;
  uint64_t timeoutTime;
  uint8_t pairing;
} eq3_device_t;


/*********************************************************************************************\
 * variables to control operation
\*********************************************************************************************/
int retries = 0;
// allow 240s before timeout of sa device - based on that we restart BLE if we don't see adverts for 120s
#define EQ3_TIMEOUT 240L

uint8_t pairingaddr[6] = {0,0,0,0,0,0};
uint8_t pairing = 0;

#define EQ3_NUM_DEVICESLOTS 8
eq3_device_tag EQ3Devices[EQ3_NUM_DEVICESLOTS];
void *EQ3mutex = nullptr;


/*********************************************************************************************\
 * Functions
\*********************************************************************************************/

bool EQ3Operation(const uint8_t *MAC, const uint8_t *data = nullptr, int datalen = 0, int retries_in = 0) {
  BLE_ESP32::generic_sensor_t *op = nullptr;

  // ALWAYS use this function to create a new one.
  int res = BLE_ESP32::newOperation(&op);
  if (!res){
    AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3:Can't get a newOperation from BLE"));
    retries = 0;
    return 0;
  } else {
#ifdef EQ3_DEBUG
    AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3:got a newOperation from BLE"));
#endif
  }

  NimBLEAddress addr((uint8_t *)MAC);
  op->addr = addr;

  bool havechar = false;
  op->serviceUUID = NimBLEUUID(EQ3_Svc);
  op->characteristicUUID = NimBLEUUID(EQ3_rw_Char);
  op->notificationCharacteristicUUID = NimBLEUUID(EQ3_notify_Char);

  if (data && datalen) {
    op->writelen = datalen;
    memcpy(op->dataToWrite, data, datalen);
  } else {
    op->writelen = 1;
    op->dataToWrite[0] = 0x03; // just request status
  }

  // this op will call us back on complete or failure.
  op->completecallback = (void *)EQ3GenericOpCompleteFn;
  op->context = nullptr;

  res = BLE_ESP32::extQueueOperation(&op);
  if (!res){
    // if it fails to add to the queue, do please delete it
    BLE_ESP32::freeOperation(&op);
    AddLog_P(LOG_LEVEL_ERROR,PSTR("Failed to queue new operation - deleted"));
    retries = 0;
  } else {
    if (retries_in){
      retries = retries_in;
    }
  }

  return res;
}


int EQ3ParseOp(BLE_ESP32::generic_sensor_t *op, bool success, int retries){
  int res = 0;
  char *p = TasmotaGlobal.mqtt_data;
  int maxlen = sizeof(TasmotaGlobal.mqtt_data);

  strcpy(p, "{\"trv\":");
  p += strlen(p);
  NimBLEAddress addr = NimBLEAddress(op->addr);

  *(p++) = '\"';
  strcpy(p, addr.toString().c_str());
  p += strlen(p);
  *(p++) = '\"';

  if (success) {
    sprintf(p, ",\"blestate\":\"%s\"", BLE_ESP32::getStateString(op->state));
    p += strlen(p);

    char raw[40];
    BLE_ESP32::dump(raw, 40, op->dataNotify, op->notifylen);
    sprintf(p, ",\"raw\":\"%s\"", raw);
    p += strlen(p);

    if ((op->notifylen >= 6) && (op->dataNotify[0] == 2) && (op->dataNotify[1] == 1)){

      sprintf(p, ",\"temp\":%2.1f", ((float)op->dataNotify[5])/2);
      p += strlen(p);

      sprintf(p, ",\"posn\":%d", op->dataNotify[3]);
      p += strlen(p);

      int stat = op->dataNotify[2];
      sprintf(p, ",\"mode\":");
      p += strlen(p);

      switch (stat & 3){
        case 0:
          sprintf(p, "\"auto\"");
          break;
        case 1:
          sprintf(p, "\"manual\"");
          break;
        case 2:
          sprintf(p, "\"holiday\"");
          break;
        case 3:
          sprintf(p, "\"manualholiday\"");
          break;
      }
      p += strlen(p);

      sprintf(p, ",\"boost\":\"%s\"", (stat & 4)?"active":"inactive");
      p += strlen(p);

      sprintf(p, ",\"dst\":\"%s\"", (stat & 8)?"set":"unset");
      p += strlen(p);

      sprintf(p, ",\"window\":\"%s\"", (stat & 16)?"open":"closed");
      p += strlen(p);

      sprintf(p, ",\"state\":\"%s\"", (stat & 32)?"locked":"unlocked");
      p += strlen(p);

      sprintf(p, ",\"battery\":\"%s\"", (stat & 128)?"LOW":"GOOD");
      p += strlen(p); 
    }
    if ((op->notifylen >= 10) && (op->dataNotify[0] == 2) && (op->dataNotify[1] == 1)){
      int mm = op->dataNotify[8] * 30;
      int hh = mm/60;
      mm = mm % 60; 
      sprintf(p, ",\"holidayend\":\"%02d-%02d-%02d %02d:%02d\"", 
        op->dataNotify[7], 
        op->dataNotify[9], 
        op->dataNotify[6],
        hh, mm
        );
      p += strlen(p); 
    }
    if ((op->notifylen >= 3) && (op->dataNotify[0] == 2) && (op->dataNotify[1] == 2)){
      sprintf(p, ",\"profiledayset\":%d", op->dataNotify[2]);
      p += strlen(p); 
    }
    if ((op->notifylen >= 16) && (op->dataNotify[0] == 0x21)){
//YY is the time, coded as (minutes/10), up to which to maintain the temperature declared in XX
//XX represents the temperature to be maintained until then, codified as (temperature*2)
// byte 0: 21 (default value)
// byte 1: 02 (Monday = 0x02)
// byte (2,3): 22 24 (17°C up to 06:00)
// byte (4,5): 2A 36 (21°C up to 09:00)
// byte (6,7): 22 66 (17°C up to 17:00)
// byte (8,9): 2A 8A (21°C up to 23:00)
// byte (10,11): 22 90 (17°C up to 24:00)
// byte (12,13): 22 90 (unused)
// byte (14,15): 22 90 (unused)      
      sprintf(p, ",\"profileday%d\":\"", op->dataNotify[1]);
      p += strlen(p);
      uint8_t *data = op->dataNotify + 2;
      for (int i = 0; i < 7; i++){
        float t = *(data++);
        t /= 2;
        int mm = *(data++);
        mm *= 10;
        int hh = mm / 60;
        mm = mm % 60;
        sprintf(p, "%2.1f-%02d:%02d", t, hh, mm);
        p += strlen(p);
        if (i < 6){
          *(p++) = ',';
        }
      }
      *(p++) = '\"';
    }

    res = 1;
  } else {
    if (retries){
      sprintf(p, ",\"blestate\":\"%s\"", BLE_ESP32::getStateString(op->state));
      p += strlen(p);
      sprintf(p, ",\"retriesremain\":%d", retries);
      p += strlen(p);
    } else {
      sprintf(p, ",\"blestate\":\"%s\"", BLE_ESP32::getStateString(op->state));
      p += strlen(p);
    }
  }
  *(p++) = '}';
  *(p++) = 0;

  uint8_t addrev[6];
  const uint8_t *native = op->addr.getNative();
  memcpy(addrev, native, 6);
  BLE_ESP32::ReverseMAC(addrev);

  char addrstr[4+8*2+2] = "EQ3/";
  BLE_ESP32::dump(&addrstr[4], 8*2+2, addrev, 6);
  MqttPublishPrefixTopic_P(STAT, addrstr, false);

  return res;
}

int EQ3GenericOpCompleteFn(BLE_ESP32::generic_sensor_t *op){
  uint32_t context = (uint32_t) op->context;

  if (op->state <= GEN_STATE_FAILED){
    if (retries > 1){
      retries--;
      uint8_t addrev[6];
      const uint8_t *native = op->addr.getNative();
      memcpy(addrev, native, 6);
      BLE_ESP32::ReverseMAC(addrev);

      EQ3Operation(addrev, op->dataToWrite, op->writelen);
      EQ3ParseOp(op, false, retries);
      AddLog_P(LOG_LEVEL_ERROR,PSTR("trv operation failed - retrying %d"), op->state);
    } else {
      retries = 0;
      EQ3ParseOp(op, false, 0);
      AddLog_P(LOG_LEVEL_ERROR,PSTR("trv operation failed - no more retries %d"), op->state);
    }
    return 0;
  }

  retries = 0;

  EQ3ParseOp(op, true, 0);
  return 0;
}



/*********************************************************************************************\
 * Functons actualy called from within the BLE task
\*********************************************************************************************/

int ispairing2(const uint8_t *payload, int len, char *name, int namelen, char *serial, int seriallen ){
  while (len){
    int l = *payload;
    //BLE_ESP32::dump(temp, 40, payload, l+1);
    //AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3: %s"), temp);

    payload++;
    len--;
    if (len < l){
      //AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3: part len er %d<%d"),len, l);
      return 0;
    }
    switch (*payload){
      case 0xff: {// parse the EQ3 advert payload looking for nnFF01ssssssss
        payload++;
        len--;
        l--;
        if (*payload == 1){
          payload++;
          len--;
          l--;
          //char serialstr[20];
          //strncpy(serialstr, (const char *)payload, l);
          //AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3: adv part FF01 detected %s"), serialstr);
          // we don;t use these, but that's what they seem to be....
          uint8_t copylen = (l > seriallen)?seriallen:l;
          strncpy(serial, (const char *)payload, copylen);
          payload += l;
          len -= l;
          return 1;
        } else {
          payload += l;
          len -= l;
        }
      } break;
      case 0x09: {
        payload++;
        len--;
        l--;
        if (*payload == 1){
          payload++;
          len--;
          l--;
          //char serialstr[20];
          //strncpy(serialstr, (const char *)payload, l);
          //AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3: adv part FF01 detected %s"), serialstr);
          // we don;t use these, but that's what they seem to be....
          uint8_t copylen = (l > namelen)?seriallen:l;
          strncpy(name, (const char *)payload, copylen);
          payload += l;
          len -= l;
          return 1;
        } else {
          payload += l;
          len -= l;
        }
      } break;
      default:{
        payload += l;
        len -= l;
      } break;
    }
  }
  return 0;
}

int ispairing(const uint8_t *payload, int len){
  //char temp[40];
  //BLE_ESP32::dump(temp, 40, payload, len);
  //AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3: pair%d %s"), len, temp);
  while (len){
    int l = *payload;
    //BLE_ESP32::dump(temp, 40, payload, l+1);
    //AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3: %s"), temp);

    payload++;
    len--;
    if (len < l){
      //AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3: part len er %d<%d"),len, l);
      return 0;
    }
    if (*payload == 0xff){
      payload++;
      len--;
      l--;
      if (*payload == 1){
        payload++;
        len--;
        l--;
        //char serialstr[20];
        //strncpy(serialstr, (const char *)payload, l);
        //AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3: adv part FF01 detected %s"), serialstr);
        // we don;t use these, but that's what they seem to be....
        const uint8_t *serial = payload;
        uint8_t seriallen = l;
        payload += l;
        len -= l;
        return 1;
      } else {
        payload += l;
        len -= l;
      }
    } else {
      payload += l;
      len -= l;
    }
  }
  return 0;
}

int TaskEQ3AddDevice(int8_t RSSI, const uint8_t* addr, int pairingin){
  int free = -1;
  int i = 0;
  uint64_t now = esp_timer_get_time();

  for(i = 0; i < EQ3_NUM_DEVICESLOTS; i++){
    if(memcmp(addr,EQ3Devices[i].addr,6)==0){
      break;
    }
    if (EQ3Devices[i].timeoutTime && (EQ3Devices[i].timeoutTime < now)) {
#ifdef EQ3_DEBUG
    AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3 timeout at %d"), i);
#endif
      EQ3Devices[i].timeoutTime = 0L;
    }
    if (!EQ3Devices[i].timeoutTime){
      if (free == -1){
        free = i;
      }
    }
  }

  if (i == EQ3_NUM_DEVICESLOTS){
    if (free >= 0){
      i = free;
    } else {
      AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3: > %d devices"), EQ3_NUM_DEVICESLOTS);
      return 0;
    }
  }

#ifdef EQ3_DEBUG
  if (!EQ3Devices[i].timeoutTime)
    AddLog_P(LOG_LEVEL_INFO,PSTR("EQ3 added at %d"), i);
#endif

  EQ3Devices[i].timeoutTime = now + (1000L*1000L)*EQ3_TIMEOUT;
  memcpy(EQ3Devices[i].addr, addr, 6);
  EQ3Devices[i].RSSI = RSSI;

  EQ3Devices[i].pairing = pairingin;

  if (EQ3Devices[i].pairing){
    memcpy(pairingaddr, addr, 6);
    pairing = 1;
  }

  return 1;
}


const char *EQ3Names[] = {
  "CC-RT-BLE",
  "CC-RT-BLE-EQ",
  "CC-RT-M-BLE"
};

int TaskEQ3advertismentCallback(BLE_ESP32::ble_advertisment_t *pStruct)
{
  // we will try not to use this...
  BLEAdvertisedDevice *advertisedDevice = pStruct->advertisedDevice;

  std::string sname = advertisedDevice->getName();

  bool found = false;
  const char *nameStr = sname.c_str();

  for (int i = 0; i < sizeof(EQ3Names)/sizeof(*EQ3Names); i++){
    if (!strcmp(nameStr, EQ3Names[i])){
      found = true;
      break;
    }
  }
  if (!found) return 0;

  int8_t RSSI = pStruct->RSSI;
  const uint8_t *addr = pStruct->addr;
#ifdef EQ3_DEBUG
  if (BLE_ESP32::BLEDebugMode > 0) AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3Device: saw %s"),advertisedDevice->getAddress().toString().c_str());
#endif

  uint8_t* payload = advertisedDevice->getPayload();
  size_t payloadlen = advertisedDevice->getPayloadLength();

  char name[20] = {0};
  char serial[20] = {0};
  int pairing = 0;
  ispairing2(payload, payloadlen, name, 20, serial, 20);

  // this will take and keep the mutex until the function is over
  TasAutoMutex localmutex(&EQ3mutex);
  TaskEQ3AddDevice(RSSI, addr, (serial[0] != 0));
  return 0;
}




/*********************************************************************************************\
 * Helper functions
\*********************************************************************************************/



/*********************************************************************************************\
 * init
\*********************************************************************************************/
void EQ3Init(void) {
  memset(&EQ3Devices, 0, sizeof(EQ3Devices));
  BLE_ESP32::registerForAdvertismentCallbacks((const char *)"EQ3", TaskEQ3advertismentCallback);
#ifdef EQ3_DEBUG
  AddLog_P(LOG_LEVEL_INFO,PSTR("EQ3: init: request callbacks"));
#endif
  return;
}

/***********************************************************************\
 * Regular
\***********************************************************************/

void EQ3Every50mSecond(){

}

/**
 * @brief Main loop of the driver, "high level"-loop
 *
 */

void EQ3EverySecond(bool restart){
  if (pairing){
    pairing = 0;
    char *p = TasmotaGlobal.mqtt_data;
    int maxlen = sizeof(TasmotaGlobal.mqtt_data);

    strcpy(p, "{\"pairing\":");
    p += strlen(p);
    *(p++) = '\"';
    BLE_ESP32::dump(p, 20, pairingaddr, 6);
    p += strlen(p);
    *(p++) = '\"';
    *(p++) = '}';
    *(p++) = 0;

    char addrstr[4+8*2+2] = "EQ3/";
    BLE_ESP32::dump(&addrstr[4], 8*2+2, pairingaddr, 6);
    MqttPublishPrefixTopic_P(STAT, addrstr, false);
  }
}


/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/
int EQ3SendCurrentDevices(){
  // send the active devices

  char *p = TasmotaGlobal.mqtt_data;
  int maxlen = sizeof(TasmotaGlobal.mqtt_data);

  strcpy(p, "{\"devices\":{");
  p += strlen(p);

  int added = 0;
  for(int i = 0; i < EQ3_NUM_DEVICESLOTS; i++){
    if (!EQ3Devices[i].timeoutTime)
      continue;
    if (added){
      *(p++) = ',';
    }
    *(p++) = '\"';
    BLE_ESP32::dump(p, 20, EQ3Devices[i].addr, 6);
    p += strlen(p);
    *(p++) = '\"';
    *(p++) = ':';
    sprintf(p, "%d", EQ3Devices[i].RSSI);
    p += strlen(p);
    added = 1;
  }
  *(p++) = '}';
  *(p++) = '}';
  *(p++) = 0;

  MqttPublishPrefixTopic_P(STAT, PSTR("EQ3"), false);

  return 0;
}

int EQ3SendResult(uint8_t *addr, const char *result){
  // send the active devices
  char *p = TasmotaGlobal.mqtt_data;
  int maxlen = sizeof(TasmotaGlobal.mqtt_data);

  strcpy(p, "{\"result\":\"");
  p += strlen(p);

  sprintf(p, "%s", result);
  p += strlen(p);
  *(p++) = '\"';
  *(p++) = '}';
  *(p++) = 0;

  char addrstr[4+8*2+2] = "EQ3/";
  BLE_ESP32::dump(&addrstr[4], 8*2+2, addr, 6);
  MqttPublishPrefixTopic_P(STAT, addrstr, false);

  return 0;
}


/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void simpletolower(char *p){
  if (!p) return;
  while (*p){
    *p = *p | 0x20;
    p++;
  }
}

//
// great description here:
// https://reverse-engineering-ble-devices.readthedocs.io/en/latest/protocol_description/00_protocol_description.html
// not all implemented yet.
//
int EQ3Send(const uint8_t* addr, char *cmd, char* param, char* param2){

  char p[] = "";
  if (!param) param = p;
  if (!param2) param2 = p;
#ifdef EQ3_DEBUG
  AddLog_P(LOG_LEVEL_INFO,PSTR("EQ3 cmd: [%s] [%s] [%s]"), cmd, param, param2);
#endif
  uint8_t d[20];
  int dlen = 0;

/* done on whole string before here.
  simpletolower(cmd);
  simpletolower(param);
  simpletolower(param2);
*/

  do {
    if (!strcmp(cmd, "raw")){
      if (!param || param[0] == 0){
        int len = strlen(param) / 2;
        if (len > 20){
          return -1;
        }
        BLE_ESP32::fromHex(d, param, len);
        dlen = len;
      }
      break;
    }

    if (!strcmp(cmd, "state")){
      d[0] = 0x03;
      dlen = 1;
      break;
    }

    if (!strcmp(cmd, "settime")){
      if (!param || param[0] == 0){
        time_t now = 0;
        struct tm timeinfo = { 0 };
        time(&now);
        localtime_r(&now, &timeinfo);
        d[0] = 0x03;
        d[1] = timeinfo.tm_year % 100;
        d[2] = timeinfo.tm_mon + 1;
        d[3] = timeinfo.tm_mday;
        d[4] = timeinfo.tm_hour;
        d[5] = timeinfo.tm_min;
        d[6] = timeinfo.tm_sec;
      } else {
        BLE_ESP32::fromHex(d+1, param, 6);
      }
      dlen = 7;
      break;
    }

    if (!strcmp(cmd, "settemp")){
      if (!param || param[0] == 0){
        return -1;
      }
      float ftemp = 20;
      sscanf(param, "%f", &ftemp);
      if (ftemp < 5) ftemp = 5;
      ftemp *= 2;
      uint8_t ctemp = (uint8_t) ftemp;
      d[0] = 0x41; d[1] = ctemp; dlen = 2;
      break; 
    }

    if (!strcmp(cmd, "offset")){
      if (!param || param[0] == 0){
        return 0;
      }
      float ftemp = 20;
      sscanf(param, "%f", &ftemp);
      ftemp *= 2;
      int8_t ctemp = (int8_t) ftemp;
      ctemp += 7;
      d[0] = 0x13; d[1] = ctemp; dlen = 2;
      break; 
    }

    if (!strcmp(cmd, "setdaynight")){
      if (!param || param[0] == 0){
        return -1;
      }
      if (!param2 || param2[0] == 0){
        return -1;
      }
      float ftemp = 15;
      sscanf(param, "%f", &ftemp);
      if (ftemp < 5) ftemp = 5;
      ftemp *= 2;
      uint8_t dtemp = (uint8_t) ftemp;

      ftemp = 20;
      sscanf(param2, "%f", &ftemp);
      if (ftemp < 5) ftemp = 5;
      ftemp *= 2;
      uint8_t ntemp = (uint8_t) ftemp;

      d[0] = 0x11; d[1] = dtemp; d[1] = ntemp; dlen = 3;
      break; 
    }

    if (!strcmp(cmd, "setwindowtempdur")){
      if (!param || param[0] == 0){
        return -1;
      }
      if (!param2 || param2[0] == 0){
        return -1;
      }
      float ftemp = 15;
      sscanf(param, "%f", &ftemp);
      if (ftemp < 5) ftemp = 5;
      ftemp *= 2;
      uint8_t temp = (uint8_t) ftemp;

      int dur = 0;
      sscanf(param2, "%d", &dur);
      d[0] = 0x14; d[1] = temp; d[1] = (dur/5); dlen = 3;
      break; 
    }

    if (!strcmp(cmd, "boost"))    { 
      d[0] = 0x45; d[1] = 0x01; 
      if (param && (!strcmp(param, "off") || param[0] == '0')){
        d[1] = 0x00;
      }
      dlen = 2; break; 
    }
    if (!strcmp(cmd, "unboost"))  { d[0] = 0x45; d[1] = 0x00; dlen = 2; break; }
    if (!strcmp(cmd, "lock"))     { d[0] = 0x80; d[1] = 0x01; 
      if (param && (!strcmp(param, "off") || param[0] == '0')){
        d[1] = 0x00;
      }
      dlen = 2; break; 
    }
    if (!strcmp(cmd, "unlock"))   { d[0] = 0x80; d[1] = 0x00; dlen = 2; break; }
    if (!strcmp(cmd, "auto"))     { d[0] = 0x40; d[1] = 0x00; dlen = 2; break; }
    if (!strcmp(cmd, "manual"))   { d[0] = 0x40; d[1] = 0x40; dlen = 2; break; }
    if (!strcmp(cmd, "eco"))      { d[0] = 0x40; d[1] = 0x80; dlen = 2; break; }
    if (!strcmp(cmd, "on"))       { d[0] = 0x41; d[1] = 0x3c; dlen = 2; break; }
    if (!strcmp(cmd, "off"))      { d[0] = 0x41; d[1] = 0x09; dlen = 2; break; }
    if (!strcmp(cmd, "valve"))     { d[0] = 0x41; d[1] = 0x3c; 
      if (!param || param[0] == 0){
        return -1;
      }
      if ((!strcmp(param, "off") || param[0] == '0')){
        d[1] = 0x09;
      }
      dlen = 2; break; 
    }
    if (!strcmp(cmd, "mode"))     { d[0] = 0x40; d[1] = 0x00; 
      if (!param || param[0] == 0){
        return -1;
      }
      if (!strcmp(param, "auto")){
        d[1] = 0x00;
      }
      if (!strcmp(param, "manual")){
        d[1] = 0x40;
      }
      if (!strcmp(param, "eco")){
        d[1] = 0x80;
      }
      dlen = 2; break; 
    }
    if (!strcmp(cmd, "day"))      { d[0] = 0x43; dlen = 1; break; }
    if (!strcmp(cmd, "night"))    { d[0] = 0x44; dlen = 1; break; }

    if (!strcmp(cmd, "reqprofile"))     { 
      if (!param || param[0] == 0){
        return -1;
      }
      d[0] = 0x20; d[1] = atoi(param); dlen = 2; 
      break; 
    }

    if (!strcmp(cmd, "setprofile"))     { 
      if (!param || param[0] == 0){
        return -1;
      }
      if (!param2 || param2[0] == 0){
        return -1;
      }
      d[0] = 0x10; d[1] = atoi(param); 
      
      // default
      uint8_t temps[7] = {0x22,0x22,0x22,0x22,0x22,0x22,0x22};
      uint8_t times[7] = {0x90,0x90,0x90,0x90,0x90,0x90,0x90};

      // 20.5-17:30,
      const char *p = strtok(param2, ",");
      int i = 0;
      while (p){
        float t = 17;
        int mm = 0;
        int hh = 24;
        sscanf(p, "%f-%d:%d", &t, &hh, &mm);
        t *= 2;
        temps[i] = (uint8_t) t;
        int time = hh*60+mm;
        time = time / 10;
        times[i] = time;
        p = strtok(nullptr, ",");
        i++;
        if (i >= 7) break;
      }

      // remaining left at 00 00
      for (int j = 0; j < 7; j++){
        d[2+j*2] = temps[j];
        d[2+j*2+1] = times[j];
      }
      
      dlen = 2+14; 
      break; 
    }

    break;
  } while(0);

  if (dlen){
    return EQ3Operation(addr, d, dlen, 4);
  }

  return -1;
}


const char *responses[] = {
  PSTR("Done"),
  PSTR("queued"),
  PSTR("ignoredbusy"),
  PSTR("invcmd"),
  PSTR("cmdfail"),
  PSTR("invidx")
};


int CmndTrvNext(int index, char *data){
  AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3 cmd index: %d"), index);
  //simpletolower(data);

  switch(index){
    case 0: 
    case 1: {

      char *p = strtok(data, " ");
      bool trigger = false;
      if (!strcmp(p, "reset")){
        retries = 0;
        for (int i = 0; i < EQ3_NUM_DEVICESLOTS; i++){
          EQ3Devices[i].timeoutTime = 0L;
        }
        return 0;
      }

      if (!strcmp(p, "scan")){
#ifdef EQ3_DEBUG
        AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3 cmd: %s"), p);
#endif
        EQ3SendCurrentDevices();
        return 0;
      }
      if (!strcmp(p, "devlist")){
#ifdef EQ3_DEBUG
        AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3 cmd: %s"), p);
#endif
        EQ3SendCurrentDevices();
        return 0;
      }

      // only allow one command in progress
      if (retries){
        return 2;
      }


      uint8_t addrbin[6];
      int addrres = BLE_ESP32::getAddr(addrbin, p);
      if (addrres){
        if (addrres == 2){
          AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3 addr used alias: %s"), p);
        }
        NimBLEAddress addr(addrbin);

#ifdef EQ3_DEBUG
        //AddLog_P(LOG_LEVEL_INFO,PSTR("EQ3 cmd addr: %s -> %s"), p, addr.toString().c_str());
#endif
      } else {
        AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3 addr invalid: %s"), p);
        return 3;
      }

      // get next part of cmd
      char *cmd = strtok(nullptr, " ");
      if (!cmd){
        return 3;
      }

      char *param = strtok(nullptr, " ");
      char *param2 = nullptr;
      if (param){
        param2 = strtok(nullptr, " ");
      }
      int res = EQ3Send(addrbin, cmd, param, param2);
      if (res > 0) {
        // succeeded to queue
        AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3 queued"));
        return 1;
      }
      
      if (res < 0) { // invalid in some way
        AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3 invalid"));
        return 3;
      }

      AddLog_P(LOG_LEVEL_ERROR,PSTR("EQ3 failed to queue"));
      // failed to queue
      return 4;
    } break;

    case 2:
      retries = 0;
      return 0;
      break;
  }

  return 4;
}

void CmndTrv(void) {
  int res = CmndTrvNext(XdrvMailbox.index, XdrvMailbox.data);
  ResponseCmndChar(responses[res]);
}

#define EQ3_TOPIC "EQ3"
static char tmp[120];

bool mqtt_direct(){
  char stopic[TOPSZ];
  strncpy(stopic, XdrvMailbox.topic, TOPSZ);

  //AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3 mqtt: %s:%s"), stopic, XdrvMailbox.data);

  char *items[10];
  char *p = stopic;
  int cnt = 0;
  do {
    items[cnt] = strtok(p, "/");
    cnt++;
    p = nullptr;
  } while (items[cnt-1]);
  cnt--; // repreents the number of items

  if (cnt < 4){ // not for us?
    //AddLog_P(LOG_LEVEL_INFO,PSTR("cnt: %d < 4"), cnt);
    return false;
  }

  for (int i = 0; i < cnt; i++){
    //AddLog_P(LOG_LEVEL_INFO,PSTR("cnt %d:%s"), i, items[i]);
  }


  int EQ3index = 0;
  int MACindex = 0;
  int CMDindex = 0;
  if (strncasecmp_P(items[cnt-3], PSTR(EQ3_TOPIC), strlen(EQ3_TOPIC)) != 0) {
    //AddLog_P(LOG_LEVEL_INFO,PSTR("cnt-3 not %s"), PSTR(EQ3_TOPIC));
    if (strncasecmp_P(items[cnt-2], PSTR(EQ3_TOPIC), strlen(EQ3_TOPIC)) != 0) {
      //AddLog_P(LOG_LEVEL_INFO,PSTR("cnt-2 not %s"), PSTR(EQ3_TOPIC));
      return false; // not for us
    } else {
      EQ3index = cnt-2;
      MACindex = cnt-1;
    }
  } else {
    EQ3index = cnt-3;
    MACindex = cnt-2;
    CMDindex = cnt-1;
  }

  int remains = 120;
  p = tmp;
  strncpy(p, items[MACindex], remains-6);
  p += strlen(p);
  *(p++) = 0x20;
  remains = 120 - (p-tmp);

  if (CMDindex){
    strncpy(p, items[CMDindex], remains-6);
    p += strlen(p);
    *(p++) = 0x20;
    remains = 120 - (p-tmp);
  }

  strncpy(p, XdrvMailbox.data, remains-6);
  p += strlen(p);
  *(p++) = 0x20;
  remains = 120 - (p-tmp);
  *(p++) = 0;

  AddLog_P(LOG_LEVEL_DEBUG,PSTR("EQ3:mqtt->cmdstr %s"), tmp);


  int res = CmndTrvNext(1, tmp);
  uint8_t addrbin[6];
  BLE_ESP32::fromHex(addrbin, items[MACindex], sizeof(addrbin));

  // post result to stat/tas/EQ3/<MAC> {"result":"<string>"}
  EQ3SendResult(addrbin, responses[res]);

  return true;
}

} // end namespace XDRV_49

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv49(uint8_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      EQ3_ESP32::EQ3Init();
      break;
    case FUNC_EVERY_50_MSECOND:
      EQ3_ESP32::EQ3Every50mSecond();
      break;
    case FUNC_EVERY_SECOND:
      EQ3_ESP32::EQ3EverySecond(false);
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(EQ3_ESP32::kEQ3_Commands, EQ3_ESP32::EQ3_Commands);
      break;
    case FUNC_MQTT_DATA:
      //AddLog_P(LOG_LEVEL_INFO,PSTR("topic %s"), XdrvMailbox.topic);
      result = EQ3_ESP32::mqtt_direct();
      break;
    case FUNC_JSON_APPEND:
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      break;
#endif  // USE_WEBSERVER
    }
  return result;
}
#endif  // 
#endif  // ESP32

#endif