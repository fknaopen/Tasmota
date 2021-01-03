/*
  xsns_81_sniffer.ino - Test wifi sniffing

  Copyright (C) 2021  Simon Hailes

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
*/

#ifdef USE_WIFI_SNIFFER

#define XSNS_81 81
#ifdef ESP32    
  #include "esp_wifi.h"
  #include "esp_wifi_types.h"
#endif
#include <vector>
#include <deque>
#include <string.h>
#include <cstdarg>



namespace XSNS81 {

  struct SNIFF_simple_device_t {
    uint8_t mac[6];
    int8_t RSSI;
    int8_t publishedRSSI;
    uint8_t secondsCounter;
    uint64_t lastseen;  // last seen us
    uint16_t maxAge;    // maximum observed age of this device
  };

  
  // seen devices
  #define MAX_XSNS81_DEVICES_LOGGED 40
  std::deque<SNIFF_simple_device_t*> seenDevices;
  std::deque<SNIFF_simple_device_t*> freeDevices;
#ifdef ESP32    
  SemaphoreHandle_t  DevicesMutex;
#endif
  int MaxAge = 60;
  int SnifMQTTInterval = 30;
  int DeviceLimitReached = 0;
  int MinRSSI = -70;
  int SnifRSSIChange = 10;


  void sniff_mqtt(XSNS81::SNIFF_simple_device_t* dev, int lost);


  int addSeenDevice(const uint8_t *mac, int8_t RSSI){

    int res = 0;
    uint64_t now = millis();
#ifdef ESP32    
    TasAutoMutex localmutex(&DevicesMutex, "DevAdd");
#endif
    int devicefound = 0;
    // do we already know this device?
    for (int i = 0; i < seenDevices.size(); i++){
      if (!memcmp(seenDevices[i]->mac, mac, 6)){
        seenDevices[i]->lastseen = now; 
        seenDevices[i]->RSSI = RSSI;
        devicefound = 1;
        break;
      }
    }
    if (!devicefound){
      // new device and RSSI too low, ignore.
      if (RSSI < MinRSSI) return 0;

      // if no free slots, add one if we have not reached our limit
      if (!freeDevices.size()){
        int total = seenDevices.size();
        if (total < MAX_XSNS81_DEVICES_LOGGED){
          AddLog_P(LOG_LEVEL_INFO,PSTR("new seendev slot %d"), total);
          XSNS81::SNIFF_simple_device_t* dev = new XSNS81::SNIFF_simple_device_t;
          freeDevices.push_back(dev);
        } else {
          // flag we hit the limit
          DeviceLimitReached ++;
          if (DeviceLimitReached >= 254){
            DeviceLimitReached = 254;
          }
        }
      }

      // get a new device from the free list
      if (freeDevices.size()){
        XSNS81::SNIFF_simple_device_t* dev = freeDevices[0];
        freeDevices.erase(freeDevices.begin());
        memcpy(dev->mac, mac, 6);
        dev->lastseen = now; 
        dev->RSSI = RSSI;
        dev->maxAge = 1;
        dev->secondsCounter = 0;
        char addr[20];
        ToHex_P(dev->mac, 6, addr, 20, 0);
        //const char *alias = BLE_ESP32::getAlias(dev->mac);
        AddLog_P(LOG_LEVEL_INFO,PSTR("SNIFF add device %s."), 
          addr);
        seenDevices.push_back(dev);
        res = 2; // added
      }
    } else {
      res = 1; // already there
    }
    return res;
  }

  // remove devices from the seen list by age, and add them to the free list
  // set ageS to 0 to delete all...
  int deleteSeenDevices(int ageS = 0){
    int res = 0;
    uint64_t now = millis();
    now = now/1000L;
    uint32_t nowS = (uint32_t)now;   
    uint32_t mintime = nowS - ageS;

    {
#ifdef ESP32    
      TasAutoMutex localmutex(&DevicesMutex, "DevDel");
#endif
      for (int i = seenDevices.size()-1; i >= 0; i--){
          XSNS81::SNIFF_simple_device_t* dev = seenDevices[i];
          uint64_t lastseen = dev->lastseen/1000L;
          uint32_t lastseenS = (uint32_t) lastseen; 
          uint32_t del_at = lastseenS + ageS;
          uint32_t devAge = nowS - lastseenS;
          if (dev->maxAge < devAge){
            dev->maxAge = devAge;
          }

          if ((del_at < nowS) || (ageS == 0) || (dev->RSSI < MinRSSI)) {
            char addr[20];
            ToHex_P(dev->mac, 6, addr, 20, 0);
            //const char *alias = BLE_ESP32::getAlias(dev->mac);
            if (dev->RSSI < MinRSSI) {
              AddLog_P(LOG_LEVEL_INFO,PSTR("SNIFF delete device %s by RSSI %d < %d"), 
                addr, dev->RSSI, MinRSSI);
            } else {
              AddLog_P(LOG_LEVEL_INFO,PSTR("SNIFF delete device %s by age lastseen %u + maxage %u < now %u."), 
                addr, lastseenS, ageS, nowS);
            }
            seenDevices.erase(seenDevices.begin()+i);
            freeDevices.push_back(dev);
#ifdef ESP32    
            localmutex.give();
#endif
            sniff_mqtt(dev, 1);
            res++;
            break; // delete one at a time only because of mutex release.
          }
      }
    }
    return res;
  }

  int deleteSeenDevice(uint8_t *mac){
    int res = 0;
#ifdef ESP32    
    TasAutoMutex localmutex(&DevicesMutex, "DevDel2");
#endif
    for (int i = 0; i < seenDevices.size(); i++){
      if (!memcmp(seenDevices[i]->mac, mac, 6)){
        XSNS81::SNIFF_simple_device_t* dev = seenDevices[i];
        seenDevices.erase(seenDevices.begin()+i);
        freeDevices.push_back(dev);
#ifdef ESP32    
        localmutex.give();
#endif
        sniff_mqtt(dev, 1);
        res = 1;
        break;
      }
    }
    return res;
  }


  void checkDeviceTimouts(){
    if (MaxAge){
      deleteSeenDevices(MaxAge);
    }
  }

#ifdef ESP32

  typedef struct {
    unsigned frame_ctrl:16;
    unsigned duration_id:16;
    uint8_t addr1[6]; /* receiver address */
    uint8_t addr2[6]; /* sender address */
    uint8_t addr3[6]; /* filtering address */
    unsigned sequence_ctrl:16;
    uint8_t addr4[6]; /* optional */
  } wifi_ieee80211_mac_hdr_t;

  typedef struct {
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
  } wifi_ieee80211_packet_t;


/*
  const char *
  wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
  {
    switch(type) {
    case WIFI_PKT_MGMT: return "MGMT";
    case WIFI_PKT_DATA: return "DATA";
    default:	
    case WIFI_PKT_MISC: return "MISC";
    }
  }
*/

  void
  wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)
  {

    if (type != WIFI_PKT_MGMT)
      return;

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    addSeenDevice(hdr->addr2, ppkt->rx_ctrl.rssi);

    // Serial.printf("PACKET TYPE=%s, CHAN=%02d, RSSI=%02d,"
    //   " ADDR1=%02x:%02x:%02x:%02x:%02x:%02x,"
    //   " ADDR2=%02x:%02x:%02x:%02x:%02x:%02x,"
    //   " ADDR3=%02x:%02x:%02x:%02x:%02x:%02x\n",
    //   wifi_sniffer_packet_type2str(type),
    //   ppkt->rx_ctrl.channel,
    //   ppkt->rx_ctrl.rssi,
    //   /* ADDR1 */
    //   hdr->addr1[0],hdr->addr1[1],hdr->addr1[2],
    //   hdr->addr1[3],hdr->addr1[4],hdr->addr1[5],
    //   /* ADDR2 */
    //   hdr->addr2[0],hdr->addr2[1],hdr->addr2[2],
    //   hdr->addr2[3],hdr->addr2[4],hdr->addr2[5],
    //   /* ADDR3 */
    //   hdr->addr3[0],hdr->addr3[1],hdr->addr3[2],
    //   hdr->addr3[3],hdr->addr3[4],hdr->addr3[5]
    // );
  }

#else
  struct RxControl {
    signed rssi:8; // signal intensity of packet
    unsigned rate:4;
    unsigned is_group:1;
    unsigned:1;
    unsigned sig_mode:2; // 0:is 11n packet; 1:is not 11n packet;
    unsigned legacy_length:12; // if not 11n packet, shows length of packet.
    unsigned damatch0:1;
    unsigned damatch1:1;
    unsigned bssidmatch0:1;
    unsigned bssidmatch1:1;
    unsigned MCS:7; // if is 11n packet, shows the modulation and code used (range from 0 to 76)
    unsigned CWB:1; // if is 11n packet, shows if is HT40 packet or not
    unsigned HT_length:16;// if is 11n packet, shows length of packet.
    unsigned Smoothing:1;
    unsigned Not_Sounding:1;
    unsigned:1;
    unsigned Aggregation:1;
    unsigned STBC:2;
    unsigned FEC_CODING:1; // if is 11n packet, shows if is LDPC packet or not.
    unsigned SGI:1;
    unsigned rxend_state:8;
    unsigned ampdu_cnt:8;
    unsigned channel:4; //which channel this packet in.
    unsigned:12;
  };

  #define DATA_LENGTH           112

  #define TYPE_MANAGEMENT       0x00
  #define TYPE_CONTROL          0x01
  #define TYPE_DATA             0x02
  #define SUBTYPE_PROBE_REQUEST 0x04

  struct SnifferPacket{
      struct RxControl rx_ctrl;
      uint8_t data[DATA_LENGTH];
      uint16_t cnt;
      uint16_t len;
  };

  //https://github.com/kalanda/esp8266-sniffer/blob/master/src/main.cpp
  void ICACHE_FLASH_ATTR sniffer_callback(uint8_t *buffer, uint16_t length) {
    struct SnifferPacket *snifferPacket = (struct SnifferPacket*) buffer;
    uint8_t *mac = snifferPacket->data+10;
    addSeenDevice(mac, snifferPacket->rx_ctrl.rssi);
  }
#endif

  void init(){
#ifdef ESP32    
    esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
    esp_wifi_set_promiscuous(true);
#else
    wifi_set_promiscuous_rx_cb(&sniffer_callback);
    wifi_promiscuous_enable(1);
#endif
    //Serial.printf("i");
  }

  #ifdef USE_WEBSERVER
  const char HTTP_SNIFF_mac[] PROGMEM = "{s}WIFI-MAC : %s" " {m} RSSI:%d age %u(%u)" "{e}";

  void SniffShow(){
    uint64_t now = millis();
    now = now/1000L;
    uint32_t nowS = (uint32_t)now;   
    int count = seenDevices.size();
    for (int i = 0; i < count; i++){
      XSNS81::SNIFF_simple_device_t* dev = seenDevices[i];
      uint64_t lastseen = dev->lastseen/1000L;
      uint32_t lastseenS = (uint32_t) lastseen; 
      uint32_t devAge = nowS - lastseenS;
      char addr[20];
      ToHex_P(dev->mac, 6, addr, 20, 0);
      WSContentSend_PD(HTTP_SNIFF_mac,addr,dev->RSSI, devAge, dev->maxAge);
    }
  }
  #endif

  void sniff_mqtt(XSNS81::SNIFF_simple_device_t* dev, int lost) {
    uint64_t now = millis();
    now = now/1000L;
    uint32_t nowS = (uint32_t)now;   
    uint64_t lastseen = dev->lastseen/1000L;
    uint32_t lastseenS = (uint32_t) lastseen; 
    uint32_t devAge = nowS - lastseenS;
    char addr[20];
    ToHex_P(dev->mac, 6, addr, 20, 0);
    dev->publishedRSSI = dev->RSSI;
    dev->secondsCounter = 0;

    ResponseTime_P(PSTR(",\"WIFISNIF\":{\"MAC\":\"%s\",\"RSSI\":%d,\"STATE\":\"%s\"}}"),
      addr,dev->RSSI, lost? "OFF":"ON");
    MqttPublishTeleSensor();
  }

  void everySecond(){
    int count = seenDevices.size();
    for (int i = 0; i < count; i++){
      XSNS81::SNIFF_simple_device_t* dev = seenDevices[i];
      dev->secondsCounter++;
      if (dev->secondsCounter > SnifMQTTInterval){
        sniff_mqtt(dev, 0);
      } else {
        if (abs(dev->RSSI - dev->publishedRSSI) > SnifRSSIChange){
          sniff_mqtt(dev, 0);
        }
      }
    }
  }

}




bool Xsns81(uint8_t function) {
  bool result = false;

  switch (function) {
    case FUNC_INIT:{
    } break;

    case FUNC_EVERY_SECOND:{
      XSNS81::init();
      XSNS81::checkDeviceTimouts();
      XSNS81::everySecond();
    }break;

    case FUNC_EVERY_250_MSECOND:
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      XSNS81::SniffShow();
      break;
#endif  // USE_WEBSERVER
  }
  
  return result;
}

#endif