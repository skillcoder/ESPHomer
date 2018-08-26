#ifndef ESPHomer_h
#define ESPHomer_h

#include "Arduino.h"

#define ESP_HOMER_VERSION "0.12"

#include <ESP8266WiFi.h>

/*
- Need increase the size of MQTT_MAX_PACKET_SIZE in w:\User\Documents\Arduino\libraries\PubSubClient\src\PubSubClient.h
   from 128 to 512 to get MQTT working properly. This is an issue in the library
   which has not been fixed yet.
   */
#include <PubSubClient.h>

#include <WiFiUdp.h>
//#include <WINSResponder.h>
// ATTENTION!!! Need NTPClient version ^3.2.0 or from https://github.com/arduino-libraries/NTPClient.git
// Cuz using setPoolServerName
#include <NTPClient.h>
// HT7333 need 2 capasitor 10uF, but on vcc need min 1000uF 6.3v for small spike

// DEEP SLEEP = GPIO16 -> RST
// https://www.losant.com/blog/making-the-esp8266-low-powered-with-deep-sleep
// Max deepSleepTime is ~ 1 hour (32 bit in us (microseconds))
// ESP.deepSleep(20e6, mode); // 20e6 is 20 seconds // mode -> WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED
// delay(100);
// Need use RTC momory

// Modem Sleep (15mA consumption)
// Wifi.forceSleepBegin();
// Wifi.forceSleepWake();

// FLASH = GPIO0 -> GND

// https://esp8266.ru/arduino-ide-esp8266/
// ESP.wdtEnable(), ESP.wdtDisable(), и ESP.wdtFeed() управляют сторожевым таймером.
// ESP.getChipId() возвращает ESP8266 chip IDE, int 32bit
// ESP.getFlashChipId() возвращает flash chip ID, int 32bit
// ESP.getFlashChipSize() возвращает размер флеш памяти в байтах, так, как его определяет SDK (может быть меньше реального размера).
// ESP.getFlashChipSpeed(void) возвращает частоту флеш памяти, в Гц.
// ESP.getCycleCount() возвращает количество циклов CPU с момента старта, unsigned 32-bit. 

// Внешний сторожевой таймер для ESP8266 на NE555
// http://samopal.pro/wdt-ne555/

// WIFI setup 
// https://git.kernel.org/pub/scm/linux/kernel/git/sforshee/wireless-regdb.git/tree/db.txt
// https://habrahabr.ru/post/317220/
// 2. Лучший регуляторный домен — это VE.
// 4. 13 канал обычно пустует.
// 5. ACS_SURVEY, ширина канала 20 МГц, TX-STBC, RX-STBC123 улучшат качество сигнала.
// 6. 40 МГц, больше антенн, SHORT-GI увеличат скорость, но SHORT-GI уменьшает помехозащищённость


// ----------------------------------- eeprom
// increment EEPROM_FORMAT_VERSION if add fields to eeprom_data_t
// add code for support new version and old too
#define EEPROM_FORMAT_VERSION 1
#define EEPROM_START 0
struct eeprom_data_t {
  uint8_t format = 255;
  uint32_t updateTime = 0;
};
// ----------------------------------- 

#ifdef ESP8266
#include <functional>
#define HOMER_CALLBACK_SIGNATURE std::function<void(char*, uint8_t*, unsigned int)> callback
#define HOMER_COMMAND_SIGNATURE std::function<void(char*, char*)> commandHandler
#else
#define HOMER_CALLBACK_SIGNATURE void (*callback)(char*, uint8_t*, unsigned int)
#define HOMER_COMMAND_SIGNATURE void (*commandHandler)(char*, char*)
#endif

#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 60000
#endif

#ifndef BATT_WARNING_VOLTAGE
#define BATT_WARNING_VOLTAGE 3.1
#endif

// According to the nonOS datasheet: Power voltage of VDD33; unit: 1/1024 V
#ifndef VCC_ADJ
#define VCC_ADJ 1024
#endif

#ifndef HOMER_STATUS_PERIOD_MS
#define HOMER_STATUS_PERIOD_MS 300000
#endif

#ifndef HOMER_NTP_SERVER
#define HOMER_NTP_SERVER "192.168.1.2"
#endif

#ifndef HOMER_MQTT_SERVER
#define HOMER_MQTT_SERVER "192.168.1.2"
#endif

#ifndef HOMER_MQTT_PORT
#define HOMER_MQTT_PORT 1883
#endif

// base_channel_e alreadu use some
#ifndef HOMER_MQTT_MAX_TOPIC_COUNT 
#define HOMER_MQTT_MAX_TOPIC_COUNT 10
#endif

#ifndef HOMER_MAX_CMD_SIZE 
#define HOMER_MAX_CMD_SIZE 16
#endif

enum base_channel_e {C_STAT, C_LED, C_DATA, C_CMD, C_SCAN};

class ESPHomer
{
  public:
    uint32_t nowtime = 0;
    uint32_t inited = 0;
    NTPClient timeClient;
    PubSubClient client;

    //void initNTPClient(WiFiUDP _udp, const char* , int _offset, int _update_interval) : timeClient(_udp, _server, _offset, _update_interval) {}

    ESPHomer(const char *espname);
    ~ESPHomer();

    char* getSSID();
    char* getPASS();

    void setup();
    uint8_t loop(boolean isCanReboot);

    void writeEEPROM(boolean isCommit);
    boolean reconnect();
    //String getMAC();
    char*  getMAC();
    //String showMAC(const uint8_t mac[WL_MAC_ADDR_LENGTH]);
    char* initMAC(const uint8_t mac[WL_MAC_ADDR_LENGTH]);
    void serialln(const char* msg);
    void serialln(const __FlashStringHelper* msg);
    void setAppVer(const char* ver);
    void setVccAdj(int16_t adj);
    void setOTA(const char *pass);
    void setMQTT(const char *host, uint16_t port);
    void setNTP(const char *host);
    void setStatusPeriod(uint32_t ms);
    void pinLed(int8_t pin, uint8_t lvlLedOn);
    uint8_t publish(uint8_t topic_idx, const char *msg);
    uint8_t addTopic(const char* name);
    void setSSID(const char *ssid, const char *pass);
    ESPHomer& setCallback(HOMER_CALLBACK_SIGNATURE);
    ESPHomer& onCommand(HOMER_COMMAND_SIGNATURE);
  private:
    //static const char init_format[] PROGMEM = "";
    const char* appver;
    int16_t _VCC_ADJ = VCC_ADJ;
    unsigned long initTime = 0;
    unsigned long time1, time2;
    unsigned long last_stat_stamp = 0;
    unsigned long lastReconnectAttempt = 0;
    unsigned long lastWifiAttempt = 0;
    uint32_t lastNTPUpdate = 0;
    unsigned long timeStartWifiConnection = 0;
    boolean isTimeouted = false;
    uint8_t loopState = 0;
    uint8_t _firstConnect = 1;

    const char* _espname;
  	const char* _topic_init = "/esp/init";
    const char* _topic_prefix = "/esp/";
    
    const char* base_topics[5]={"stat", "led", "data", "cmd", "scan"};
    char* _topics[HOMER_MQTT_MAX_TOPIC_COUNT];
    char* _ssid;
    char* _pass;
    const char* _ota_pass = "HOMERotaPASS";
    const char* _mqtt_server = HOMER_MQTT_SERVER;
    int16_t _mqtt_port = HOMER_MQTT_PORT;
    const char* _ntp_server = HOMER_NTP_SERVER;
    uint8_t topic_count;
    int8_t _pinLed = -1;
    int8_t _lvlLedOn = HIGH;
    uint32_t _statusPeriod = HOMER_STATUS_PERIOD_MS;
    boolean setEEPROM = false;
    WiFiUDP ntpUDP;
    WiFiClient espClient;
    //WINSResponder s_winsResponder;
    eeprom_data_t eeprom_data;
    char __mac[WL_MAC_ADDR_LENGTH*3];

    WiFiEventHandler stationModeConnected, stationModeDisconnected, stationModeAuthModeChanged, stationModeGotIP, stationModeDHCPTimeout;

    //void setup_Serial();
    void setup_wifi();
    void setup_OTA();
    void setup_NTP();
    void setup_MQTT();
    void setup_EEPROM();

    void StatusSend();
    //String connectionStatus(int which);
    //void WiFiEvent(WiFiEvent_t event);
    
    void _callback(char* topic, byte* payload, unsigned int length);
    HOMER_CALLBACK_SIGNATURE;
    HOMER_COMMAND_SIGNATURE;
};

#endif