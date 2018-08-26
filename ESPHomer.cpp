#include <stdlib.h> // for free()
#include <string.h> // for strlen, strcpy, strcat
#include <pgmspace.h> // for PSTR, strcpy_P
#include <inttypes.h> // frintf PRIu32, PRIu16, PRIu8
//#include <HashMap.h> // for Associative Arrays (__topics)

#include "Arduino.h"
#include "ESPHomer.h"

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
//#include <WINSResponder.h>
#include <ArduinoOTA.h>

/*
- Need increase the size of MQTT_MAX_PACKET_SIZE in w:\User\Documents\Arduino\libraries\PubSubClient\src\PubSubClient.h
   from 128 to 512 to get MQTT working properly. This is an issue in the library
   which has not been fixed yet.
   */
#include <PubSubClient.h>


#include <EEPROM.h>
#include <CRC32.h>

String connectionStatus(int which) {
  switch ( which )
    {
        case WL_CONNECTED:
            return F("Connected");
            break;

        case WL_NO_SSID_AVAIL:
            return F("Network not availible");
            break;

        case WL_CONNECT_FAILED:
            return F("Wrong password");
            break;

        case WL_IDLE_STATUS:
            return F("Idle status");
            break;

        case WL_DISCONNECTED:
            return F("Disconnected");
            break;

        default:
            return F("Unknown");
            break;
    }
}

String disconnectReason(int which) {
switch ( which )
    {
        case WIFI_DISCONNECT_REASON_UNSPECIFIED:
            return F("UNSPECIFIED");
            break;

        case WIFI_DISCONNECT_REASON_AUTH_EXPIRE:
            return F("AUTH_EXPIRE");
            break;

        case WIFI_DISCONNECT_REASON_AUTH_LEAVE:
            return F("AUTH_LEAVE");
            break;

        case WIFI_DISCONNECT_REASON_ASSOC_EXPIRE:
            return F("ASSOC_EXPIRE");
            break;

        case WIFI_DISCONNECT_REASON_ASSOC_TOOMANY:
            return F("ASSOC_TOOMANY");
            break;

        case WIFI_DISCONNECT_REASON_NOT_AUTHED:
            return F("NOT_AUTHED");
            break;

        case WIFI_DISCONNECT_REASON_NOT_ASSOCED:
            return F("NOT_ASSOCED");
            break;

        case WIFI_DISCONNECT_REASON_ASSOC_LEAVE:
            return F("ASSOC_LEAVE");
            break;

        case WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED:
            return F("ASSOC_NOT_AUTHED");
            break;

        case WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD:
            return F("DISASSOC_PWRCAP_BAD");
            break;

        case WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD:
            return F("DISASSOC_SUPCHAN_BAD");
            break;

        case WIFI_DISCONNECT_REASON_IE_INVALID:
            return F("IE_INVALID");
            break;

        case WIFI_DISCONNECT_REASON_MIC_FAILURE:
            return F("MIC_FAILURE");
            break;

        case WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT:
            return F("4WAY_HANDSHAKE_TIMEOUT");
            break;

        case WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT:
            return F("GROUP_KEY_UPDATE_TIMEOUT");
            break;

        case WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS:
            return F("IE_IN_4WAY_DIFFERS");
            break;

        case WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID:
            return F("GROUP_CIPHER_INVALID");
            break;

        case WIFI_DISCONNECT_REASON_AKMP_INVALID:
            return F("AKMP_INVALID");
            break;

        case WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION:
            return F("UNSUPP_RSN_IE_VERSION");
            break;

        case WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP:
            return F("INVALID_RSN_IE_CAP");
            break;

        case WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED:
            return F("802_1X_AUTH_FAILED");
            break;

        case WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED:
            return F("CIPHER_SUITE_REJECTED");
            break;

        case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:
            return F("BEACON_TIMEOUT");
            break;

        case WIFI_DISCONNECT_REASON_NO_AP_FOUND:
            return F("NO_AP_FOUND");
            break;

        case WIFI_DISCONNECT_REASON_AUTH_FAIL:
            return F("AUTH_FAIL");
            break;

        case WIFI_DISCONNECT_REASON_ASSOC_FAIL:
            return F("ASSOC_FAIL");
            break;

        case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:
            return F("HANDSHAKE_TIMEOUT");
            break;

        default:
            return F("-=Unknown=-");
            break;
    }
}

ESPHomer::ESPHomer(const char *espname)
  : timeClient(ntpUDP, HOMER_NTP_SERVER, 0, 3600000),
  // You can specify the time server pool and the offset (in seconds, can be
  // changed later with setTimeOffset() ). Additionaly you can specify the
  // update interval (in milliseconds, can be changed using setUpdateInterval() ).
  //initNTPClient(ntpUDP, HOMER_NTP_SERVER, 0, 3600000);
  client(espClient)
  //: s_winsResponder(&ntpUDP);
{
	_espname = espname;
  topic_count = (sizeof(base_topics)/sizeof(char *));

  setCallback(NULL);
  onCommand(NULL);
}

ESPHomer::~ESPHomer() {
  free(_ssid);
  free(_pass);
}

char* ESPHomer::getSSID() {
	return _ssid;
	
}

char* ESPHomer::getPASS() {
	return _pass;
}

void ESPHomer::setAppVer(const char* ver) {
	appver = ver;
}

void ESPHomer::setVccAdj(int16_t adj) {
  _VCC_ADJ = adj;
}

void ESPHomer::setSSID(const char *ssid, const char *pass) {
  _ssid = (char *)ssid;
  _pass = (char *)pass;
}

void ESPHomer::setOTA(const char *pass) {
  _ota_pass = (char *)pass;
}

void ESPHomer::setMQTT(const char *host, uint16_t port) {
  _mqtt_server = (char *)host;
  _mqtt_port = port;
}

void ESPHomer::setNTP(const char *host) {
  _ntp_server = (char *)host;
}

void ESPHomer::setStatusPeriod(uint32_t ms) {
  _statusPeriod = ms;
}

void ESPHomer::pinLed(int8_t pin, uint8_t lvlLedOn) {
  _pinLed = (int8_t) pin;
  _lvlLedOn= lvlLedOn;
  if (_pinLed >= 0) {
    pinMode(_pinLed, OUTPUT);
  }
}

uint8_t ESPHomer::addTopic(const char* name) {
  uint8_t pl = strlen(_topic_prefix);
  uint8_t nl = strlen(_espname);
  uint8_t tl = strlen(name);
  uint8_t memcount = sizeof(char) * (pl + nl + tl + 2);
  _topics[topic_count] = (char *)malloc(memcount);
  strncpy(_topics[topic_count], _topic_prefix, pl);
  strncpy(_topics[topic_count]+pl, _espname, nl);
  strncpy(_topics[topic_count]+pl+nl, "/", 1);
  strncpy(_topics[topic_count]+pl+nl+1, name, tl+1);
  topic_count++;
  return topic_count-1;
}

uint8_t ESPHomer::publish(uint8_t topic_idx, const char* msg) {
  return client.publish(_topics[topic_idx], msg);
}

void ESPHomer::setup() {
  initTime = millis();
  #ifdef SERIAL_DEBUG
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Boot done in ");
  Serial.print(initTime);
  Serial.println("ms");
  #endif
  
  //setup_Serial();

  setup_EEPROM();

  setup_wifi();

  setup_OTA();

  setup_NTP();

  setup_MQTT();

  #ifdef SERIAL_DEBUG
  //возвращает ESP8266 chip ID, int 32bit
  Serial.printf("  ESP8266 ChipId: %08X\r\n", ESP.getChipId());
  //возвращает flash chip ID, int 32bit
  Serial.printf("  Flash   ChipId: %08X\r\n", ESP.getFlashChipId());
  // возвращает размер флеш памяти в байтах, так, как его определяет SDK (может быть меньше реального размера).
  Serial.printf("  Flash ChipSize: %d\r\n", ESP.getFlashChipSize());
  Serial.printf("  Flash realSize: %d\r\n", ESP.getFlashChipRealSize());
  // возвращает частоту флеш памяти, в Гц.
  Serial.printf("  FlashChipSpeed: %d\r\n", ESP.getFlashChipSpeed());
  FlashMode_t ideMode = ESP.getFlashChipMode();
  Serial.printf("  Flash ide mode: %s\r\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  Serial.printf("  SDK version: "); Serial.println(ESP.getSdkVersion());
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Ready [");
  Serial.print(_espname);
  Serial.println("]");
  #endif
}

/*void ESPHomer::setup_Serial() {
  #ifdef SERIAL_DEBUG
  time1 = millis();

  Serial.begin(115200);
  
  Serial.setTimeout(2000);
  // Wait for serial to initialize.
  while(!Serial) { }
  time2 = millis() - time1;
  Serial.print("Serial: ");
  Serial.print(time2);
  Serial.println("ms");
  #endif
}
*/
//TODO
//WiFi.onSoftAPModeStationConnected(onSoftAPModeStationConnected);
//WiFi.onSoftAPModeStationDisconnected(onSoftAPModeStationDisconnected);
//WiFi.onSoftAPModeProbeRequestReceived(onSoftAPModeProbeRequestReceived);

void ESPHomer::setup_EEPROM() {
  #ifdef SERIAL_DEBUG
  time1 = millis();
  #endif

  word i;
  uint8_t eeprom_data_tmp[sizeof(eeprom_data)];
  uint32_t memcrc; uint8_t *p_memcrc = (uint8_t*)&memcrc;
  uint8_t format; uint8_t *p_format = (uint8_t*)&format;
  uint32_t datacrc;
  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  for (i = EEPROM_START; i < EEPROM_START+sizeof(eeprom_data); i++) {
    eeprom_data_tmp[i] = EEPROM.read(i);
  }

  p_format[0] = eeprom_data_tmp[0];
  
  if (format == EEPROM_FORMAT_VERSION) {
    p_memcrc[0] = EEPROM.read(i++);
    p_memcrc[1] = EEPROM.read(i++);
    p_memcrc[2] = EEPROM.read(i++);
    p_memcrc[3] = EEPROM.read(i++);
  
    datacrc = CRC32::calculate(eeprom_data_tmp, sizeof(eeprom_data_tmp));
    if (memcrc == datacrc) {
      setEEPROM = true;
      memcpy(&eeprom_data, eeprom_data_tmp,  sizeof(eeprom_data));
    }
  } else {
    // format UPGRADE/DOWNGRADE
    if (format == 1) {
        unsigned long date; uint8_t *p_date = (uint8_t*)&date;
        p_date[0] = eeprom_data_tmp[1];
        p_date[1] = eeprom_data_tmp[2];
        p_date[2] = eeprom_data_tmp[3];
        p_date[3] = eeprom_data_tmp[4];
        setEEPROM = true;
        // FIXME HERE - EEPROM_FORMAT_VERSION UPGRADE
        eeprom_data.format = EEPROM_FORMAT_VERSION;
        eeprom_data.updateTime = date;
    } else
      // DOWNGRADE
      if (format == 2) {
        unsigned long date; uint8_t *p_date = (uint8_t*)&date;
        p_date[0] = eeprom_data_tmp[1];
        p_date[1] = eeprom_data_tmp[2];
        p_date[2] = eeprom_data_tmp[3];
        p_date[3] = eeprom_data_tmp[4];
        setEEPROM = true;
        // FIXME HERE - EEPROM_FORMAT_VERSION UPGRADE
        eeprom_data.format = EEPROM_FORMAT_VERSION;
        eeprom_data.updateTime = date;
    }
  }

  #ifdef SERIAL_DEBUG

  time2 = millis() - time1;
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] EEPROM: ");
  Serial.print(time2);
  Serial.println("ms");

  Serial.printf("  format:     %u\r\n  updateTime: %" PRIu32 "\r\n", eeprom_data.format, eeprom_data.updateTime);
  #endif
}

void ESPHomer::setup_wifi() {
  // We start by connecting to a WiFi network
  #ifdef SERIAL_DEBUG
  time1 = millis();
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Start wifi for connecting to ");
  Serial.println(_ssid);
  #endif
  
  //WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setPhyMode(WIFI_PHY_MODE_11N); //WIFI_PHY_MODE_11G
  WiFi.hostname(_espname);
  //WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  //WiFi.onEvent(WiFiEvent); // depricated
  stationModeConnected = WiFi.onStationModeConnected([this](const WiFiEventStationModeConnected& event) {
	  isTimeouted = false;
	  #ifdef SERIAL_DEBUG
	  unsigned long now = millis();
	  Serial.printf("[%lu] [WiFi] Connected. SSID=%s BSSID=%s ch=%u\r\n", now, event.ssid.c_str(), initMAC(event.bssid), event.channel);
	  #endif
  });
  stationModeDisconnected = WiFi.onStationModeDisconnected([this](const WiFiEventStationModeDisconnected& event) {
	  #ifdef SERIAL_DEBUG
	  unsigned long now = millis();
    Serial.printf("[%lu] [WiFi] Disconnected [%d:%s] Status %d, %s\r\n", now, event.reason, disconnectReason(event.reason).c_str(), WiFi.status(), connectionStatus( WiFi.status() ).c_str() );
    #endif
  });
  stationModeAuthModeChanged = WiFi.onStationModeAuthModeChanged([this](const WiFiEventStationModeAuthModeChanged& event) {
	  #ifdef SERIAL_DEBUG
	  unsigned long now = millis();
    Serial.printf("[%lu] [WiFi] AuthMode Change %u -> %u\r\n", now, event.oldMode, event.newMode);
    #endif
  });
  stationModeGotIP = WiFi.onStationModeGotIP([this](const WiFiEventStationModeGotIP& event) {
    #ifdef SERIAL_DEBUG
    unsigned long now = millis();
    Serial.printf("[%lu] [WiFi] Got IP: %s, gw: %s, host=%s in %lums\r\n", now, event.ip.toString().c_str(), event.gw.toString().c_str(), WiFi.hostname().c_str(), (now - timeStartWifiConnection));
    #endif
    //s_winsResponder.begin(WiFi.hostname().c_str(), event.ip);
    if (MDNS.begin(WiFi.hostname().c_str(), event.ip)) {
      MDNS.addService("homer", "tcp", 28066);
    }
    #ifdef SERIAL_DEBUG
    Serial.print("[");
    Serial.print(millis());
    Serial.println("] WINS??? and mDNS responder started");
    #endif
  });
  stationModeDHCPTimeout = WiFi.onStationModeDHCPTimeout([this]() {
    #ifdef SERIAL_DEBUG
    unsigned long now = millis();
    Serial.printf("[%lu] [WiFi] DHCP timeout\r\n", now);
    #endif
  });
  //TODO
  //WiFi.onSoftAPModeStationConnected(onSoftAPModeStationConnected);
  //WiFi.onSoftAPModeStationDisconnected(onSoftAPModeStationDisconnected);
  //WiFi.onSoftAPModeProbeRequestReceived(onSoftAPModeProbeRequestReceived);
  /*
void WiFiEvent(WiFiEvent_t event) {
    unsigned long now = millis();
    #ifdef SERIAL_DEBUG
    Serial.printf("[%d] [WiFi-event] event: %d\r\n", now, event);
    #endif
    
    case WIFI_EVENT_SOFTAPMODE_STACONNECTED:
      #ifdef SERIAL_DEBUG
      Serial.printf("[AP] %d, Client Connected\r\n", event);
      #endif
    break;
    
    case WIFI_EVENT_SOFTAPMODE_STADISCONNECTED:
      #ifdef SERIAL_DEBUG
      Serial.printf("[AP] %d, Client Disconnected\r\n", event);
      #endif
    break;
    
    case WIFI_EVENT_SOFTAPMODE_PROBEREQRECVED:
      #ifdef SERIAL_DEBUG
      Serial.printf("[AP] %d, Probe Request Recieved\r\n", event);
      #endif
    break;
  }
}
  */


  if (WiFi.status() != WL_CONNECTED) {
    timeStartWifiConnection = millis();
    lastWifiAttempt = timeStartWifiConnection;
    isTimeouted = true;
    WiFi.begin(_ssid, _pass);
    /*
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        #ifdef SERIAL_DEBUG
        Serial.print(".");
        #endif
        time2 = millis();
        if (time2 - time1 > WIFI_CONNECT_TIMEOUT_MS)  // wifi connection lasts too ling, retry
        {
          #ifdef SERIAL_DEBUG
          Serial.println("Connection Failed! Rebooting...");
          #endif
          delay(1000);
          ESP.restart();
          yield();
        }
    }
    */
  } else {
    #ifdef SERIAL_DEBUG
    Serial.println("");
    Serial.println("!!! WiFi ALREADY connected !!!");
    #endif
  }

  #ifdef SERIAL_DEBUG
  time2 = millis() - time1;
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Wifi: ");
  Serial.print(time2);
  Serial.println("ms");
  #endif
}

void ESPHomer::setup_OTA() {
  #ifdef SERIAL_DEBUG
  time1 = millis();
  #endif
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(_espname);

  // No authentication by default
  ArduinoOTA.setPassword(_ota_pass);
/*
  #ifdef SERIAL_DEBUG
  Serial.print("OTA: [");
  Serial.print(_ota_pass
  );
  Serial.println("]");
  #endif
*/

  ArduinoOTA.onStart([this]() {
    #ifdef SERIAL_DEBUG
    Serial.print("[");
    Serial.print(millis());
    Serial.println("] Start OTA update");
    #endif
  });
  ArduinoOTA.onEnd([this]() {
    #ifdef SERIAL_DEBUG
    Serial.print("\n[");
    Serial.print(millis());
    Serial.println("] End OTA update");
    #endif
    //EPPROM set
    eeprom_data.updateTime = timeClient.getEpochTime();
    writeEEPROM(true);
    // Dont delete this, this need for end all network job before reboot
    delay(500);
  });
  ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
    #ifdef SERIAL_DEBUG
    Serial.printf("[%lu] OTA Progress: %u%%\r", millis(), (progress / (total / 100)));
    #endif
  });
  ArduinoOTA.onError([this](ota_error_t error) {
    #ifdef SERIAL_DEBUG
    Serial.print("\n[");
    Serial.print(millis());
    Serial.printf("] OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    #endif
  });
  
  ArduinoOTA.begin();

  #ifdef SERIAL_DEBUG
  time2 = millis() - time1;
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] OTA: ");
  Serial.print(time2);
  Serial.println("ms");
  #endif
}

void ESPHomer::setup_NTP() {
  #ifdef SERIAL_DEBUG
  time1 = millis();
  #endif

  timeClient.setPoolServerName(_ntp_server);
  timeClient.begin();

  #ifdef SERIAL_DEBUG
  time2 = millis() - time1;
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] NTP: ");
  Serial.print(time2);
  Serial.println("ms");
  #endif
}

void ESPHomer::setup_MQTT() {
  #ifdef SERIAL_DEBUG
  time1 = millis();
  #endif

  uint8_t pl = strlen(_topic_prefix);
  uint8_t nl = strlen(_espname);
  uint8_t i = 0;
  uint8_t base_topic_count = (sizeof(base_topics)/sizeof(char *));
  while(i < base_topic_count && base_topics[i]) {
    uint8_t tl = strlen(base_topics[i]);
    if (tl == 0) {
      break;
    }

    // если ещё не было замены
    if (tl < pl || strncmp(_topic_prefix, _topics[i], pl)) {
      uint8_t memcount = sizeof(char) * (pl + nl + tl + 2);
      _topics[i] = (char *)malloc(memcount);
      strncpy(_topics[i], _topic_prefix, pl);
      strncpy(_topics[i]+pl, _espname, nl);
      strncpy(_topics[i]+pl+nl, "/", 1);
      strncpy(_topics[i]+pl+nl+1, base_topics[i], tl+1);
    }

    i++;
  }



  client.setServer(_mqtt_server, _mqtt_port);
  client.setCallback([this] (char* topic, byte* payload, unsigned int length) { this->_callback(topic, payload, length); });

  #ifdef SERIAL_DEBUG
  time2 = millis() - time1;
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] MQTT: ");
  Serial.print(time2);
  Serial.println("ms");
  #endif
}

uint8_t ESPHomer::loop(boolean isCanReboot) {
  loopState = 255; // Unknown error
  #ifdef SERIAL_DEBUG
  if (inited == 0) {
    inited = millis();
    Serial.print("[");
    Serial.print(inited);
    Serial.print("] INITED in ");
    Serial.print(millis() - initTime);
    Serial.println("ms");
  }
  #endif

  if (millis() - lastNTPUpdate >= 1000) {
    lastNTPUpdate = millis();
    nowtime = timeClient.getEpochTime();
  }

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    timeClient.update();
  
    if (!client.connected()) {
      loopState = 1; // MQTT NOT connected
      long now = millis();
      if (now - lastReconnectAttempt > 5000) {
        lastReconnectAttempt = now;
        if (reconnect()) {
          lastReconnectAttempt = 0;
        }
      }
    } else {
      client.loop();
      StatusSend();
      loopState = 0; // All OK
    }
  } else {
    loopState = 2; // Wifi NOT connected
    uint32_t now = millis();
    // wifi connection lasts too ling, retry
    if (isTimeouted && now - timeStartWifiConnection > WIFI_CONNECT_TIMEOUT_MS) {
      loopState = 254; // REBOOTING
      #ifdef SERIAL_DEBUG
      Serial.println("Connection Failed! Rebooting...");
      #endif
      delay(1000);
      ESP.restart();
      yield();
    }
    
    #ifdef SERIAL_DEBUG
    if (now - lastWifiAttempt > 5000) {
      lastWifiAttempt = now;
      Serial.printf("WiFi NOT connected: [%d] %s\r\n", WiFi.status(), connectionStatus( WiFi.status() ).c_str());
    }
    #endif
  }

  // max 4294967.296 sec So - 3600296
  if (millis() > 4291367000 && isCanReboot) {
    loopState = 253; // RESET MILIS
    ESP.restart();
    ESP.reset();
    delay(100);
    yield();
  }

  return loopState;
}

boolean ESPHomer::reconnect() {
  #ifdef SERIAL_DEBUG
  Serial.print("[");
  Serial.print(millis());
  Serial.println(F("] Attempting MQTT connection... "));
  time1 = millis();
  #endif
  if (client.connect(_espname)) {
    #ifdef SERIAL_DEBUG
    time2 = millis() - time1;
    time1 = millis();
    Serial.print("[");
    Serial.print(time1);
    Serial.print("] ");
    Serial.printf("mqtt.connect: %lu\r\n", time2);
    #endif

    unsigned long t = timeClient.getEpochTime();
    String name = WiFi.SSID();
    long rssi = WiFi.RSSI();
    uint32_t freemem = ESP.getFreeHeap();
    float vcc = ((float)ESP.getVcc())/_VCC_ADJ;
    
    //String msg = "connected "+name+" ("+String(_espname)+") {"+getMAC()+"} ["+WiFi.localIP().toString()+"] h"+String(ESP_HOMER_VERSION)+" v"+appver+" #"+eeprom_data.updateTime+" TI:"+inited+" T:"+String(t)+" R:"+String(rssi)+" M:"+String(freemem)+" V:"+String(vcc);
    char msg[MQTT_MAX_PACKET_SIZE];
    const char * init_format = PSTR(R"({"act":"connected","ap":"%s","name":"%s","mac":"%s","ip":"%s","rssi":%d,"vcc":%.2f,"h":"%s","v":"%s","upd":%u,"time":%u,"inited":%u,"free":%u})");
    snprintf_P(msg, sizeof(msg), init_format, name.c_str(), _espname, getMAC(), WiFi.localIP().toString().c_str(), rssi, vcc, ESP_HOMER_VERSION, appver, eeprom_data.updateTime, t, inited, freemem);
    #ifdef SERIAL_DEBUG
    Serial.print("[");
    Serial.print(millis());
    Serial.print("] ");
    Serial.println(msg);
    //Serial.print("size: ");
    //Serial.println(msg.length());
    #endif

    if (!client.connected()) {
      serialln(F("NOT CONNECTED !!!"));
    }
    
    client.publish(_topic_init, msg); // init
    // ... and resubscribe
    client.subscribe(_topics[C_LED]); // 1 - led
    client.subscribe(_topics[C_CMD]); // 3 - cmd
    #ifdef SERIAL_DEBUG
    time2 = millis() - time1;
    Serial.print("[");
    Serial.print(millis());
    Serial.print("] ");
    Serial.printf_P(PSTR("mqtt.subscribe: %lu\r\n"), time2);
    if (_firstConnect) {
      _firstConnect = 0;
      Serial.print("[");
      Serial.print(millis());
      Serial.print("] Full READY in ");
      Serial.print(millis() - initTime);
      Serial.println("ms");
    }
    #endif
    return client.connected();
  }

  #ifdef SERIAL_DEBUG
  time2 = millis() - time1;
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] failed, rc=");
  Serial.print(client.state());
  Serial.println(" try again in 5s");
  Serial.printf("\tmqtt.connecting: %lu\r\n", time2);
  #endif
  return 0;
}

void ESPHomer::writeEEPROM(boolean isCommit) {
  word i;
  uint8_t eeprom_data_tmp[sizeof(eeprom_data)];
  uint32_t memcrc; uint8_t *p_memcrc = (uint8_t*)&memcrc;
  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  eeprom_data.format = EEPROM_FORMAT_VERSION;

  memcpy(eeprom_data_tmp, &eeprom_data, sizeof(eeprom_data));

  for (i = EEPROM_START; i < EEPROM_START+sizeof(eeprom_data); i++) {
    EEPROM.write(i, eeprom_data_tmp[i]);
  }
  
  memcrc = CRC32::calculate(eeprom_data_tmp, sizeof(eeprom_data_tmp));

  EEPROM.write(i++, p_memcrc[0]);
  EEPROM.write(i++, p_memcrc[1]);
  EEPROM.write(i++, p_memcrc[2]);
  EEPROM.write(i++, p_memcrc[3]);

  if (isCommit) {
    EEPROM.commit();
  }
}

char* ESPHomer::initMAC(const uint8_t mac[WL_MAC_ADDR_LENGTH]) {
  snprintf(__mac, sizeof(__mac)
        , "%02X:%02X:%02X:%02X:%02X:%02X"
        , mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return __mac;
}
/*
String ESPHomer::showMAC(const uint8_t mac[WL_MAC_ADDR_LENGTH]) {
  String resukt = String(mac[WL_MAC_ADDR_LENGTH - 6], HEX) + ":" +
               String(mac[WL_MAC_ADDR_LENGTH - 5], HEX) + ":" +
               String(mac[WL_MAC_ADDR_LENGTH - 4], HEX) + ":" +
               String(mac[WL_MAC_ADDR_LENGTH - 3], HEX) + ":" +
               String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) + ":" +
               String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
   resukt.toUpperCase();
   return resukt;  
}

String ESPHomer::getMAC() {
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  return showMAC(mac);
}
*/
char* ESPHomer::getMAC() {
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  return initMAC(mac);
}

void ESPHomer::serialln(const char* msg) {
  #ifdef SERIAL_DEBUG
  Serial.println(msg);
  #endif
}

void ESPHomer::serialln(const __FlashStringHelper* msg) {
  #ifdef SERIAL_DEBUG
  Serial.println(msg);
  #endif
}

void ESPHomer::StatusSend() {
  long now = millis();
  if (now - last_stat_stamp > _statusPeriod) {
      last_stat_stamp=now;
      unsigned long t = timeClient.getEpochTime();
      long rssi = WiFi.RSSI();
      String name = WiFi.SSID();
      unsigned long freemem = ESP.getFreeHeap();
      float vcc = ((float)ESP.getVcc())/_VCC_ADJ;
      if (client.connected()) {
        //String msg = "status t="+String(t)+" s="+name+" r="+String(rssi)+" m="+String(freemem)+" v="+String(vcc)+" u="+String(millis());
        char msg[MQTT_MAX_PACKET_SIZE];
        const char * init_format = PSTR(R"({"act":"status","time":%u,"ap":"%s","rssi":%d,"vcc":%.2f,"free":%u,"uptime":%u})");
        snprintf_P(msg, sizeof(msg), init_format, t, name.c_str(), rssi, vcc, freemem, millis());
        client.publish(_topics[C_STAT], msg); // stat
        #ifdef SERIAL_DEBUG
        Serial.println(msg);
        #endif
      }
  }
}


void ESPHomer::_callback(char* topic, byte* payload, unsigned int length) {
  #ifdef SERIAL_DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif

  // проверяем из нужного ли нам топика пришли данные 
  if (_pinLed >= 0 && strcmp(topic, _topics[C_LED]) == 0) { // led
    if ((char)payload[0] == '1') {
      #ifdef SERIAL_DEBUG
      Serial.println("ON");
      #endif
      digitalWrite(_pinLed, _lvlLedOn); // включаем или выключаем светодиод в зависимоти от полученных значений данных
    } else {
      #ifdef SERIAL_DEBUG
      Serial.print("OFF");
      #endif
      digitalWrite(_pinLed, !_lvlLedOn);
    }
  } else if (strcmp(topic, _topics[C_CMD])==0) { // cmd
    char cmd[HOMER_MAX_CMD_SIZE+1];
    uint8_t dataidx = 0;
    for (uint8_t i = 0; i < length; i++) {
      if (payload[i] == 32) {// space
        if (i <= HOMER_MAX_CMD_SIZE) {
          memcpy(cmd, payload, i);
          cmd[i] = '\0';
          dataidx = i+1;
        } else {
          Serial.print("WARN: max len cmd is 16 char!!!");
          Serial.print(HOMER_MAX_CMD_SIZE);
          Serial.println("char!!!");
        }
      }
    }

    if (dataidx == 0) {
      if (length <= HOMER_MAX_CMD_SIZE) {
        dataidx = length;
        memcpy(cmd, payload, length);
        cmd[length] = '\0';
      }
    }

    if (strcmp(cmd, "scan") == 0) {
      unsigned long t = timeClient.getEpochTime();
      serialln("** Scan Networks **");
      long rssi = WiFi.RSSI();
      String name = WiFi.SSID();
      unsigned long freemem = ESP.getFreeHeap();
      float vcc = ((float)ESP.getVcc())/_VCC_ADJ;
      uint8_t numSsid = WiFi.scanNetworks();
      serialln("scan done");
      
      if (numSsid == 0) {
        serialln("no networks found");
      } else {
        String msg = "scan t="+String(t)+" s="+name+" r="+String(rssi)+" m="+String(freemem)+" v="+String(vcc)+" u="+String(millis());
        for (int i = 0; i < numSsid; ++i) {
          String _name = WiFi.SSID(i);
          long _rssi = WiFi.RSSI(i);
          #ifdef SERIAL_DEBUG
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(_name);
          Serial.print(" (");
          Serial.print(_rssi);
          Serial.print(")");
          Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
          #endif
          msg += "\n[" + _name + "] " + _rssi;
        }

        if (client.connected()) {
            client.publish(_topics[C_SCAN], msg.c_str()); // scan
        }
      }
    } else {
      if (commandHandler) {
        // Move data of cmd to buff
        char* data[MQTT_MAX_PACKET_SIZE-HOMER_MAX_CMD_SIZE+1];
        memcpy(data, payload+dataidx, length-dataidx);
        data[length-dataidx] = '\0';
        commandHandler((char*)cmd, (char*)data);
      }  
    }
  } else {
    if (callback) {
      callback(topic,payload,length);
    }
  }
}

ESPHomer& ESPHomer::setCallback(HOMER_CALLBACK_SIGNATURE) {
    this->callback = callback;
    return *this;
}

ESPHomer& ESPHomer::onCommand(HOMER_COMMAND_SIGNATURE) {
    this->commandHandler = commandHandler;
    return *this;
}
