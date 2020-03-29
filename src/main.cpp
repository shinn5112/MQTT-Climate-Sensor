#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <DHT.h>
#include <settings.h>

// globals
char payload[5];
WiFiClient network;
MQTTClient client;
DHT dht;
bool rtcValid;
float temperature, humidity, battery_voltage;

// prototype functions
void transmit();
void connect();
void mqtt_connect();
uint32_t calculateCRC32(const uint8_t *data, size_t length);

// structs
/* The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
 * so the RTC data structure should be padded to a 4-byte multiple. 
 */
struct {
  uint32_t crc32;        // 4 bytes
  uint8_t channel;       // 1 byte,   5 in total
  uint8_t bssid[6];      // 6 bytes, 11 in total
  uint8_t padding;       // 1 byte,  12 in total
  float battery_voltage; // 4 bytes, 16 total
  float temperature;     // 4 bytes, 20 total
  float humidity;        // 4 bytes, 24 total
  uint32_t reset_state;
} rtcData;


void setup() {
  // power saving, ensuring that the wifi radio is not killing our battery when the setup loop is called
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  // initialize the dht and connect to wifi/MQTT
  Serial.begin(9600);
  Serial.println("Time waking up...");
  dht.setup(dht_pin, dht.DHT22);

  // attempts to read the rtcData from flash memory, if none is found, we assume this is a fresh 
  if(ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4 );
    if(crc == rtcData.crc32) {
      rtcValid = true;
    }
  }
}


void loop() {
  Serial.println("Reading sensors now.");

  // read temperature/humidity and pring debug info
  temperature = dht.toFahrenheit(dht.getTemperature());
  humidity = dht.getHumidity();

  // if we get a bad value, reset and try to fix it.
  if(isnan(temperature) || isnan(humidity)) ESP.restart();

  battery_voltage = analogRead(A0);
  if (battery_voltage == 1024){
    Serial.print("rf disabled screwed up the analog read... resetting.");
    ESP.reset();
  }
  Serial.println(battery_voltage);
  battery_voltage *= (4.2/1035.5); // calculated value for my resistor setup.
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Battery voltage: ");
  Serial.println(battery_voltage);

  // check to see if we need to transmit data
  if(rtcValid){ // if we can access the rtcData and that it is valid, check to see if we need to transmit data
    if(!(battery_voltage == rtcData.battery_voltage && abs(temperature - rtcData.temperature) < 0.5 && abs(humidity - rtcData.humidity) < 0.5)) transmit();
    else Serial.println("No data changes, nothing to transmit.");
  }
  // if we do not have valid rtcData, we transmit to update/write the rtcData.
  else transmit();
  

  Serial.println("I've done my work, nap time!");
  client.disconnect();
  WiFi.disconnect(true);
  delay(1); // give time to complete the action

  // deep sleep to save power, disable radio.
  rtcData.reset_state = 1;
  ESP.deepSleep(wait, WAKE_RF_DISABLED);
}

/**
 * Handles connections to WiFi and MQTT, then transmits the collected data.
 */
void transmit(){
  // connect to wifi and MQTT
  connect();
  mqtt_connect();
  client.loop(); 

  // send over MQTT
  dtostrf(temperature, 4, 2, payload);  // convert temp to char array
  client.publish(publish_topic_temperature, payload, false, 0);
  dtostrf(humidity, 4, 2, payload); // convert humidity to char array
  client.publish(publish_topic_humidity, payload, false, 0);
  dtostrf(battery_voltage, 4, 2, payload); // covert battyer percent to char array
  client.publish(publish_topic_battery, payload);
}

/**
 * Connects to the WiFi network specified in the settings header.
 * 
 * This function attempts to save power by connecting to the network
 * using a static IP address and by using previously discovered network
 * settings. In the event that the WiFI channel changes, it will 
 * attempt to discover the new channel and update the sensor's
 * settings that are stored in the rtc flash memory.
 */
void connect(){
  int retries = 0;
  int wifiStatus;
  bool channelChange = false; 

  Serial.print("Connecting to WiFi Network: ");
  Serial.println(ssid);

  // renable the radio
  WiFi.forceSleepWake();
  delay(1);

  // prevent WiFi settings load/save to flash memory
  WiFi.persistent(false);

  // Bring up the WiFi connection
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet); // static configs save power

  if(rtcValid) {
    // The RTC data was good, make a quick connection
    WiFi.begin(ssid, pass, rtcData.channel, rtcData.bssid, true );
  }else{
    // The RTC data was not valid, so make a regular connection
    WiFi.begin(ssid, pass);
    channelChange = true;
  }
  Serial.print("Connecting");

  wifiStatus = WiFi.status();
  while( wifiStatus != WL_CONNECTED ) {
    retries++;
    if( retries == 100 ) {
      // Quick connect is not working, reset WiFi and try regular connection
      channelChange = true;
      WiFi.disconnect();
      delay( 10 );
      WiFi.forceSleepBegin();
      delay( 10 );
      WiFi.forceSleepWake();
      delay( 10 );
      WiFi.begin(ssid, pass);
    }
    if( retries == 600 ) {
      // Giving up after 30 seconds and going back to sleep
      WiFi.disconnect(true);
      Serial.println("\nFatal Error, could not connect to WiFi. Sleeping.");
      delay(1);
      WiFi.mode(WIFI_OFF);
      ESP.deepSleep( wait, WAKE_RF_DISABLED );
    }
    Serial.print(".");
    delay(50);
    wifiStatus = WiFi.status();
  }
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  // update the RTC data store
  if(channelChange){ // if the channel has changed, we need to update the BSSID and the channel
    Serial.println("New WiFi channel settings discoverd");
    // Copy 6 bytes of BSSID (AP's MAC address), this won't neccesarily change, but I need a place to update it at run, so I'm doing it here.
    memcpy(rtcData.bssid, WiFi.BSSID(), 6); 
    rtcData.channel = WiFi.channel();
  }
  rtcData.battery_voltage = battery_voltage;
  rtcData.temperature = temperature;
  rtcData.humidity = humidity;
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof( rtcData ) - 4); // cacluate checksum.
  ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));

 
}

/**
 * Connects to the mqtt server specified in the settings header.
 */
void mqtt_connect(){
  // client.setWill(will_topic, "offline", true, 0);
  // client.setOptions(60, true, wait);
  client.begin(broker, network);
  Serial.print("Connecting to MQTT Broker: ");
  Serial.println(broker);
  while(!client.connect(client_id, mqtt_user, mqtt_pass)){
    Serial.print(".");
    delay(500);
  }
  // client.publish(will_topic, "online", true, 0); // birth message
  Serial.println("Connected to MQTT!");
}

/**
 * Calculates the crc value of the rtcData.
 */
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while(length--) {
    uint8_t c = *data++;
    for(uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if(c & i) {
        bit = !bit;
      }

      crc <<= 1;
      if(bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

