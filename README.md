# NodeMCU based temperature and humidity sensor using MQTT

This was built using an ESP8266 and a DHT22.

Because this sensor is compatiable with MQTT, it is also compatible
with the open source home automation platform [Home Assistant](https://www.home-assistant.io/).

In order to use this software, you will need to install the following libraries:
1. MQTT by Joel Gaehwiler
2. DHT by Mark Ruys

These libraries are included as dependencies in the .pio file.

It should also be noted that this was developed using Visual Studio Code with the [PlatformIO](https://platformio.org/) addon.
I reccomend you use platformio since this project was written using its dev style, but it should
work fine if you would like to use the Arduino IDE. You will need to modify some file names and such to fit what
the Arduino IDE requires.

## Usage

This solution is pretty much plug and play. You will need to create a settings.h file containing the following:

```c++
// General Settings
const int dht_pin = 4;
const int wait = 300e6;        // this is 5 minutes

// WiFi Settings
const char ssid[] = "";
const char pass[] = "";
IPAddress ip(192, 168, 0, 1);  // sets a static IP, more power effecient
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT Settings
const char client_id[] = "";  // unique id for client
const char broker[] = "";     // IP of broker
const char mqtt_user[] = "";
const char mqtt_pass[] = "";
const char will_topic[] = "";
const char publish_topic_temperature[] = "";
const char publish_topic_humidity[] = "";
const char publish_topic_battery[] = "";

```

You will also need to edit the dht_pin on line 14 to reflect your own usage.
