# Dyson UART Parser
Dyson Air Purifier Home Assistant Integration

![Downloads](https://img.shields.io/github/downloads/HeatseekerXXX/Dyson_UART_parser/total) ![Contributors](https://img.shields.io/github/contributors/HeatseekerXXX/Dyson_UART_parser?color=dark-green) ![Issues](https://img.shields.io/github/issues/HeatseekerXXX/Dyson_UART_parser) ![License](https://img.shields.io/github/license/HeatseekerXXX/Dyson_UART_parser)


## About the project
<div align="center"> <img src="https://dyson-h.assetsadobe2.com/is/image/content/dam/dyson/images/products/hero/369806-01.png?$responsive$&cropPathE=desktop&fit=stretch,1&wid=200"></div>

Having a Dyson Purifier like TP07 **without** Wi-Fi module installed? Willing to integrate it into your Home Assistant? This project can help you. Dyson TP07 has a 4-pin connector, installed on the mainboard, that is made for diagnostics purposes. It can be easily accessed using only a torx screwdriver. The connector provides various data transferring via UART protocol.
Dyson UART Parser reads UART data retrieving all valuable information an providing an interface to Home Assistant server.

## New features
Using IR imitation module now you can control Dyson Purifier via sending IR commands. Controlling interface is provided by Home Assistant integration. User needs to make changes in HA configuration files as mention below.
## Getting started
### Hardware requirements
* ESP32 | ESP8266 module
* 4-pin connector or wires and soldering iron
* additional single wire to enable IR command functions

![ESP](https://www.espressif.com/sites/default/files/dev-board/ESP32-DevKitC%28ESP32-WROVER-E%29_0.png)
<img src="https://cdn-shop.adafruit.com/970x728/4924-00.jpg" width="200" height="200" />

### Software requirements
* ESP-IDF installed
* [esp32-wifi-manager](https://github.com/tonyp7/esp32-wifi-manager) component
* [cJSON](https://github.com/DaveGamble/cJSON) component
### Installation
1. Connect ESP module to UART port of Dyson Purifier
<img src="https://raw.githubusercontent.com/HeatseekerXXX/Dyson_UART_parser/89a82205fedfe304d66252f21ff204d3248e9697/Img/ESP_Dyson_connect.svg" width="200"/>
Various ESP DevKits have different pin alignment. Be sure making correct connection.
Solder a single wire to correct test point on purifier's main board as shown below.
<img src="https://raw.githubusercontent.com/HeatseekerXXX/Dyson_UART_parser/89a82205fedfe304d66252f21ff204d3248e9697/Img/IR%20line3.jpg" width="400"/>
Connect the wire to D4 ESP32 pin through 100R resistor (optional safety precaution).

2. Clone the repository where you want it to be.
```bash
git clone https://github.com/HeatseekerXXX/Dyson_UART_parser.git
```
3. Navigate to main folder
```bash
cd main
```
4. Compile and load the code to ESP module
```bash
idf.py build flash monitor
```
5. After rebooting temporary Wi-Fi access point *"esp32"* will be created. Pass: *"esp32pwd"*. After successful connection to AP you will be redirected to captive portal. Choose you home Wi-Fi access point and enter credentials. Remember DHCP assigned IP address.
6. On the Home Assistant side main configuration file has to be edited to create new sensors. Add the lines from [HA_Config.yaml](https://github.com/HeatseekerXXX/Dyson_UART_parser/blob/master/HA_config.yaml) and [HA_automation.yaml](https://github.com/HeatseekerXXX/Dyson_UART_parser/blob/master/HA_automation.yaml) to the end of *configuration.yaml* and *automations.yaml* **changing the IP  to ESP IP address.**
7. Create new Entity module in HomeAssistant dashboard

![HAimg](https://github.com/HeatseekerXXX/Dyson_UART_parser/blob/master/Img/HA.jpg?raw=true)

## Precompiled image
If you are going to use ESP32-WROOM32 devboard just download precompiled version of Dyson UART Parser and upload all images using [Flash Download Tool](https://www.espressif.com/en/support/download/other-tools). The archive with images stored in **compile** folder.
## License
Distributed under the MIT License. See [LICENSE](https://github.com/HeatseekerXXX/Dyson_UART_parser/blob/master/LICENSE.md) for more information.
