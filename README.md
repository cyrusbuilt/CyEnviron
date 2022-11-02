# CyEnviron

Environmental Monitoring System (ESP-8266 based)

![Build Status](https://github.com/cyrusbuilt/CyEnviron/actions/workflows/ci.yml/badge.svg)

CyEnviron is an indoor environmental monitoring system designed to be integrated with home-automation systems. This repository contains the firmware, schematics, PCB design files, and OpenHAB3 integration.

## Theory of Operation

Once every 5 seconds (by default), CyEnviron will take the following measurements and then publish a status payload (in JSON format) to it's status topic in [MQTT](https://mqtt.org/):

- Temperature
- Humdity
- Barometric Pressure
- Altitude above sealevel
- Gas Resistance
- Ambient light level
- Indoor Air Quality (IAQ)
- Air Quality Index (AQI)
- CO2 Equivalent
- Breath Volatile Organic Compounds (Breath VOC)
- Dew Point

Ambient light level is sampled using an [LDR/CDS Photocell](https://www.electronics-notes.com/articles/electronic_components/resistors/light-dependent-resistor-ldr.php). All other measurements are sampled using an [Adafruit BME688/BME680](https://www.adafruit.com/product/5046). At present, the more advanced features such as integrated AI (BME688), IAQ (Indoor Air Quality), and VOC (Volatile Organic Compound) sampling are not yet supported (but will be in a future firmware release). Specifically, this project was designed to use the BME688 module, but the BME680 *can* be used instead (especially since the more advanced features of the 688 are not yet implemented).  In a future release, I also intend to support alarm thresholds for certain values. Once the threshold value(s) are exceeded an audible and visible alarm will trigger and also be published as part of the status payload.

## IMPORTANT NOTE

It is very important that you *DO NOT* touch the BME680/BME688 sensor during assembly so as not to contaminate the sensor. I'm not currently aware of any way to decontimnate it. Once contaminated, you will no longer get accurate readings.

## Project Structure

This is a [PlatformIO](https://platformio.org/) project, so please refer to the PlatformIO documentation for building,

```
|-- .github          # Continuous Integration stuff
|-- data             # Configuration data. Anything in here gets flashed when uploading a filesystem image.
|-- include          # Contains C++ header files for the firmware.
|-- openhab          # Contains OpenHAB3 integration files.
|-- schematics       # Contains schematics (EasyEDA and PDF), PCB design (EasyEDA and PDF), BOM, and gerbers.
|-- src              # Contains C++ source files for the firmware.
|-- LICENSE          # MIT open source license.
|-- platformio.ini   # PlatformIO project configuration file.
|-- README.md        # This file.
```

## Configuration

The config.h file contains default configuration directives. These options are the hardcoded default values that the system will initially boot with. However, if there is a config.json file present in the root of the SPIFFS file system, then its values will override the defaults. Here we'll discuss each option:

- DEBUG: Uncomment this define to generate additional debug output in the console.
- ENABLE_OTA: If you do not wish to support OTA updates, just comment this define.
- ENABLE_MDNS: If you do not wish to support [MDNS](https://tttapa.github.io/ESP8266/Chap08%20-%20mDNS.html), comment this define.
- CONFIG_FILE_PATH: This is the path to the configuration file. The default is /config.json. The values in this file will override these defaults.
- DEFAULT_SSID: Change this line to reflect the SSID of your WiFi network.
- DEFAULT_PASSWORD: Change this line to reflect the password to your WiFi network.
- CLOCK_TIMEZONE: The GMT offset for your timezone (EST [America/New York] is -4 when observing DST. It's -5 when not.)
- SERIAL_BAUD: While it is not recommended, you can change the BAUD rate of the serial port here.
- CHECK_WIFI_INTERVAL: The interval (in milliseconds) to check to make sure the device is still connected to WiFi, and if not attempt to reconnect. Default is 30 seconds.
- CHECK_SENSORS_INTERVAL: The interval (in millisceonds) to check the water depth.
- CLOCK_SYNC_INTERVAL: The interval (in milliseconds) to try to resync the clock with NTP. Default is 1 hour.
- DEVICE_NAME: This essentially serves as the host name of the device on the network (default is CYENVIRON).
- CHECK_MQTT_INTERVAL: The interval (in milliseconds) to check connectivity to the MQTT broker. If the connection is lost, a reconnect will occur. The default value is 35 seconds.
- MQTT_TOPIC_STATUS: The MQTT status topic to publish device status messages to. Default is 'cysump/status'.
- MQTT_TOPIC_CONTROL: The MQTT control topic to subscribe to for control messages. Default is 'cysump/control'.
- MQTT_BROKER: The hostname or IP of the MQTT broker.
- MQTT_PORT: The port on the MQTT broker to connect to. The default is 8883 (default port for MQTT over TLS).
- OTA_HOST_PORT: Defines the port to listen for OTA updates on. This option is ignored if ENABLE_OTA is disabled.
- OTA_PASSWORD: The password used to authenticate with the OTA server. This option is ignored if ENABLE_OTA is disabled.
- ip: The default IP address. By default, this devices boots with a static IP configuration. The default IP is 192.168.0.202. You can change that here if you wish.
- gw: The default gateway address. The current default is 192.168.0.1. You can change that here if you wish.
- sm: The subnet mask. By default, it is 255.255.255.0, but you can change that here if need be.

To override the default configuration options, you need to upload a filesystem image containing a file named 'config.json' in the root of the SPIFFS filesystem. The file should like something like this:

```json
{
    "hostname": "CLCONTROLLER",
    "useDHCP": false,
    "ip": "192.168.0.202",
    "gateway": "192.168.0.1",
    "subnetMask": "255.255.255.0",
    "dnsServer": "192.168.0.1",
    "wifiSSID": "your_ssid_here",
    "wifiPassword": "your_password_here",
    "otaPort": 8266,
    "otaPassword": "your_ota_password_here",
    "mqttBroker": "your_mqtt_broker_here",
    "mqttPort": 8883,
    "mqttControlChannel": "clcontroller/control",
    "mqttStatusChannel": "clcontroller/status",
    "mqttUsername": "your_mqtt_username",
    "mqttPassword": "your_mqtt_password",
	"timezone": -4
}
```

This configuration file is pretty self explanatory and one is included in the source. The file *MUST* be located in the "data" directory located in the root of the project in order to be picked up by the flash uploader (either via Serial or OTA). Each of the options in the configuration file are self-explanatory and match up with the hard-coded default settings mentioned above. If this file is not present when the firmware boots, a new file will be created and populated with the hardcoded defaults. These defaults can then be changed via the fail-safe menu and saved. You'll notice a couple of additional options in the config.json file not present in config.h. They are as follows:

- mqttUsername: If you have enabled password authentication on your MQTT broker, provide the username here.
- mqttPassword: If you have enabled password authentication on your MQTT broker, provide the password for the username above. If *both* the username and password are provided, then CLController will attempt to connect to the broker with the provided credentials; Otherwise, it will attempt to connect without credentials.
- mqttControlChannel: The MQTT control topic to subscribe to in order to receive device commands.
- mqttStatusChannel: The MQTT status topic to publish device status to.

## Getting Started

After you've configured everything the way you need it (as discussed above), build the firmware by either clicking "Build" in the PlatformIO tasks menu, or by opening a terminal to the project directory and typing:

```bash
> pio run
```

NOTE: The first time you flash the Huzzah, you need to do so over serial (since the OTA code isn't there yet), but subsequent uploads can be done via OTA if configured properly.

The next thing to do is connect the Huzzah to your computer using an FTDI cable like [this one](https://www.adafruit.com/product/70?gclid=EAIaIQobChMIm7-50ZiZ5AIVlIvICh284QPxEAQYBCABEgJkcPD_BwE) and then configure the port in platformio.ini like so:

```ini
[env:huzzah]
monitor_speed = 115200
monitor_port = /dev/cu.usbserial-AL05HSL2  ; Change this to match your port if necessary
```

With the above mentioned FTDI cable attached to my MacBook, the port appears as it does in the config file above (usually PlatformIO is pretty good about auto-detecting the port for you).

Now all you have to do is flash the firmware onto the Huzzah. You can do this by first pressing and holding the "GPIO" button and then press the "reset" button and let go of both on the Huzzah to put it into flash mode (this is not necessary when you flash via OTA), then click the "Upload and Monitor" task in the PlatformIO menu or by typing the following into the terminal:

```bash
> pio run --target upload --target monitor
```

Once complete, press the "reset" button on the Huzzah. You should see the device boot up in the serial console. Now put the device back into flash mode.  Configure the config.json file as needed and click the "Upload File System Image" task in the PlatformIO menu. When finished, press the "reset" button again and the device will reboot and come back up with your new configuration settings. Of course, to monitor the console, you can click the "Monitor" task from the PlatformIO menu or run the following command:

```bash
> pio run --target monitor
```

## OTA Updates

If you wish to be able to upload firmware updates Over-the-Air, then besides leaving the ENABLE_OTA option uncommented, you will also need to uncomment all the upload_* lines in platformio.ini, and change the line 'upload_port = ' line to reflect the IP of the device and the line '--auth=' to reflect whatever OTA_PASSWORD is set to. Then when you click "upload" from the PlatformIO tasks (or if you execute 'pio run --target upload' from the command line) it should upload directly over WiFi and once completed, the device will automatically flash itself and reboot with the new version. If you wish to upload a new configuration file, you can also do this via OTA. Assuming the above-mentioned settings are configured, you can then click "Upload File System Image" from the PlatformIO project tasks.

## Serial Console

If the device ever fails to connect to WiFi or if you press the 'I' key on your keyboard while in the serial console, normal operation of the device is suspended and the device will fall into a 'fail-safe' mode and present the user with a command menu, which can be use to re-configure the device, reboot, manually read sensor data, etc.

## Integration

As previously mentioned, this was intended to be integrated with 3rd-party systems (ie. Home Automation). While my personal preference is [OpenHAB](https://www.openhab.org/), since CyEnviron communicates over MQTT, it can be integrated with any other system that can use MQTT as well (ie. [Home Assistant](https://www.home-assistant.io/), [Node-RED](https://nodered.org/), etc). While the integration files included in this repo are for OpenHAB, you should be able to fairly easily create your own configuration files to integrate with other systems.