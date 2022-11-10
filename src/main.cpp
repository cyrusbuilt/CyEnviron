/**
 * @file main.cpp
 * @author Cyrus Brunner (cyrusbuilt at gmail dot com)
 * @brief Firmware for the CyEnviron ESP8266-based Evironmental sensor module.
 * This module samples temperature, humidity, barometric pressure, gas
 * resistance and ambient light level. It also derives altitude above
 * sea-level and eventually will provide measurements for IAQ (Indoor Air
 * Quality) and VOC (Volatile Organic Compounds) which can be used to detect
 * the presence of dangerous gasses such as Carbon Monoxide, etc.
 * @version 1.4
 * @date 2022-09-19
 * 
 * @copyright Copyright (c) 2022 Cyrus Brunner
 * 
 */

#ifndef ESP8266
    #error This firmware is only compatible with ESP8266 controllers.
#endif

#include <Arduino.h>
#include <FS.h>
#include <time.h>
#include <TZ.h>
#include <Wire.h>
#include "config.h"
#include "ArduinoJson.h"
#include "ESPCrashMonitor.h"
#include "Console.h"
#include "Buzzer.h"
#include "LED.h"
#include "PubSubClient.h"
#include "ResetManager.h"
#include "TaskScheduler.h"
#include "TelemetryHelper.h"
#include "bsec.h"
#include "Environment.h"
#include "LDR.h"

#define FIRMWARE_VERSION "1.4"

// Pin definitions
#define PIN_WIFI_LED 16
#define PIN_LDR 12
#define PIN_ALARM_LED 15
#define PIN_ALARM_BUZZER 14

// Forward declarations
void onCheckWifi();
void onCheckMqtt();
void onSyncClock();
void onReadSensors();
void onMqttMessage(char* topic, byte* payload, unsigned int length);

// Global vars.
#ifdef ENABLE_MDNS
    #include <ESP8266mDNS.h>
    MDNSResponder mdns;
#endif
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Task tCheckWiFi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWifi);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Task tClockSync(CLOCK_SYNC_INTERVAL, TASK_FOREVER, &onSyncClock);
Task tReadSensors(CHECK_SENSORS_INTERVAL, TASK_FOREVER, &onReadSensors);
Scheduler taskMan;
LED wifiLED(PIN_WIFI_LED, NULL);
LED alarmLED(PIN_ALARM_LED, NULL);
Buzzer alarmBuzzer(PIN_ALARM_BUZZER, NULL, "alarm");
Bsec iaqSensor;
LDR ldr(PIN_LDR);
config_t config;
env_t envData;
bool filesystemMounted = false;
volatile SystemState sysState = SystemState::BOOTING;

/**
 * @brief Get the current date/time as string (adjusted for local time and DST).
 * 
 * @return String The current time string.
 */
String getTimeInfo() {
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    String result = String(asctime(timeinfo));
    result.replace("\n", "");
    return result;
}

/**
 * @brief Synchronize the local system clock via NTP. Note: This does not take DST
 * into account. Currently, you will have to adjust the CLOCK_TIMEZONE define
 * manually to account for DST when needed.
 */
void onSyncClock() {
    wifiLED.on();
    configTzTime(TZ_America_New_York, "pool.ntp.org");

    Serial.print("INIT: Waiting for NTP time sync...");
    delay(500);
    while (!time(nullptr)) {
        wifiLED.off();
        ESPCrashMonitor.iAmAlive();
        Serial.print(F("."));
        delay(500);
        wifiLED.on();
    }
    
    wifiLED.off();
    Serial.println(F(" DONE"));
    Serial.print(F("INFO: Current time: "));
    Serial.println(getTimeInfo());
}

/**
 * @brief Publishes the current system status as a JSON payload to the MQTT
 * status topic, if connected. This will also flash the WiFi activity LED.
 */
void publishSystemState() {
    if (mqttClient.connected()) {
        wifiLED.on();

        // TODO Make reported measurements configurable (imperial vs metric, or both).
        DynamicJsonDocument doc(400);
        doc["clientId"] = config.hostname.c_str();
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;
        doc["altitudeFeet"] = envData.altitudeF;
        doc["gasKohms"] = envData.gasKohms;
        doc["humidity"] = envData.humidity;
        doc["pressureHpa"] = envData.pressureHpa;
        doc["tempF"] = envData.tempF;
        doc["brightness"] = envData.brightness;
        doc["lightLevel"] = (uint8_t)envData.lightLevel;
        doc["alarm"] = envData.alarmCondition;
        doc["iaq"] = envData.iaq;
        doc["co2Equivalent"] = envData.co2Equivalent;
        doc["breathVoc"] = envData.breathVoc;
        doc["dewPoint"] = envData.dewPointF;
        doc["aqi"] = (uint8_t)envData.aqi;
        doc["lastUpdate"] = envData.lastUpdate;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicStatus.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        wifiLED.off();
    }
}

/**
 * @brief Publishes a device disovery packet to the configured discovery topic.
 */
void publishDiscoveryPacket() {
    if (mqttClient.connected()) {
        wifiLED.on();

        DynamicJsonDocument doc(250);
        doc["name"] = config.hostname;
        doc["class"] = DEVICE_CLASS;
        doc["statusTopic"] = config.mqttTopicStatus;
        doc["controlTopic"] = config.mqttTopicControl;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing discovery packet: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicDiscovery.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        wifiLED.off();
    }
}

/**
 * @brief Triggers an alarm condition. This will set the system state, turn the
 * alarm LED and buzzer on, and then publish the system state.
 * 
 * @param reason The reason for the alarm.
 */
void raiseAlarm(const char* reason) {
    Serial.print(F("ALARM: !!! "));
    Serial.print(reason);
    Serial.println(F(" !!!"));
    alarmLED.on();
    alarmBuzzer.on();
    sysState = SystemState::ALARM;
    envData.alarmCondition = strdup(reason);
    publishSystemState();
}

/**
 * @brief Clears the alarm state, turns the alarm LED and buzzer off, then
 * publishes the status update.
 */
void clearAlarm() {
    Serial.println(F("INFO: Alarm condition cleared."));
    alarmBuzzer.off();
    alarmLED.off();
    sysState = SystemState::NORMAL;
    envData.alarmCondition = "";
    publishSystemState();
}

/**
 * @brief Silences the audible alarm, but does not clear the alarm state.
 */
void silenceAlarm() {
    if (sysState == SystemState::ALARM) {
        Serial.println(F("WARN: Alarm silenced."));
        alarmBuzzer.off();
    }
}

/**
 * @brief Checks for alarm conditions. This will clear an active alarm if the
 * alarm conditions drop below threshold or raise an alarm if conditions go
 * above threshold. Trigger an alarm causes a number of things to happen:
 * 1) Alarm buzzer turns on.
 * 2) Alarm LED turns on.
 * 3) System state changes to ALARM.
 * 4) alarmCondition in env data set to reason for alarm.
 * 5) Updated state is then published to MQTT status topic.
 */
void checkAlarm() {
    switch (envData.aqi) {
        case AQI::EXCELLENT:
        case AQI::GOOD:
        case AQI::LIGHTLY_POLLUTED:
            if (sysState == SystemState::ALARM) {
                clearAlarm();
            }
            break;
        case AQI::MODERATELY_POLLUTED:
        case AQI::HEAVILY_POLLUTED:
        case AQI::SEVERELY_POLLUTED:
        case AQI::EXTREME_POLLUTION:
            if (sysState != SystemState::ALARM) {
                raiseAlarm("High pollution level!");
            }
            break;
        default:
            break;
    }

    // TODO CO2 levels factor in to the IAQ/AQI level, but do we want a separate check for that?
    // NOTE acceptable CO2 range is 400 - 650.

    // TODO should we have an alarm for temperature/humidity?
}

/**
 * @brief Checks that status of the BME680/688 sensor and reports any errors
 * or warnings detected.
 */
void checkIaqSensorStatus() {
    String output = "";
    if (iaqSensor.stabStatus != BSEC_OK) {
        if (iaqSensor.stabStatus < BSEC_OK) {
            output = "ERROR: BSEC error code: ";
        }
        else {
            output = "WARN: BSEC warn code: ";
        }

        output += String(iaqSensor.status);
        Serial.println(output);
    }

    if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
            output = "ERROR: BME680/688 error code: ";
        }
        else {
            output = "WARN: BME680/688 warn code: ";
        }

        output += String(iaqSensor.bme680Status);
        Serial.println(output);
    }
}

/**
 * @brief Read all the sensor data and publishes the updated state data.
 */
void onReadSensors() {
    Serial.println(F("INFO: Reading sensor data ..."));
    if (!iaqSensor.run()) {
       checkIaqSensorStatus();
       return;
    }

    ldr.performReading();

    envData.altitudeM = EnvUtils::getAltitude(iaqSensor.pressure, AVG_PRESSURE_SEALEVEL_HPA);
    envData.altitudeF = envData.altitudeM * 3.281;
    envData.gasKohms = iaqSensor.gasResistance / 10000;
    envData.humidity = iaqSensor.humidity;
    envData.pressureHpa = iaqSensor.pressure / 100.0;
    envData.tempC = iaqSensor.temperature;
    envData.tempF = EnvUtils::convertTempCtoF(envData.tempC);
    envData.iaq = iaqSensor.staticIaq;
    envData.co2Equivalent = iaqSensor.co2Equivalent;
    envData.breathVoc = iaqSensor.breathVocEquivalent;
    envData.dewPointC = EnvUtils::getDewPoint(envData.tempC, envData.humidity);
    envData.dewPointF = EnvUtils::convertTempCtoF(envData.dewPointC);
    envData.aqi = EnvUtils::getAQI(envData.iaq);
    envData.brightness = ldr.readSensorBrightness();
    envData.lightLevel = ldr.getBrightnessLevel();
    envData.lastUpdate = getTimeInfo();

    checkAlarm();
    publishSystemState();
}

/**
 * @brief Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
    wifiLED.off();
    sysState = SystemState::NORMAL;
    publishSystemState();
}

/**
 * @brief Prints network information details to the serial console.
 */
void printNetworkInfo() {
    Serial.print(F("INFO: Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("INFO: Gateway: "));
    Serial.println(WiFi.gatewayIP());
    Serial.print(F("INFO: Subnet mask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("INFO: DNS server: "));
    Serial.println(WiFi.dnsIP());
    Serial.print(F("INFO: MAC address: "));
    Serial.println(WiFi.macAddress());
    #ifdef DEBUG
        WiFi.printDiag(Serial);
    #endif
}

/**
 * @brief Perform a soft-reboot of the firmware.
 */
void reboot() {
    Serial.println(F("INFO: Rebooting..."));
    Serial.flush();
    delay(1000);
    ResetManager.softReset();
}

/**
 * @brief Saves configuration data to the config file.
 */
void saveConfiguration() {
    Serial.print(F("INFO: Saving configuration to "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.println(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mount."));
        return;
    }

    StaticJsonDocument<350> doc;
    doc["hostname"] = config.hostname;
    doc["useDhcp"] = config.useDhcp;
    doc["ip"] = config.ip.toString();
    doc["gateway"] = config.gw.toString();
    doc["subnetmask"] = config.sm.toString();
    doc["dnsServer"] = config.dns.toString();
    doc["wifiSSID"] = config.ssid;
    doc["wifiPassword"] = config.password;
    doc["timezone"] = config.clockTimezone;
    doc["mqttBroker"] = config.mqttBroker;
    doc["mqttPort"] = config.mqttPort;
    doc["mqttControlTopic"] = config.mqttTopicControl;
    doc["mqttStatusTopic"] = config.mqttTopicStatus;
    doc["mqttDiscoveryTopic"] = config.mqttTopicDiscovery;
    doc["mqttUsername"] = config.mqttUsername;
    doc["mqttPassword"] = config.mqttPassword;
    #ifdef ENABLE_OTA
        doc["otaPort"] = config.otaPort;
        doc["otaPassword"] = config.otaPassword;
    #endif

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "w");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to open config file for writing."));
        doc.clear();
        return;
    }

    serializeJsonPretty(doc, configFile);
    doc.clear();
    configFile.flush();
    configFile.close();
    Serial.println(F("DONE"));
}

/**
 * @brief Helper method for printing warnings about configuration settings.
 * 
 * @param message The warning message to print.
 */
void printWarningAndContinue(const __FlashStringHelper *message) {
    Serial.println();
    Serial.println(message);
    Serial.print(F("INFO: Continuing... "));
}

/**
 * @brief Sets all the configuration values to their defaults.
 */
void setConfigurationDefaults() {
    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = defHostname;
    config.ip = defaultIp;
    config.mqttBroker = MQTT_BROKER;
    config.mqttPassword = "";
    config.mqttPort = MQTT_PORT;
    config.mqttTopicControl = MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = MQTT_TOPIC_STATUS;
    config.mqttTopicDiscovery = MQTT_TOPIC_DISCOVERY;
    config.mqttUsername = "";
    config.password = DEFAULT_PASSWORD;
    config.sm = defaultSm;
    config.ssid = DEFAULT_SSID;
    config.useDhcp = false;
    config.clockTimezone = CLOCK_TIMEZONE;
    config.dns = defaultDns;
    config.gw = defaultGw;

    #ifdef ENABLE_OTA
        config.otaPassword = OTA_PASSWORD;
        config.otaPort = OTA_HOST_PORT;
    #endif
}

/**
 * @brief Loads the configuration parameters into memory, defaulting the
 * configuration values where necessary.
 */
void loadConfiguration() {
    memset(&config, 0, sizeof(config));

    Serial.print(F("INFO: Loading config file "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.print(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    if (!SPIFFS.exists(CONFIG_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("WARN: Config file does not exist. Creating with default config ..."));
        saveConfiguration();
        return;
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "r");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to open config file. Using default config."));
        return;
    }

    size_t size = configFile.size();
    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
    if (size > freeMem) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: Not enough free memory to load document. Size = "));
        Serial.print(size);
        Serial.print(F(", Free = "));
        Serial.println(freeMem);
        configFile.close();
        return;
    }

    DynamicJsonDocument doc(freeMem);
    DeserializationError error = deserializeJson(doc, configFile);
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Fail to parse config file to JSON. Using default config."));
        configFile.close();
        return;
    }

    doc.shrinkToFit();
    configFile.close();

    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = doc.containsKey("hostname") ? doc["hostname"].as<String>() : defHostname;
    config.useDhcp = doc.containsKey("isDhcp") ? doc["isDhcp"].as<bool>() : false;
    
    if (doc.containsKey("ip")) {
        if (!config.ip.fromString(doc["ip"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.ip = defaultIp;
    }

    if (doc.containsKey("gateway")) {
        if (!config.gw.fromString(doc["gateway"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
        }
    }
    else {
        config.gw = defaultGw;
    }

    if (doc.containsKey("subnetmask")) {
        if (!config.sm.fromString(doc["subnetmask"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid subnet mask in configuration. Falling back to factory default."));
        }
    }
    else {
        config.sm = defaultSm;
    }

    if (doc.containsKey("dns")) {
        if (!config.dns.fromString(doc["dns"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid DNS IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.dns = defaultDns;
    }

    config.ssid = doc.containsKey("wifiSSID") ? doc["wifiSSID"].as<String>() : DEFAULT_SSID;
    config.password = doc.containsKey("wifiPassword") ? doc["wifiPassword"].as<String>() : DEFAULT_PASSWORD;
    config.clockTimezone = doc.containsKey("timezone") ? doc["timezone"].as<uint8_t>() : CLOCK_TIMEZONE;
    config.mqttBroker = doc.containsKey("mqttBroker") ? doc["mqttBroker"].as<String>() : MQTT_BROKER;
    config.mqttPort = doc.containsKey("mqttPort") ? doc["mqttPort"].as<int>() : MQTT_PORT;
    config.mqttTopicControl = doc.containsKey("mqttControlTopic") ? doc["mqttControlTopic"].as<String>() : MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = doc.containsKey("mqttStatusTopic") ? doc["mqttStatusTopic"].as<String>() : MQTT_TOPIC_STATUS;
    config.mqttTopicDiscovery = doc.containsKey("mqttDiscoveryTopic") ? doc["mqttDiscoveryTopic"].as<String>() : MQTT_TOPIC_DISCOVERY;
    config.mqttUsername = doc.containsKey("mqttUsername") ? doc["mqttUsername"].as<String>() : "";
    config.mqttPassword = doc.containsKey("mqttPassword") ? doc["mqttPassword"].as<String>() : "";

    #ifdef ENABLE_OTA
        config.otaPort = doc.containsKey("otaPort") ? doc["otaPort"].as<uint16_t>() : MQTT_PORT;
        config.otaPassword = doc.containsKey("otaPassword") ? doc["otaPassword"].as<String>() : OTA_PASSWORD;
    #endif

    doc.clear();
    Serial.println(F("DONE"));
}

/**
 * @brief Perform a "factory restore" of the configration data. This clears
 * configuration data on flash and reboots after a 5 second countdown. This
 * will force a configuration file to be generated on restart using default
 * values.
 */
void doFactoryRestore() {
    Serial.println();
    Serial.println(F("Are you sure you wish to restore to factory default? (Y/n)"));
    Console.waitForUserInput();
    
    String str = Console.getInputString();
    if (str == "Y" || str == "y") {
        Serial.print(F("INFO: Clearing current config... "));
        if (filesystemMounted) {
            if (SPIFFS.remove(CONFIG_FILE_PATH)) {
                Serial.println(F("DONE"));
                Serial.print(F("INFO: Removed file: "));
                Serial.println(CONFIG_FILE_PATH);

                Serial.print(F("INFO: Rebooting in "));
                for (uint8_t i = 5; i >= 1; i--) {
                    Serial.print(i);
                    Serial.print(F(" "));
                    delay(1000);
                }

                reboot();
            }
            else {
                Serial.println(F("FAIL"));
                Serial.println(F("ERROR: Failed to delete cofiguration file."));
            }
        }
        else {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Filesystem not mounted."));
        }
    }

    Serial.println();
}

/**
 * @brief Scan for available networks and dump each discovered network to the console.
 */
void getAvailableNetworks() {
    ESPCrashMonitor.defer();
    Serial.println(F("INFO: Scanning WiFi networks..."));
    int numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        Serial.print(F("ID: "));
        Serial.print(i);
        Serial.print(F("\tNetwork name: "));
        Serial.print(WiFi.SSID(i));
        Serial.print(F("\tSignal strength:"));
        Serial.println(WiFi.RSSI(i));
    }
    Serial.println(F("----------------------------------"));
}

/**
 * @brief Reconnect to the MQTT broker and resubscribe to the control topic
 * if no longer connected.
 * 
 * @return true If already connected or successfully reconnected.
 * @return false If no longer connected and unable reconnect.
 */
bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
        wifiLED.on();
        Serial.print(F("INFO: Attempting to establish MQTT connection to "));
        Serial.print(config.mqttBroker);
        Serial.print(F(" on port "));
        Serial.print(config.mqttPort);
        Serial.println(F(" ... "));

        bool didConnect = false;
        if (config.mqttUsername.length() > 0 && config.mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(config.hostname.c_str(), config.mqttUsername.c_str(), config.mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(config.hostname.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to topic: "));
            Serial.println(config.mqttTopicControl);
            mqttClient.subscribe(config.mqttTopicControl.c_str());

            Serial.print(F("INFO: Publishing to topic: "));
            Serial.println(config.mqttTopicStatus);

            Serial.print(F("INFO: Discovery topic: "));
            Serial.println(config.mqttTopicDiscovery);
        }
        else {
            String failReason = TelemetryHelper::getMqttStateDesc(mqttClient.state());
            Serial.print(F("ERROR: Failed to connect to MQTT broker: "));
            Serial.println(failReason);
            return false;
        }

        wifiLED.off();
    }

    return true;
}

/**
 * @brief Attempts to reconnect to MQTT if necessary and then publishes the
 * current system state. Otherwise, reports the failure.
 */
void onCheckMqtt() {
    Serial.println(F("INFO: Checking MQTT connections status..."));
    if (reconnectMqttClient()) {
        Serial.println(F("INFO: Successfully reconnected to MQTT broker."));
        publishSystemState();
        publishDiscoveryPacket();
    }
    else {
        Serial.println(F("ERROR: MQTT connection lost and reconnect failed."));
        Serial.print(F("INFO: Retrying connection in "));
        Serial.print(CHECK_MQTT_INTERVAL % 1000);
        Serial.println(F(" seconds ..."));
    }
}

/**
 * @brief Handles control requests recieved from the MQTT control topic and
 * then publishes the current system state. This will execute any valid
 * command it received and ignores any commands that are unrecognized or not
 * valid in the current state.
 * 
 * @param cmd The command to process.
 */
void handleControlRequest(ControlCommand cmd) {
    if (sysState == SystemState::DISABLED && cmd != ControlCommand::ENABLE) {
        // THOU SHALT NOT PASS!!! 
        // We can't process this command because we are disabled.
        Serial.print(F("WARN: Ingoring command "));
        Serial.print((uint8_t)cmd);
        Serial.print(F(" because the system is currently disabled."));
        return;
    }

    switch (cmd) {
        case ControlCommand::ENABLE:
            Serial.println(F("INFO: Enabling system."));
            sysState = SystemState::NORMAL;
            break;
        case ControlCommand::DISABLE:
            Serial.println(F("WARN: Disabling system."));
            sysState = SystemState::DISABLED;
            break;
        case ControlCommand::REBOOT:
            reboot();
            break;
        case ControlCommand::REQUEST_STATUS:
            break;
        case ControlCommand::SILENCE_ALARM:
            silenceAlarm();
            break;
        default:
            Serial.print(F("WARN: Unknown command: "));
            Serial.println((uint8_t)cmd);
            break;
    }

    publishSystemState();
}

/**
 * @brief Handles incoming messages on the MQTT control topic the device is
 * subscribed to. This will utlimately process valid commands if intended for
 * this device.
 * 
 * @param topic The topic the message was received on. 
 * @param payload The message payload.
 * @param length The payload size.
 */
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print(F("INFO: [MQTT] Message arrived: ["));
    Serial.print(topic);
    Serial.print(F("] "));

    // It's a lot easier to deal with if we just convert the payload
    // to a string first.
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.println(msg);

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg.c_str());
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

    if (doc.containsKey("client_id")) {
        String id = doc["client_id"].as<String>();
        id.toUpperCase();
        if (!id.equals(config.hostname)) {
            Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
            doc.clear();
            return;
        }
    }
    else {
        Serial.println(F("WARN: MQTT message does not contain client ID. Ignoring..."));
        doc.clear();
        return;
    }

    if (!doc.containsKey("command")) {
        Serial.println(F("WARN: MQTT message does not contain a control command. Ignoring..."));
        doc.clear();
        return;
    }

    // When system is the "disabled" state, the only command it will accept
    // is "enable". All other commands are ignored.
    ControlCommand cmd = (ControlCommand)doc["command"].as<uint8_t>();

    doc.clear();
    handleControlRequest(cmd);
}

/**
 * @brief Enter fail-safe mode. This will suspend all tasks, disable sensor reading,
 * and propmpt the user for configuration.
 */
void failSafe() {
    sysState = SystemState::DISABLED;
    publishSystemState();
    ESPCrashMonitor.defer();
    Serial.println();
    Serial.println(F("ERROR: Entering failsafe (config) mode..."));
    taskMan.disableAll();
    wifiLED.on();
    Console.enterCommandInterpreter();
}

/**
 * @brief Initializes the MDNS responder (if enabled).
 */
void initMDNS() {
    #ifdef ENABLE_MDNS
        Serial.print(F("INIT: Starting MDNS responder... "));
        if (WiFi.status() == WL_CONNECTED) {
            ESPCrashMonitor.defer();
            delay(500);

            if (!mdns.begin(config.hostname)) {
                Serial.println(F(" FAILED"));
                return;
            }

            #ifdef ENABLE_OTA
                mdns.addService(config.hostname, "ota", config.otaPort);
            #endif
            Serial.println(F(" DONE"));
        }
        else {
            Serial.println(F(" FAILED"));
        }
    #endif
}

/**
 * @brief Initialize the SPIFFS filesystem.
 */
void initFilesystem() {
    Serial.print(F("INIT: Initializing SPIFFS and mounting filesystem... "));
    if (!SPIFFS.begin()) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to mount filesystem."));
        return;
    }

    filesystemMounted = true;
    Serial.println(F("DONE"));
    setConfigurationDefaults();
    loadConfiguration();
}

/**
 * @brief Initializes the MQTT client.
 */
void initMQTT() {
    Serial.print(F("INIT: Initializing MQTT client... "));
    mqttClient.setServer(config.mqttBroker.c_str(), config.mqttPort);
    mqttClient.setCallback(onMqttMessage);
    mqttClient.setBufferSize(500);
    Serial.println(F("DONE"));
    if (reconnectMqttClient()) {
        delay(500);
        publishSystemState();
    }
}

/**
 * @brief Attempt to connect to the configured WiFi network. This will break
 * any existing connection first.
 */
void connectWifi() {
    if (config.hostname) {
        WiFi.hostname(config.hostname.c_str());
    }

    Serial.println(F("DEBUG: Setting mode..."));
    WiFi.mode(WIFI_STA);
    Serial.println(F("DEBUG: Disconnect and clear to prevent auto connect..."));
    WiFi.persistent(false);
    WiFi.disconnect(true);
    ESPCrashMonitor.defer();

    delay(1000);
    if (config.useDhcp) {
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        WiFi.config(config.ip, config.gw, config.sm, config.gw);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(config.ssid, config.password);
    Serial.println(F("DEBUG: Waiting for connection..."));
    
    const int maxTries = 20;
    int currentTry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (currentTry < maxTries)) {
        ESPCrashMonitor.iAmAlive();
        currentTry++;
        wifiLED.blink(500);
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        // Connection failed. Maybe the AP went down? Let's try again later.
        Serial.println(F("ERROR: Failed to connect to WiFi!"));
        Serial.println(F("WARN: Will attempt to reconnect at scheduled interval."));
    }
    else {
        printNetworkInfo();
    }
}

/**
 * @brief Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to SSID: "));
    Serial.print(config.ssid);
    Serial.println(F("..."));
    
    connectWifi();
}

/**
 * @brief Initializes the OTA update listener if enabled.
 */
void initOTA() {
    #ifdef ENABLE_OTA
        Serial.print(F("INIT: Starting OTA updater... "));
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setPort(config.otaPort);
            ArduinoOTA.setHostname(config.hostname.c_str());
            ArduinoOTA.setPassword(config.otaPassword.c_str());
            ArduinoOTA.onStart([]() {
                // Handles start of OTA update. Determines update type.
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                }
                else {
                    type = "filesystem";
                }

                sysState = SystemState::UPDATING;
                publishSystemState();
                Serial.println("INFO: Starting OTA update (type: " + type + ") ...");
            });
            ArduinoOTA.onEnd([]() {
                // Handles update completion.
                Serial.println(F("INFO: OTA updater stopped."));
            });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
                // Reports update progress.
                wifiLED.blink(100);
                ESPCrashMonitor.iAmAlive();
                Serial.printf("INFO: OTA Update Progress: %u%%\r", (progress / (total / 100)));
            });
            ArduinoOTA.onError([](ota_error_t error) {
                // Handles OTA update errors.
                Serial.printf("ERROR: OTA update error [%u]: ", error);
                switch(error) {
                    case OTA_AUTH_ERROR:
                        Serial.println(F("Auth failed."));
                        break;
                    case OTA_BEGIN_ERROR:
                        Serial.println(F("Begin failed."));
                        break;
                    case OTA_CONNECT_ERROR:
                        Serial.println(F("Connect failed."));
                        break;
                    case OTA_RECEIVE_ERROR:
                        Serial.println(F("Receive failed."));
                        break;
                    case OTA_END_ERROR:
                        Serial.println(F("End failed."));
                        break;
                }
            });
            ArduinoOTA.begin();
            Serial.println(F("DONE"));
        }
        else {
            Serial.println(F("FAIL"));
        }
    #endif
}

/**
 * @brief Callback routine for checking WiFi connectivity.
 */
void onCheckWifi() {
    Serial.println(F("INFO: Checking WiFi connectivity..."));
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WARN: Lost connection. Attempting reconnect..."));
        connectWifi();
        if (WiFi.status() == WL_CONNECTED) {
            initMDNS();
            initOTA();
        }
    }
}

/**
 * @brief Initializes the RS-232 serial interface.
 */
void initSerial() {
    Serial.begin(BAUD_RATE);
    #ifdef DEBUG
        const bool debug = true;
    #else
        const bool debug = false;
    #endif
    Serial.setDebugOutput(debug);
    Serial.println();
    Serial.print(F("CyEnviron v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" booting ..."));
}

/**
 * @brief Initializes the BME680/BME688 sensor.
 */
void initGasSensor() {
    Serial.print(F("INIT: Initializing BME680/688 sensor ..."));
    Wire.begin();
    iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    if (iaqSensor.status != BSEC_OK || iaqSensor.bme680Status != BME680_OK) {
        Serial.println(F("FAIL"));
        checkIaqSensorStatus();
        return;
    }

    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
    };

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    if (iaqSensor.status != BSEC_OK || iaqSensor.bme680Status != BME680_OK) {
        Serial.println(F("FAIL"));
        checkIaqSensorStatus();
        return;
    }

    Serial.println(F("DONE"));
    
    String libVersion = "INIT: BSEC library v";
    libVersion += String(iaqSensor.version.major) + ".";
    libVersion += String(iaqSensor.version.minor) + ".";
    libVersion += String(iaqSensor.version.major_bugfix) + ".";
    libVersion += String(iaqSensor.version.minor_bugfix);
    Serial.println(libVersion);
}

/**
 * @brief Initializes the LDR/CdS sensor.
 */
void initLightSensor() {
    Serial.print(F("INIT: Initializing LDR ..."));
    if (!ldr.begin()) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: LDR/CdS sensor not detected."));
        return;
    }

    Serial.println(F("DONE"));
}

/**
 * @brief Initializes outputs (LEDs and Buzzer).
 */
void initOutputs() {
    Serial.print(F("INIT: Initializing outputs ..."));
    wifiLED.init();
    wifiLED.on();
    alarmLED.init();
    alarmLED.on();
    alarmBuzzer.init();
    alarmBuzzer.off();
    Serial.println(F("DONE"));
}

/**
 * @brief Handler for when the host name is changed from the console.
 * Sets the new hostname and re-initializes MDNS (if enabled) if the
 * host name is different from the currently configured host name.
 * 
 * @param newHostname The new host name.
 */
void handleNewHostname(const char* newHostname) {
    if (strcmp(newHostname, config.hostname.c_str()) == 0) {
        config.hostname = newHostname;
        initMDNS();
    }
}

/**
 * @brief Handler for when the user switches from a static IP to DHCP
 * from the console. If not already in DHCP mode, this will force the
 * config change in memory, then force the WiFi interface to attempt
 * to get an address via DHCP.
 */
void handleSwitchToDhcp() {
    if (config.useDhcp) {
        Serial.println(F("INFO: DHCP mode already set. Skipping..."));
        Serial.println();
    }
    else {
        config.useDhcp = true;
        Serial.println(F("INFO: Set DHCP mode."));
        WiFi.config(0U, 0U, 0U, 0U);
    }
}

/**
 * @brief Handles switching to static IP from the console. This will set the
 * new static IP settings in memory and reconfigure the WiFi interface.
 * 
 * @param newIp The new IP address.
 * @param newSm The new subnet mask.
 * @param newGw The new gateway address.
 * @param newDns The new primary DNS address.
 */
void handleSwitchToStatic(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns) {
    config.ip = newIp;
    config.sm = newSm;
    config.gw = newGw;
    config.dns = newDns;
    Serial.println(F("INFO: Set static network config."));
    WiFi.config(config.ip, config.gw, config.sm, config.dns);
}

/**
 * @brief Handles the command to reconnect to WiFi from the console. This will
 * force an attempted reconnect and if successful, resume normal operation;
 * Otherwise, will drop back to the command menu after reporting the error.
 */
void handleReconnectFromConsole() {
    // Attempt to reconnect to WiFi.
    onCheckWifi();
    if (WiFi.status() == WL_CONNECTED) {
        printNetworkInfo();
        resumeNormal();
    }
    else {
        Serial.println(F("ERROR: Still no network connection."));
        Console.enterCommandInterpreter();
    }
}

/**
 * @brief Handles WiFi configuration command from the console. If either the
 * SSID and/or password are changing, then stores the new settings in memory
 * and applies them, forcing a reconnect.
 * 
 * @param newSsid The new SSID.
 * @param newPassword The new password.
 */
void handleWifiConfig(String newSsid, String newPassword) {
    if (config.ssid != newSsid || config.password != newPassword) {
        config.ssid = newSsid;
        config.password = newPassword;
        connectWifi();
    }
}

/**
 * @brief Handles the save configuration command from the console. This
 * persists all the configuration data currently in memory to the config
 * file stored in flash. If this method isn't called after making config
 * changes, the prior configuration will be restored on next boot.
 */
void handleSaveConfig() {
    saveConfiguration();
    WiFi.disconnect(true);
    onCheckWifi();
}

/**
 * @brief Handles changing the MQTT settings from the console. If any of the
 * values are changing then this will unsubscribe from the current control
 * topic and disconnect from the broker, then apply the config changes in
 * memory, then reconnect to the broker and resubscribe to the control topic.
 * 
 * @param newBroker The new broker hostname or IP value.
 * @param newPort The new port.
 * @param newUsername The new username.
 * @param newPassw The new password.
 * @param newConChan The new control topic.
 * @param newStatChan The new status topic.
 */
void handleMqttConfigCommand(String newBroker, int newPort, String newUsername, String newPassw, String newConChan, String newStatChan) {
    if (config.mqttBroker != newBroker || config.mqttPort != newPort
        || config.mqttUsername != newUsername || config.mqttPassword != newPassw
        || config.mqttTopicControl != newConChan || config.mqttTopicStatus != newStatChan) {
        mqttClient.unsubscribe(config.mqttTopicControl.c_str());
        mqttClient.disconnect();

        config.mqttBroker = newBroker;
        config.mqttPort = newPort;
        config.mqttUsername = newUsername;
        config.mqttPassword = newPassw;
        config.mqttTopicControl = newConChan;
        config.mqttTopicStatus = newStatChan;

        initMQTT();
        Serial.println();
    }
}

/**
 * @brief Initializes the console interface (CLI).
 */
void initConsole() {
    Serial.print(F("INIT: Initializing console... "));

    Console.setHostname(config.hostname);
    Console.setMqttConfig(
        config.mqttBroker,
        config.mqttPort,
        config.mqttUsername,
        config.mqttPassword,
        config.mqttTopicControl,
        config.mqttTopicStatus
    );
    Console.onRebootCommand(reboot);
    Console.onScanNetworks(getAvailableNetworks);
    Console.onFactoryRestore(doFactoryRestore);
    Console.onHostnameChange(handleNewHostname);
    Console.onDhcpConfig(handleSwitchToDhcp);
    Console.onStaticConfig(handleSwitchToStatic);
    Console.onReconnectCommand(handleReconnectFromConsole);
    Console.onWifiConfigCommand(handleWifiConfig);
    Console.onSaveConfigCommand(handleSaveConfig);
    Console.onMqttConfigCommand(handleMqttConfigCommand);
    Console.onConsoleInterrupt(failSafe);
    Console.onResumeCommand(resumeNormal);

    Serial.println(F("DONE"));
}

/**
 * @brief Initializes the task manager and all recurring tasks.
 */
void initTaskManager() {
    Serial.print(F("INIT: Initializing task scheduler... "));

    taskMan.init();
    taskMan.addTask(tCheckWiFi);
    taskMan.addTask(tCheckMqtt);
    taskMan.addTask(tClockSync);
    taskMan.addTask(tReadSensors);
    
    tCheckWiFi.enableDelayed(30000);
    tCheckMqtt.enableDelayed(1000);
    tClockSync.enable();
    tReadSensors.enable();
    Serial.println(F("DONE"));
}

/**
 * @brief Initializes the crash monitor and dump any previous crash data to
 * the serial console.
 */
void initCrashMonitor() {
    Serial.print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    Serial.println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
}

/**
 * @brief Initial startup routine. This method is called *once* when the
 * MCU first boots and executes all the initialization routines.
 */
void setup() {
    initSerial();
    initCrashMonitor();
    initOutputs();
    initLightSensor();
    initGasSensor();
    initFilesystem();
    initWiFi();
    initMDNS();
    initOTA();
    initMQTT();
    initTaskManager();
    initConsole();
    Serial.println(F("INFO: Boot sequence complete."));
    sysState = SystemState::NORMAL;
    wifiLED.off();
    alarmLED.off();
    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

/**
 * @brief This is the main program loop. This method is called by the Arduino
 * Framework endlessly while the firmware is running and executes all the
 * subroutines for keeping the watchdog fed, checking for console interrupt,
 * executing tasks and handling things like MQTT messages, OTA update requests,
 * and MDNS updates.
 */
void loop() {
    ESPCrashMonitor.iAmAlive();
    Console.checkInterrupt();
    taskMan.execute();
    #ifdef ENABLE_MDNS
        mdns.update();
    #endif
    #ifdef ENABLE_OTA
        ArduinoOTA.handle();
    #endif
    mqttClient.loop();
}