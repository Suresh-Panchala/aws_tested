#include <SSLClient.h>
#define TINY_GSM_MODEM_BG96
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "certs.h"
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#define SerialAT Serial2

TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
SSLClient secureclient(&gsmClient);
PubSubClient mqtt(secureclient);
ModbusMaster node_1;
#define MODEM_PWKEY 4
#define MAX485_DE_1 5
#define RX_PIN_1 19  
#define TX_PIN_1 18  

SoftwareSerial RS485Serial(RX_PIN_1, TX_PIN_1);  

const char MQTT_HOST[] = "a38nh0uqz51few-ats.iot.ap-southeast-2.amazonaws.com";
const char topic_publish[] = "test/DevicePh";
const char topic_subscribe[] = "test/Device";
const char connect_topic[] = "test/Device";

// Global variables for measurements
volatile float mACurrent, mACurrent1, temp_1_1, temp_1_2, pH_1, conductivity_1, temp_2_1, temp_2_2, pH_2, conductivity_2;

// Watchdog timer settings
unsigned long lastModemCheck = 0;
const unsigned long MODEM_WATCHDOG_INTERVAL = 30000; // 30 seconds

void preTransmission() {
    digitalWrite(MAX485_DE_1, 1);
}

void postTransmission() {
    digitalWrite(MAX485_DE_1, 0);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < len; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void reconnect() {
    while (!mqtt.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqtt.connect("PH_DEVICE")) {
            Serial.println("connected");
            mqtt.subscribe(topic_subscribe);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqtt.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void checkGPRSConnection() {
    if (!modem.isGprsConnected()) {
        Serial.println("GPRS not connected. Attempting to connect...");
        if (modem.gprsConnect("etisalat.ae", "", "")) {
            Serial.println("GPRS Connected");
        } else {
            Serial.println("Failed to connect GPRS");
        }
    }
}

void resetModem() {
    Serial.println("Resetting modem...");
    modem.restart();  // Restart the modem
    checkGPRSConnection(); // Check GPRS after restart
}

void setup() {
    Serial.begin(115200);
    SerialAT.begin(115200);

    // Set up modem
    pinMode(MODEM_PWKEY, OUTPUT);
    digitalWrite(MODEM_PWKEY, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWKEY, LOW);
    delay(2000);
    modem.restart();

    secureclient.setCACert(cacert);
    secureclient.setCertificate(clientcert);
    secureclient.setPrivateKey(clientkey);
    mqtt.setServer(MQTT_HOST, 8883);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(1024);

    while (!modem.isNetworkConnected()) {
        Serial.println("Network not available");
        modem.waitForNetwork();
    }

    delay(2000);
    checkGPRSConnection();  // Initial GPRS check  
    pinMode(MAX485_DE_1, OUTPUT);
    digitalWrite(MAX485_DE_1, 0);

    RS485Serial.begin(9600);
    node_1.begin(1, RS485Serial);
    node_1.preTransmission(preTransmission);
    node_1.postTransmission(postTransmission);
}

void loop() {
    // Monitor free heap
    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
    // Check GPRS connection in the main loop
    checkGPRSConnection();

//     Check if the modem is responding
    if (millis() - lastModemCheck >= MODEM_WATCHDOG_INTERVAL) {
        lastModemCheck = millis();
        if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
            resetModem();  // Reset the modem if it's not responding
        }
    }
    // Attempt to reconnect if MQTT is not connected
    if (!mqtt.connected()) {
        reconnect();
    }

    bus_1();
    delay(5000);


    // Publish data every 1 minute (60000 milliseconds)
    static unsigned long lastPublish = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastPublish >= 60000) { // 1 minute
        publishData();
        lastPublish = currentMillis;
    }
    // Call MQTT loop to handle incoming messages
    mqtt.loop();
}

void publishData() {
    String time = modem.getGSMDateTime(DATE_FULL);
    Serial.println("Publishing data...");

    StaticJsonDocument<300> doc; 
    doc["deviceId"] = "pH_Conductivity_Device_outh";
    doc["timestamp"] = time;
    JsonObject data = doc.createNestedObject("data");
    data["mACurrent_1"]  = mACurrent;
    data["mACurrent_2"]  = mACurrent1;
    data["temp_1_1"]     = temp_1_1;
    data["temp_1_2"]     = temp_1_2;
    data["pH_1"]         = pH_1;
    data["conductivity_1"] = conductivity_1;
    data["temp_2_1"]     = temp_2_1;
    data["temp_2_2"]     = temp_2_2;
    data["pH_2"]         = pH_2;
    data["conductivity_2"] = conductivity_2;

    char jsonBuffer[512];
    size_t n = serializeJson(doc, jsonBuffer);
    bool success = mqtt.publish(topic_publish, jsonBuffer); 
    if (success) {
        Serial.println("Data published successfully");
    } else {
        Serial.println("Data publish failed");
    }
}

void bus_1() {
    uint8_t result;
    uint16_t data[8]; // Array to hold the read data
    result = node_1.readInputRegisters(0x0000, 8);
    Serial.println("Form Bus 1");

    if (result == node_1.ku8MBSuccess) {
        for (int i = 1; i < 8; i++) {
            data[i] = node_1.getResponseBuffer(i);
            uint32_t combined = ((uint32_t)data[i] << 16) | data[1];
            float value;
            memcpy(&value, &combined, sizeof(value));
            switch (i) {
                case 1:
                    conductivity_1 = value;
                    Serial.print("Conductivity Value  : ");
                    Serial.println(value, 4);
                    break;
                case 3:
                    pH_1 = value;
                    Serial.print("pH Value           : ");
                    Serial.println(value, 4);
                    break;
                case 5:
                    temp_1_1 = value;
                    Serial.print("Temperature_1 Value: ");
                    Serial.println(value, 4);
                    break;
                case 7:
                    temp_2_1 = value;
                    Serial.print("Temperature_2 Value: ");
                    Serial.println(value, 4);
                    break;
            }
        }
    } else {
        Serial.print("Error reading Modbus: ");
        Serial.println(result, HEX);
    }
}
