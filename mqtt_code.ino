#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "gnet306553";
const char* password = "houda12345";

// MQTT broker details
const char* mqtt_server = "test.mosquitto.org";
const char* humidity_topic = "sensor/humidity";
const char* temperature_topic = "sensor/temperature";

WiFiClient espClient;
PubSubClient client(espClient);

#define RX2 16  // ESP32 GPIO pin for RX2
#define TX2 17  // ESP32 GPIO pin for TX2

void setup() {
    Serial.begin(9600);  // UART0 for debugging with PC
    Serial2.begin(9600, SERIAL_8N1, RX2, TX2);  // UART2 for communication with STM32

    if (Serial2) {
        Serial.println("Serial2 initialized successfully");
    } else {
        Serial.println("Failed to initialize Serial2");
    }

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Connect to MQTT broker
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);

    String clientId = "ESP32Client-" + String(WiFi.macAddress());
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.println(client.state());
            delay(2000);
        }
    }

    Serial.println("ESP32 is ready");
}

void loop() {
    if (Serial2.available()) {
        String received = Serial2.readStringUntil('\n');  // Read the incoming string until newline
        Serial.println("Received from STM32: " + received);

        // Parse temperature and humidity from the received string
        float temperature = parseValue(received, "Temperature:");
        float humidity = parseValue(received, "Humidity:");

        // Ensure valid temperature and humidity before publishing
        if (!isnan(temperature)) {
            char tempBuffer[8];
            dtostrf(temperature, 4, 2, tempBuffer);
            client.publish(temperature_topic, tempBuffer);
            Serial.println("Sent to MQTT (temperature): " + String(tempBuffer));
        } else {
            Serial.println("Error: Temperature is NaN, skipping MQTT publish");
        }

        if (!isnan(humidity)) {
            char humBuffer[8];
            dtostrf(humidity, 4, 2, humBuffer);
            client.publish(humidity_topic, humBuffer);
            Serial.println("Sent to MQTT (humidity): " + String(humBuffer));
        } else {
            Serial.println("Error: Humidity is NaN, skipping MQTT publish");
        }
    }

    // Handle MQTT communication
    client.loop();
}

// Function to parse a float value from the received string
float parseValue(String data, const char* key) {
    int keyIndex = data.indexOf(key);
    if (keyIndex != -1) {
        int start = keyIndex + strlen(key);
        int end = data.indexOf(",", start);  // Find comma as the end of the value
        if (end == -1) end = data.length();  // No comma, use the end of the string
        String value = data.substring(start, end);

        value.trim();  // Remove leading/trailing spaces
        value.replace("C", "");
        value.replace("%", "");

        // Debugging parsed value
        Serial.println("Parsed value for key: " + String(key) + " -> " + value);

        if (value.length() == 0) {
            Serial.println("Error: Parsed value is empty");
            return NAN;
        }

        // Check if the value contains invalid characters
        if (value.toFloat() == 0 && value != "0" && value != "0.0") {
            Serial.println("Error: Invalid numeric value -> " + value);
            return NAN;
        }

        return value.toFloat();  // Convert cleaned string to float
    }
    Serial.println("Error: Key not found in data -> " + String(key));
    return NAN;  // Return NaN if key is not found
}

// Callback function for receiving MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println("Received MQTT topic: " + String(topic) + " Message: " + message);

    // Check for valid float
    float value = message.toFloat();
    if (value == 0 && message != "0" && message != "0.0") {
        Serial.println("Error: Invalid MQTT payload received: " + message);
    } else {
        Serial.println("Valid MQTT payload: " + String(value));
    }
}
