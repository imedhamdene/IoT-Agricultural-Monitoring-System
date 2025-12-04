// ESP32 Plant Monitoring System - Part 2

#include <WiFi.h>
#include <PubSubClient.h>   // For MQTT communication
#include <DHT.h>            // For DHT sensor (Temperature & Humidity)
#include <U8g2lib.h>        // For SSD1106 OLED (U8g2 supports SSD1106, I2C, 1.3")
#include <Wire.h>           // Required for I2C communication (for OLED)

// --- WiFi & MQTT Configuration ---
const char* ssid = "AndroidAP";        
const char* password = "oqro54177"; 
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883; // Default MQTT port

WiFiClient espClient;
PubSubClient client(espClient);

// --- MQTT Topics for Publishing Sensor Data ---
#define MQTT_TOPIC_TEMP       "plant_monitor/temperature"
#define MQTT_TOPIC_HUMIDITY   "plant_monitor/humidity"
#define MQTT_TOPIC_LIGHT      "plant_monitor/light"
#define MQTT_TOPIC_SOIL       "plant_monitor/soil_moisture"
#define MQTT_TOPIC_STATUS     "plant_monitor/status" // For general status messages

// --- MQTT Topics for Subscribing to Actuator Control Commands ---
// These allow you to remotely turn pump/lights ON/OFF via MQTT (from Node-RED, etc.)
#define MQTT_TOPIC_PUMP_SET   "plant_monitor/pump/set"
#define MQTT_TOPIC_LIGHTS_SET "plant_monitor/lights/set"


// --- Pin Definitions for ESP32 ---
// DHT Sensor
#define DHT_PIN         13
#define DHT_TYPE        DHT22 // Change to DHT11 if you're using DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// Light Sensor (LDR)
#define LDR_PIN         34 // Analog input, connect to voltage divider output

// PIR Motion Sensor
#define PIR_PIN         27 // Digital input

// Soil Humidity Sensor
#define SOIL_ANALOG_PIN 35 // Analog input
#define SOIL_DIGITAL_PIN 26 // Optional Digital input (D0 pin from module)

// Actuator Relays (Assuming active LOW relays, i.e., HIGH = OFF, LOW = ON)
// Adjust if your relays are active HIGH (HIGH = ON, LOW = OFF)
#define PUMP_RELAY_PIN  14 // Digital output for water pump relay
#define LIGHTS_RELAY_PIN 12 // Digital output for plant lights relay

// --- I2C OLED Screen (SSD1106 1.3") ---
// U8g2lib constructor for SSD1106 1.3", I2C, 128x64 resolution
// Using hardware I2C (SCL=GPIO22, SDA=GPIO21 by default on many ESP32s)
// Changed to SSD1306 constructor as suggested by compiler for this display type.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // No reset pin for this module type

// --- Sensor Variables ---
float temperature_C = 0.0;
float humidity_percent = 0.0;
int light_raw = 0; // Raw analog reading from LDR (0-4095 for ESP32 ADC)
int soil_moisture_analog = 0; // Raw analog reading from soil sensor (0-4095)
bool soil_moisture_digital = false; // D0 output from soil sensor module (true/false)

// --- Timers & State Variables ---
unsigned long lastSensorReadMillis = 0;
const long sensorReadInterval = 5000; // Read sensors and publish every 5 seconds

unsigned long lastMotionDetectedMillis = 0;
const long screenOffDelay = 10000; // Screen turns off 10 seconds after last motion
bool screenIsOn = false; // Current state of the OLED screen

bool pumpState = false;   // true = ON, false = OFF
bool lightsState = false; // true = ON, false = OFF

// --- Actuator Control Thresholds (Adjust these values based on your sensor readings) ---
const int LIGHT_THRESHOLD_LOW = 700; // Below this, lights turn on (e.g., 0-4095 scale)
const int SOIL_MOISTURE_THRESHOLD_LOW = 2500; // Below this, pump turns on (e.g., 0-4095 scale, higher value = dryer)


// --- Function Prototypes ---
void setup_wifi();
void reconnect_mqtt();
void callback_mqtt(char* topic, byte* payload, unsigned int length);
void readSensors();
void publishSensorData();
void updateOLEDDisplay(bool force_update = false);
void turnScreenOff();
void turnScreenOn();
void controlActuators();
void setPumpState(bool state);
void setLightsState(bool state);
void handleMotionSensor();


void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C bus

  Serial.println("\n--- ESP32 Plant Monitoring System ---");

  // --- Initialize DHT Sensor ---
  dht.begin();
  Serial.println("DHT Sensor initialized.");

  // --- Initialize OLED Screen ---
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr); // Choose a suitable font
  u8g2.setFontMode(1); // Transparent background
  Serial.println("OLED Screen initialized.");
  // Display initial message on OLED
  u8g2.clearBuffer();
  u8g2.setCursor(0, 10); u8g2.print("Plant Monitor");
  u8g2.setCursor(0, 30); u8g2.print("Connecting to WiFi...");
  u8g2.sendBuffer();
  screenIsOn = true; // Turn screen on for initial connection phase

  // --- Pin Modes ---
  pinMode(PIR_PIN, INPUT);
  pinMode(SOIL_DIGITAL_PIN, INPUT); // If using D0 output from soil sensor module
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(LIGHTS_RELAY_PIN, OUTPUT);

  // Ensure relays are OFF initially (assuming active LOW relays: HIGH = OFF)
  digitalWrite(PUMP_RELAY_PIN, HIGH);
  digitalWrite(LIGHTS_RELAY_PIN, HIGH);
  Serial.println("Actuator relays initialized to OFF.");

  // --- WiFi Setup ---
  setup_wifi();

  // --- MQTT Setup ---
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback_mqtt); // Set callback for incoming MQTT messages
  Serial.println("MQTT client configured.");

  // Give components time to settle
  delay(1000);
  Serial.println("Setup Complete. Starting main loop.");
}


void loop() {
  // --- MQTT Connection Management ---
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop(); // Keep MQTT connection alive and process incoming messages

  // --- Sensor Reading & Publishing ---
  if (millis() - lastSensorReadMillis >= sensorReadInterval) {
    readSensors();
    publishSensorData();
    controlActuators(); // Actuator logic based on sensor data
    updateOLEDDisplay(); // Update display with new data (if screen is on)
    lastSensorReadMillis = millis();
  }

  // --- Motion Sensor & Screen Control ---
  handleMotionSensor();

  // Small delay to prevent watchdog timer issues and save power
  delay(50);
}


// --- WiFi Functions ---
void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) { // Try for 15 seconds (30 * 500ms)
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10); u8g2.print("WiFi Connected!");
    u8g2.setCursor(0, 30); u8g2.print(WiFi.localIP());
    u8g2.sendBuffer();
    delay(2000);
  } else {
    Serial.println("\nFailed to connect to WiFi. Restarting...");
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10); u8g2.print("WiFi Failed!");
    u8g2.setCursor(0, 30); u8g2.print("Restarting...");
    u8g2.sendBuffer();
    delay(3000);
    ESP.restart(); // Restart ESP32 if WiFi connection fails
  }
}


// --- MQTT Functions ---
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Generate a unique client ID using part of MAC address or random number
    String clientId = "ESP32_PlantMonitor_" + String(WiFi.macAddress()).substring(9);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected to MQTT broker.");
      client.publish(MQTT_TOPIC_STATUS, "ESP32 Sensor node online.");
      // Subscribe to actuator control topics
      client.subscribe(MQTT_TOPIC_PUMP_SET);
      client.subscribe(MQTT_TOPIC_LIGHTS_SET);
      Serial.println("Subscribed to actuator control topics.");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

// MQTT Callback function for handling incoming messages (e.g., remote control from Node-RED)
void callback_mqtt(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  if (String(topic) == MQTT_TOPIC_PUMP_SET) {
    if (message == "ON") {
      setPumpState(true);
      Serial.println("Pump manually turned ON.");
      client.publish(MQTT_TOPIC_STATUS, "Pump ON (manual)");
    } else if (message == "OFF") {
      setPumpState(false);
      Serial.println("Pump manually turned OFF.");
      client.publish(MQTT_TOPIC_STATUS, "Pump OFF (manual)");
    }
  } else if (String(topic) == MQTT_TOPIC_LIGHTS_SET) {
    if (message == "ON") {
      setLightsState(true);
      Serial.println("Lights manually turned ON.");
      client.publish(MQTT_TOPIC_STATUS, "Lights ON (manual)");
    } else if (message == "OFF") {
      setLightsState(false);
      Serial.println("Lights manually turned OFF.");
      client.publish(MQTT_TOPIC_STATUS, "Lights OFF (manual)");
    }
  }
}


// --- Sensor Reading Functions ---
void readSensors() {
  // Read DHT Sensor
  temperature_C = dht.readTemperature();
  humidity_percent = dht.readHumidity();
  if (isnan(temperature_C) || isnan(humidity_percent)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature_C = 0.0; // Reset to default on failure
    humidity_percent = 0.0;
  }

  // Read Light Sensor (LDR)
  light_raw = analogRead(LDR_PIN); // Returns 0-4095 for ESP32 ADC

  // Read Soil Humidity Sensor
  soil_moisture_analog = analogRead(SOIL_ANALOG_PIN); // Returns 0-4095
  soil_moisture_digital = digitalRead(SOIL_DIGITAL_PIN); // Reads HIGH/LOW

  Serial.printf("Temp: %.1fC, Hum: %.1f%%, Light: %d, Soil A: %d, Soil D: %s\n",
                temperature_C, humidity_percent, light_raw, soil_moisture_analog,
                soil_moisture_digital ? "DRY" : "WET"); // Assuming D0 is HIGH when DRY, LOW when WET
}


// --- Data Publishing Function ---
void publishSensorData() {
  if (client.connected()) {
    char buffer[10]; // Buffer to hold converted sensor values

    // Publish Temperature
    dtostrf(temperature_C, 1, 1, buffer); // (value, min_width, num_decimal_places, char_array)
    client.publish(MQTT_TOPIC_TEMP, buffer);

    // Publish Humidity
    dtostrf(humidity_percent, 1, 1, buffer);
    client.publish(MQTT_TOPIC_HUMIDITY, buffer);

    // Publish Light (raw analog value)
    itoa(light_raw, buffer, 10); // (value, char_array, base)
    client.publish(MQTT_TOPIC_LIGHT, buffer);

    // Publish Soil Moisture (raw analog value)
    itoa(soil_moisture_analog, buffer, 10);
    client.publish(MQTT_TOPIC_SOIL, buffer);

    Serial.println("Sensor data published to MQTT.");
  }
}


// --- OLED Display Functions ---
void updateOLEDDisplay(bool force_update) {
  if (!screenIsOn && !force_update) return; // Only update if screen is on or forced

  u8g2.clearBuffer();
  u8g2.setCursor(0, 10); u8g2.print("Temp: "); u8g2.print(temperature_C, 1); u8g2.print("C");
  u8g2.setCursor(0, 25); u8g2.print("Hum: "); u8g2.print(humidity_percent, 1); u8g2.print("%");
  u8g2.setCursor(0, 40); u8g2.print("Light: "); u8g2.print(light_raw);
  u8g2.setCursor(0, 55); u8g2.print("Soil: "); u8g2.print(soil_moisture_analog);

  u8g2.sendBuffer();
}

void turnScreenOff() {
  if (screenIsOn) {
    u8g2.clearDisplay(); // Clear display contents and turn off pixels
    screenIsOn = false;
    Serial.println("OLED Screen OFF.");
  }
}

void turnScreenOn() {
  if (!screenIsOn) {
    u8g2.clearDisplay(); // Clear any previous off state
    u8g2.setPowerSave(0); // Turn on display hardware
    screenIsOn = true;
    updateOLEDDisplay(true); // Force update when turning on
    Serial.println("OLED Screen ON.");
  }
}


// --- Actuator Control Functions ---
void controlActuators() {
  // Control Lights based on Light Sensor
  // If current light is below threshold AND lights are currently off
  if (light_raw < LIGHT_THRESHOLD_LOW) {
    if (!lightsState) {
      setLightsState(true);
      Serial.println("Light level low. Turning lights ON (auto).");
      client.publish(MQTT_TOPIC_STATUS, "Lights ON (auto)");
    }
  } else { // If current light is above threshold AND lights are currently on
    if (lightsState) {
      setLightsState(false);
      Serial.println("Light level sufficient. Turning lights OFF (auto).");
      client.publish(MQTT_TOPIC_STATUS, "Lights OFF (auto)");
    }
  }

  // Control Water Pump based on Soil Moisture Sensor
  // Note: Soil moisture sensor readings are typically inverted (higher analog value means dryer soil)
  // If soil moisture is below threshold (i.e., dry) AND pump is currently off
  if (soil_moisture_analog > SOIL_MOISTURE_THRESHOLD_LOW) {
    if (!pumpState) {
      setPumpState(true);
      Serial.println("Soil dry. Turning pump ON (auto).");
      client.publish(MQTT_TOPIC_STATUS, "Pump ON (auto)");
      // For a real system, you might want to run the pump for a fixed duration
      // and then turn it off, rather than continuously until moisture improves.
      // This simple logic will run it until moisture improves.
    }
  } else { // If soil moisture is above threshold (i.e., wet enough) AND pump is currently on
    if (pumpState) {
      setPumpState(false);
      Serial.println("Soil moist enough. Turning pump OFF (auto).");
      client.publish(MQTT_TOPIC_STATUS, "Pump OFF (auto)");
    }
  }
}

void setPumpState(bool state) {
  pumpState = state;
  // Assuming active LOW relay (LOW = ON, HIGH = OFF)
  digitalWrite(PUMP_RELAY_PIN, state ? LOW : HIGH);
}

void setLightsState(bool state) {
  lightsState = state;
  // Assuming active LOW relay (LOW = ON, HIGH = OFF)
  digitalWrite(LIGHTS_RELAY_PIN, state ? LOW : HIGH);
}


// --- Motion Sensor Handler ---
void handleMotionSensor() {
  if (digitalRead(PIR_PIN) == HIGH) { // Motion detected
    lastMotionDetectedMillis = millis();
    turnScreenOn();
  }

  // If screen is on and no motion for 'screenOffDelay' time
  if (screenIsOn && (millis() - lastMotionDetectedMillis > screenOffDelay)) {
    turnScreenOff();
  }
}