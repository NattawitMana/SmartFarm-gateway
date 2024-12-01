#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>

// Define DHT11 sensor
#define DHTPIN 22       // Pin connected to DHT11
#define DHTTYPE DHT11   // DHT 11 sensor
DHT dht(DHTPIN, DHTTYPE);

// Define MQ135 sensor pin (analog input)
#define MQ135_PIN 35    // Pin connected to MQ135 sensor

// Define relay and LED pins
#define RELAY_PIN 13
#define LED_PIN 4

// Time management for relay
unsigned long lastRelayActivationTime = 0; // Track last activation time
const unsigned long relayActiveDuration = 20000; // 20 seconds in milliseconds

// Structure to receive data from Device 1 and add relay state
typedef struct struct_message {
  int soilMoisture;      // Soil moisture data (from Device 1)
  float lightLevel;      // Light level sensor data (from Device 1)
  float temperature;     // DHT11 temperature
  float humidity;        // DHT11 humidity
  float co2Concentration; // MQ135 CO2 concentration
  bool relayState;       // Relay state
} struct_message;

struct_message outgoingData; // Data to send to Device 3

// ESP-NOW callback function to receive data from Device 1
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  struct_message incomingReadings;
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  // Store received data
  outgoingData.soilMoisture = incomingReadings.soilMoisture;
  outgoingData.lightLevel = incomingReadings.lightLevel;

  // Relay control logic
  if (outgoingData.lightLevel < 100) {
    outgoingData.relayState = true; // Activate relay
    lastRelayActivationTime = millis(); // Record the activation time
  } else if (millis() - lastRelayActivationTime > relayActiveDuration) {
    outgoingData.relayState = false; // Deactivate relay after 20 seconds
  }

  // Update relay state
  digitalWrite(RELAY_PIN, outgoingData.relayState ? HIGH : LOW);

  Serial.println("Received Data from Device 1:");
  Serial.print("  Soil Moisture: ");
  Serial.println(outgoingData.soilMoisture);
  Serial.print("  Light Level: ");
  Serial.println(outgoingData.lightLevel);
  Serial.print("  Relay State: ");
  Serial.println(outgoingData.relayState ? "Active" : "Inactive");
  Serial.println("---------------------------------------");
}

// Read DHT11 sensor data
void readDHT11() {
  outgoingData.temperature = dht.readTemperature();
  outgoingData.humidity = dht.readHumidity();

  // Check if readings are valid
  if (!isnan(outgoingData.temperature) && !isnan(outgoingData.humidity)) {
    Serial.println("DHT11 Sensor Data:");
    Serial.print("  Temperature: ");
    Serial.print(outgoingData.temperature);
    Serial.println(" °C");
    Serial.print("  Humidity: ");
    Serial.print(outgoingData.humidity);
    Serial.println(" %");
    Serial.println("---------------------------------------");
  } else {
    Serial.println("Failed to read from DHT sensor!");
    Serial.println("---------------------------------------");
  }
}

// Read MQ135 sensor data
void readMQ135() {
  int analogValue = analogRead(MQ135_PIN);  // Read the analog value

  // Debug raw analog value
  Serial.print("Raw MQ135 Analog Value: ");
  Serial.println(analogValue);

  // Example improved scaling (adjust values based on your calibration)
  if (analogValue > 200 && analogValue < 3000) {
    outgoingData.co2Concentration = map(analogValue, 200, 3000, 400, 1000); // Adjust range based on observations
  } else {
    outgoingData.co2Concentration = -1; // Invalid or out-of-range reading
  }

  Serial.print("MQ135 CO2 Concentration: ");
  if (outgoingData.co2Concentration != -1) {
    Serial.print(outgoingData.co2Concentration);
    Serial.println(" ppm");
  } else {
    Serial.println("Invalid reading");
  }
  Serial.println("---------------------------------------");
}

// Control LED based on relay state
void openLED() {
  if (outgoingData.relayState) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize DHT sensor
  dht.begin();

  // Initialize Relay and LED pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Ensure relay is off initially

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the ESP-NOW callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize Serial2 for communication with Device 3
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("Device 2 setup complete.");
}

void loop() {
  // Read sensors
  readDHT11();
  readMQ135();

  // Send data to Device 3 via Serial2
  Serial2.write((uint8_t *)&outgoingData, sizeof(outgoingData));

  // Debug: Print all data being sent
  Serial.println("Sending Data to Device 3:");
  Serial.print("  Soil Moisture: ");
  Serial.println(outgoingData.soilMoisture);
  Serial.print("  Light Level: ");
  Serial.println(outgoingData.lightLevel);
  Serial.print("  Temperature: ");
  Serial.println(outgoingData.temperature);
  Serial.print("  Humidity: ");
  Serial.println(outgoingData.humidity);
  Serial.print("  CO2: ");
  Serial.println(outgoingData.co2Concentration);
  Serial.print("  Relay State: ");
  Serial.println(outgoingData.relayState ? "Active" : "Inactive");

  openLED();

  Serial.println("---------------------------------------");

  delay(2000); // Reduce processing load
}
