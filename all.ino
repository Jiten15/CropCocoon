#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#include <DHT.h>

// Replace these with your network credentials
const char* ssid = "sandeep";
const char* password = "sandeep1";

// Firebase settings
#define FIREBASE_HOST "agritest-620ac-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "uLrT0U5KdPpVwPUkQoU5lQoZ92xo7OxqDaEFlxYD"

// DHT11 settings
#define DHTPIN 15  // GPIO15 corresponds to D8 on NodeMCU
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

// Sensor pins
const int soilMoisturePin = A0;  // Connect Soil moisture analog sensor pin to A0 of NodeMCU
const int rainDropPin = 12;      // Connect Rain drop digital sensor pin to GPIO12 (D6 on NodeMCU)

// Motor control pins
const int motorIn1 = 14;  // Connect to IN1 on L298N
const int motorIn2 = 13;  // Connect to IN2 on L298N
const int motorENA = 4;   // Connect to ENA on L298N (optional for speed control)

// Debounce variables
int rainState = LOW;
int lastRainState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // 50 milliseconds debounce delay

// Anemometer pin
#define ANEMOMETERPIN 5 // GPIO5 corresponds to D1 on NodeMCU

volatile unsigned long pulseCount; // Number of pulses from the anemometer
unsigned long lastPulseTime;       // Time of the last pulse

void IRAM_ATTR countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi Connected!");

  // Firebase initialization
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  dht.begin();
  pinMode(rainDropPin, INPUT); // Set rain drop pin as input

  // Set up the anemometer pin
  pinMode(ANEMOMETERPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETERPIN), countPulse, FALLING);

  // Set up motor control pins
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorENA, OUTPUT);

  // Initialize variables
  pulseCount = 0;
  lastPulseTime = 0;
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float soilMoisture = 100.00 - ((analogRead(soilMoisturePin) / 1023.00) * 100.00);

  // Read the rain sensor input
  int reading = digitalRead(rainDropPin);

  // If the switch changed, due to noise or pressing:
  if (reading != lastRainState) {
    // Reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Whatever the reading is at, it's been there for longer than the debounce delay
    // so take it as the actual current state:
    if (reading != rainState) {
      rainState = reading;
    }
  }

  lastRainState = reading;

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Anemometer calculation
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastPulseTime;
  float windSpeed = 0;

  // Only update every second (or any desired interval)
  if (elapsedTime > 1000) {
    windSpeed = (pulseCount / (elapsedTime / 1000.0)) / 4.5; // Assuming 4.5 pulses per second per m/s
    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s");

    // Reset variables for the next calculation
    pulseCount = 0;
    lastPulseTime = currentTime;
  }

  // Print sensor values
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C");

  Serial.print("Soil Moisture (in Percentage) = ");
  Serial.print(soilMoisture);
  Serial.println("%");

  String rainStatus = rainState == HIGH ? "No Rain" : "Rain Detected";
  Serial.print("Rain Drop = ");
  Serial.println(rainStatus);

  // Send humidity value to Firebase
  if (Firebase.setFloat(firebaseData, "/humidity", h)) {
    Serial.println("Humidity value sent to Firebase.");
  } else {
    Serial.println("Failed to send humidity value to Firebase.");
    Serial.println(firebaseData.errorReason()); // Print the error reason
  }

  // Send temperature value to Firebase
  if (Firebase.setFloat(firebaseData, "/temperature", t)) {
    Serial.println("Temperature value sent to Firebase.");
  } else {
    Serial.println("Failed to send temperature value to Firebase.");
    Serial.println(firebaseData.errorReason()); // Print the error reason
  }

  // Send soil moisture value to Firebase
  if (Firebase.setFloat(firebaseData, "/soilMoisture", soilMoisture)) {
    Serial.println("Soil moisture value sent to Firebase.");
  } else {
    Serial.println("Failed to send soil moisture value to Firebase.");
    Serial.println(firebaseData.errorReason()); // Print the error reason
  }

  // Send rain drop value to Firebase
  if (Firebase.setString(firebaseData, "/rainDrop", rainStatus)) {
    Serial.println("Rain drop status sent to Firebase.");
  } else {
    Serial.println("Failed to send rain drop status to Firebase.");
    Serial.println(firebaseData.errorReason()); // Print the error reason
  }

  // Send wind speed value to Firebase
  if (Firebase.setFloat(firebaseData, "/windSpeed", windSpeed)) {
    Serial.println("Wind speed value sent to Firebase.");
  } else {
    Serial.println("Failed to send wind speed value to Firebase.");
    Serial.println(firebaseData.errorReason()); // Print the error reason
  }

  // Control the motor based on rain detection
  if (rainState == LOW) {  // Rain detected
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorENA, 255);  // Set motor speed (0-255)
    Serial.println("Motor activated: Covering the land.");
  } else {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorENA, 0);  // Stop the motor
    Serial.println("Motor deactivated: No rain detected.");
  }

  delay(5000); // Delay for 5 seconds before the next reading
}
