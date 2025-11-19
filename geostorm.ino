#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi credentials
const char* ssid = "SomeWifiNetwork";
const char* password = "JellyBeansAreAwesome";

// Server endpoint
const char* serverUrl = "https://app.monitman.com/dashboard/receive.php?sensor=magnetometer";

// RM3100 I2C Address
#define RM3100_ADDR 0x20 //Pull down SA0/SA1 pin to GND for this address

// RM3100 Registers
#define RM3100_POLL 0x00
#define RM3100_CMM 0x01
#define RM3100_CCX 0x04
#define RM3100_CCY 0x06
#define RM3100_CCZ 0x08
#define RM3100_TMRC 0x0B
#define RM3100_MX 0x24
#define RM3100_MY 0x27
#define RM3100_MZ 0x2A
#define RM3100_BIST 0x33
#define RM3100_STATUS 0x34
#define RM3100_HSHAKE 0x35
#define RM3100_REVID 0x36

// I2C and Control Pins for ESP32-C6
#define SDA_PIN 6
#define SCL_PIN 7
#define I2CEN_PIN 8    // Pull HIGH to enable I2C mode
#define DRDY_PIN 10    // Data Ready interrupt pin

// Baseline calibration settings
#define CALIBRATION_SAMPLES 100
#define STORM_THRESHOLD_MULTIPLIER 1.5  // 50% above baseline
#define READINGS_PER_SECOND 10
#define UPLOAD_INTERVAL_MS 60000  // Upload every 60 seconds

// Baseline values (will be calibrated at startup)
float baselineX = 0, baselineY = 0, baselineZ = 0;
float baselineMagnitude = 0;
float stdDevX = 0, stdDevY = 0, stdDevZ = 0;

// Tracking variables for uploads
unsigned long lastUploadTime = 0;
float avgX = 0, avgY = 0, avgZ = 0, avgMag = 0;
int sampleCount = 0;
String currentDetectionLevel = "Normal";

// Data ready flag (set by interrupt)
volatile bool dataReady = false;

// Interrupt Service Routine for DRDY pin
void IRAM_ATTR drdyISR() {
  dataReady = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n=== RM3100 Geomagnetic Storm Monitor ===");
  
  // Configure I2CEN pin - MUST be HIGH for I2C mode
  pinMode(I2CEN_PIN, OUTPUT);
  digitalWrite(I2CEN_PIN, HIGH);
  Serial.println("I2C mode enabled (I2CEN = HIGH)");
  
  // Configure DRDY pin as input with interrupt
  pinMode(DRDY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), drdyISR, RISING);
  Serial.println("DRDY interrupt configured");
  
  delay(100); // Give pins time to settle
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println("IP address: " + WiFi.localIP().toString());
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C speed
  
  delay(100);
  
  // Check if RM3100 is connected
  if (!detectRM3100()) {
    Serial.println("ERROR: RM3100 not detected!");
    Serial.println("Check wiring:");
    Serial.println("  - I2CEN pin should be HIGH");
    Serial.println("  - SDA/SCL connections");
    Serial.println("  - Power supply (3.3V)");
    Serial.println("  - I2C address (SA0/SA1 pins)");
    while (1) delay(1000);
  }
  
  // Initialize RM3100
  if (!initRM3100()) {
    Serial.println("ERROR: Failed to initialize RM3100!");
    while (1) delay(1000);
  }
  
  Serial.println("RM3100 initialized successfully");
  
  // Verify DRDY functionality
  Serial.println("Testing DRDY pin...");
  if (testDRDY()) {
    Serial.println("DRDY pin working correctly!");
  } else {
    Serial.println("WARNING: DRDY pin may not be connected properly");
    Serial.println("System will continue but may be less efficient");
  }
  
  // Calibrate baseline
  Serial.println("\nCalibrating baseline (please ensure no magnetic interference)...");
  calibrateBaseline();
  
  Serial.println("\n=== Monitoring Started ===");
  Serial.println("Baseline Magnitude: " + String(baselineMagnitude, 2) + " uT");
  Serial.println("Storm threshold: " + String(baselineMagnitude * STORM_THRESHOLD_MULTIPLIER, 2) + " uT");
  Serial.println("Uploading to server every 60 seconds\n");
  
  lastUploadTime = millis();
  dataReady = false; // Clear any pending interrupt
}

void loop() {
  float x, y, z;
  
  // Wait for data ready interrupt (or timeout after 200ms)
  if (waitForDataReady(200)) {
    if (readMagneticField(&x, &y, &z)) {
      float magnitude = sqrt(x*x + y*y + z*z);
      
      // Accumulate values for averaging
      avgX += x;
      avgY += y;
      avgZ += z;
      avgMag += magnitude;
      sampleCount++;
      
      // Calculate deviation from baseline
      float deviationX = abs(x - baselineX);
      float deviationY = abs(y - baselineY);
      float deviationZ = abs(z - baselineZ);
      float magnitudeChange = abs(magnitude - baselineMagnitude);
      float percentChange = (magnitudeChange / baselineMagnitude) * 100.0;
      
      // Print readings
      Serial.print("X: " + String(x, 2) + " uT  ");
      Serial.print("Y: " + String(y, 2) + " uT  ");
      Serial.print("Z: " + String(z, 2) + " uT  ");
      Serial.print("Mag: " + String(magnitude, 2) + " uT  ");
      Serial.print("Change: " + String(percentChange, 1) + "%  ");
      
      // Check for unusual activity and update detection level
      bool stormDetected = false;
      
      if (magnitude > baselineMagnitude * STORM_THRESHOLD_MULTIPLIER) {
        Serial.print("[!!! POSSIBLE GEOMAGNETIC STORM !!!]");
        currentDetectionLevel = "Storm";
        stormDetected = true;
      } else if (deviationX > max(stdDevX * 5, 1.0) || 
        deviationY > max(stdDevY * 5, 1.0) || 
        deviationZ > max(stdDevZ * 5, 1.0)) {
        Serial.print("[! High Deviation Detected !]");
        currentDetectionLevel = "High Deviation";
        stormDetected = true;
      } else if (percentChange > 20) {
        Serial.print("[! Significant Change !]");
        currentDetectionLevel = "Significant Change";
      } else {
        Serial.print("[Normal]");
        currentDetectionLevel = "Normal";
      }
      
      Serial.println();
      
      // If storm detected, increase sampling rate briefly
      if (stormDetected) {
        Serial.println("  Taking rapid samples...");
        for (int i = 0; i < 5; i++) {
          delay(100);
          if (waitForDataReady(200) && readMagneticField(&x, &y, &z)) {
            magnitude = sqrt(x*x + y*y + z*z);
            Serial.println("  Rapid sample " + String(i+1) + ": " + String(magnitude, 2) + " uT");
          }
        }
      }
    } else {
      Serial.println("ERROR: Failed to read magnetic field");
    }
  } else {
    Serial.println("WARNING: Data ready timeout");
  }
  
  // Check if it's time to upload
  if (millis() - lastUploadTime >= UPLOAD_INTERVAL_MS && sampleCount > 0) {
    uploadReadings();
    lastUploadTime = millis();
    
    // Reset averages
    avgX = 0;
    avgY = 0;
    avgZ = 0;
    avgMag = 0;
    sampleCount = 0;
  }
  
  delay(1000 / READINGS_PER_SECOND);
}

bool detectRM3100() {
  Wire.beginTransmission(RM3100_ADDR);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    // Read REVID register
    uint8_t revid = readRegister(RM3100_REVID);
    Serial.println("RM3100 detected!");
    Serial.println("  REVID: 0x" + String(revid, HEX));
    
    // Read HSHAKE register for diagnostics
    uint8_t hshake = readRegister(RM3100_HSHAKE);
    Serial.println("  HSHAKE: 0x" + String(hshake, HEX));
    
    return true; // Don't enforce specific REVID in case of different versions
  } else {
    Serial.println("I2C Error code: " + String(error));
    if (error == 2) {
      Serial.println("  - Device not found at address 0x" + String(RM3100_ADDR, HEX));
      Serial.println("  - Check I2C address (SA0/SA1 pins)");
    } else if (error == 5) {
      Serial.println("  - Timeout - check connections");
    }
  }
  return false;
}

bool initRM3100() {
  // Configure HSHAKE register for optimal DRDY behavior
  // DRC0=1 (DRDY cleared by register write)
  // DRC1=1 (DRDY cleared by reading measurement results)
  writeRegister(RM3100_HSHAKE, 0x1B); // Default value with both DRC bits set
  
  // Set Cycle Count for all axes (higher = more sensitive, slower)
  // 200 (0x00C8) is a good balance
  writeRegister(RM3100_CCX, 0x00);
  writeRegister(RM3100_CCX + 1, 0xC8);
  writeRegister(RM3100_CCY, 0x00);
  writeRegister(RM3100_CCY + 1, 0xC8);
  writeRegister(RM3100_CCZ, 0x00);
  writeRegister(RM3100_CCZ + 1, 0xC8);
  
  delay(10);
  
  // Verify cycle counts were set
  uint8_t ccxMSB = readRegister(RM3100_CCX);
  uint8_t ccxLSB = readRegister(RM3100_CCX + 1);
  uint16_t cycleCount = (ccxMSB << 8) | ccxLSB;
  Serial.println("  Cycle count set to: " + String(cycleCount));
  
  // Set Continuous Measurement Mode (CMM) for all axes
  // Bit 7 = 0 (reserved)
  // Bit 6 = CMZ (1 = measure Z axis)
  // Bit 5 = CMY (1 = measure Y axis)
  // Bit 4 = CMX (1 = measure X axis)
  // Bit 3 = 0 (reserved)
  // Bit 2-1 = DRDM (00 = DRDY after all axes measured)
  // Bit 0 = START (1 = start continuous measurement)
  writeRegister(RM3100_CMM, 0x71); // 0b01110001 = all axes, DRDY after all, start
  
  // Set TMRC (sample rate) - 0x96 = ~37 Hz, good for 10 Hz reading rate
  writeRegister(RM3100_TMRC, 0x96);
  
  delay(100);
  
  // Verify CMM was set
  uint8_t cmm = readRegister(RM3100_CMM);
  Serial.println("  CMM register: 0x" + String(cmm, HEX));
  
  return true;
}

bool testDRDY() {
  // Clear any existing interrupt
  dataReady = false;
  
  // Wait up to 1 second for DRDY
  unsigned long startTime = millis();
  while (!dataReady && (millis() - startTime < 1000)) {
    delay(10);
  }
  
  // Also check the pin directly
  bool pinHigh = digitalRead(DRDY_PIN) == HIGH;
  
  if (dataReady || pinHigh) {
    // Clear the data by reading
    uint8_t status = readRegister(RM3100_STATUS);
    dataReady = false;
    return true;
  }
  
  return false;
}

bool waitForDataReady(unsigned long timeoutMs) {
  unsigned long startTime = millis();
  
  // Wait for interrupt flag or timeout
  while (!dataReady && (millis() - startTime < timeoutMs)) {
    delay(1);
    
    // Also check STATUS register as backup
    if (millis() - startTime > timeoutMs / 2) {
      uint8_t status = readRegister(RM3100_STATUS);
      if (status & 0x80) { // Bit 7 = DRDY
        dataReady = true;
        break;
      }
    }
  }
  
  if (dataReady) {
    dataReady = false; // Clear the flag
    return true;
  }
  
  return false;
}

void calibrateBaseline() {
  float sumX = 0, sumY = 0, sumZ = 0;
  float sumSqX = 0, sumSqY = 0, sumSqZ = 0;
  int validSamples = 0;
  
  Serial.print("Progress: ");
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float x, y, z;
    
    if (waitForDataReady(200) && readMagneticField(&x, &y, &z)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      sumSqX += x * x;
      sumSqY += y * y;
      sumSqZ += z * z;
      validSamples++;
      
      if (i % 10 == 0) Serial.print(".");
    }
    delay(50);
  }
  Serial.println(" Done!");
  
  if (validSamples > 0) {
    baselineX = sumX / validSamples;
    baselineY = sumY / validSamples;
    baselineZ = sumZ / validSamples;
    baselineMagnitude = sqrt(baselineX*baselineX + baselineY*baselineY + baselineZ*baselineZ);
    
    // Calculate standard deviation
    stdDevX = sqrt((sumSqX / validSamples) - (baselineX * baselineX));
    stdDevY = sqrt((sumSqY / validSamples) - (baselineY * baselineY));
    stdDevZ = sqrt((sumSqZ / validSamples) - (baselineZ * baselineZ));
    
    Serial.println("Baseline calibrated from " + String(validSamples) + " samples:");
    Serial.println("  X: " + String(baselineX, 2) + " ± " + String(stdDevX, 2) + " uT");
    Serial.println("  Y: " + String(baselineY, 2) + " ± " + String(stdDevY, 2) + " uT");
    Serial.println("  Z: " + String(baselineZ, 2) + " ± " + String(stdDevZ, 2) + " uT");
  } else {
    Serial.println("ERROR: No valid samples during calibration!");
  }
}

bool readMagneticField(float *x, float *y, float *z) {
  // Read 24-bit values for X, Y, Z
  int32_t rawX = read24bitRegister(RM3100_MX);
  int32_t rawY = read24bitRegister(RM3100_MY);
  int32_t rawZ = read24bitRegister(RM3100_MZ);
  
  // Reading the measurement registers automatically clears DRDY
  
  // Convert to microTesla (cycle count = 200)
  // From datasheet Table 3-1: gain = 75 LSB/uT for cycle count 200
  float gain = 75.0;
  *x = (float)rawX / gain;
  *y = (float)rawY / gain;
  *z = (float)rawZ / gain;
  
  return true;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  delay(1); // Small delay for register write to complete
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Send restart, not stop
  Wire.requestFrom(RM3100_ADDR, 1);
  
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

int32_t read24bitRegister(uint8_t reg) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Send restart, not stop
  Wire.requestFrom(RM3100_ADDR, 3);
  
  int32_t value = 0;
  if (Wire.available() >= 3) {
    value |= (int32_t)Wire.read() << 16;  // MSB
    value |= (int32_t)Wire.read() << 8;   // Middle byte
    value |= (int32_t)Wire.read();         // LSB
  }
  
  // Sign extend from 24-bit to 32-bit
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  
  return value;
}

void uploadReadings() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping upload");
    return;
  }
  
  // Calculate averages
  float readingX = avgX / sampleCount;
  float readingY = avgY / sampleCount;
  float readingZ = avgZ / sampleCount;
  float readingMag = avgMag / sampleCount;
  
  // Get current timestamp (Unix epoch)
  unsigned long timestamp = millis() / 1000;
  
  // Build JSON manually (no library needed)
  String json = "{";
  json += "\"timestamp\":" + String(timestamp) + ",";
  json += "\"detection_level\":\"" + currentDetectionLevel + "\",";
  json += "\"readings\":{";
  json += "\"x\":" + String(readingX, 2) + ",";
  json += "\"y\":" + String(readingY, 2) + ",";
  json += "\"z\":" + String(readingZ, 2) + ",";
  json += "\"magnitude\":" + String(readingMag, 2);
  json += "},";
  json += "\"baseline\":{";
  json += "\"x\":" + String(baselineX, 2) + ",";
  json += "\"y\":" + String(baselineY, 2) + ",";
  json += "\"z\":" + String(baselineZ, 2) + ",";
  json += "\"magnitude\":" + String(baselineMagnitude, 2);
  json += "},";
  json += "\"deviation\":{";
  json += "\"x\":" + String(abs(readingX - baselineX), 2) + ",";
  json += "\"y\":" + String(abs(readingY - baselineY), 2) + ",";
  json += "\"z\":" + String(abs(readingZ - baselineZ), 2) + ",";
  json += "\"percent_change\":" + String((abs(readingMag - baselineMagnitude) / baselineMagnitude) * 100.0, 2);
  json += "},";
  json += "\"sample_count\":" + String(sampleCount);
  json += "}";
  
  Serial.println("\n--- Uploading to Server ---");
  Serial.println(json);
  
  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(json);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Server response: " + response);
  } else {
    Serial.println("Error sending POST request: " + String(httpResponseCode));
  }
  
  http.end();
  Serial.println("--- Upload Complete ---\n");
}
