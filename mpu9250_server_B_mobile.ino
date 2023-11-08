#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "MPU9250.h"

// Network credentials
const char* ssid = "HELLO1";
const char* password = "awse2313Z";
const char* esp32A_IP = "192.168.0.100"; // The IP address of ESP32 'A'

WebServer server(80);

// MPU9250 with specific settings
MPU9250 mpu;
int rssiESP32A = 0;  // This should be a global variable

// Function to calculate distance from RSSI
float calculateDistance(int rssi) {
  const float rssiAtOneMeter = -69; // RSSI at 1 meter distance, should be calibrated
  const float envFactor = 2.0; // Environmental factor, typically ranges from 2.0 to 4.0
  return pow(10.0, (rssiAtOneMeter - rssi) / (10.0 * envFactor));
}

// Function to fetch RSSI from ESP32 'A'
void fetchRSSIFromESP32A() {
  HTTPClient http;
  http.begin("http://" + String(esp32A_IP) + "/rssi"); // ESP32 'A' should have a /rssi endpoint
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    rssiESP32A = payload.toInt();
    Serial.println("RSSI from ESP32 'A': " + String(rssiESP32A));
  } else {
    Serial.println("Error on HTTP request: " + http.errorToString(httpCode));
    // Handle the error here
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize MPU9250 sensor with the specific settings
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x00;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
  setting.accel_fchoice = 0x00;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_21HZ;

  if (!mpu.setup(0x68, setting)) { // Change to your own MPU9250 address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  mpu.setMagneticDeclination(-0.55); // Set your local magnetic declination

  // Setup the web server
  server.on("/", HTTP_GET, []() {
    float distanceToESP32A = calculateDistance(rssiESP32A); // rssiESP32A should be a global variable
    String html = "<!DOCTYPE html><html><head><title>ESP32 Sensor Data</title>"
                  "<meta http-equiv='refresh' content='5'/></head><body>"
                  "<h1>ESP32 Sensor Data</h1>"
                  "<p><strong>RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>"
                  "<p><strong>Distance to Router:</strong> " + String(calculateDistance(WiFi.RSSI()), 2) + " m</p>"
                  "<p><strong>Distance to ESP32 A:</strong> " + String(distanceToESP32A, 2) + " m</p>"
                  "<p><strong>Roll:</strong> " + String(mpu.getRoll(), 3) + "</p>"
                  "<p><strong>Pitch:</strong> " + String(mpu.getPitch(), 3) + "</p>"
                  "<p><strong>Yaw:</strong> " + String(mpu.getYaw(), 3) + "</p>"
                  "<p><strong>AccX:</strong> " + String(mpu.getAccX(), 3) + " g</p>"
                  "<p><strong>AccY:</strong> " + String(mpu.getAccY(), 3) + " g</p>"
                  "<p><strong>AccZ:</strong> " + String(mpu.getAccZ(), 3) + " g</p>"
                  "<p><strong>MagX:</strong> " + String(mpu.getMagX(), 3) + " uT</p>"
                  "<p><strong>MagY:</strong> " + String(mpu.getMagY(), 3) + " uT</p>"
                  "<p><strong>MagZ:</strong> " + String(mpu.getMagZ(), 3) + " uT</p>"
                  "</body></html>";
    server.send(200, "text/html", html);
  });

  server.on("/rssi", HTTP_GET, []() {
    server.send(200, "text/plain", String(WiFi.RSSI()));
  });

  server.on("/data", HTTP_GET, []() {
  fetchRSSIFromESP32A(); // Fetch RSSI from ESP32 'A'
  if (mpu.update()) {
    float distanceToRouter = calculateDistance(WiFi.RSSI());
    float distanceToESP32A = calculateDistance(rssiESP32A); // Calculate the distance using the RSSI from ESP32 'A'
    String jsonData = "{\"distanceToRouter\":" + String(distanceToRouter, 2) +
                      ",\"distanceToESP32A\":" + String(distanceToESP32A, 2) +
                      ",\"roll\":" + String(mpu.getRoll(), 2) +
                      ",\"pitch\":" + String(mpu.getPitch(), 2) +
                      ",\"yaw\":" + String(mpu.getYaw(), 2) +
                      ",\"accX\":" + String(mpu.getAccX(), 2) +
                      ",\"accY\":" + String(mpu.getAccY(), 2) +
                      ",\"accZ\":" + String(mpu.getAccZ(), 2) +
                      ",\"magX\":" + String(mpu.getMagX(), 2) +
                      ",\"magY\":" + String(mpu.getMagY(), 2) +
                      ",\"magZ\":" + String(mpu.getMagZ(), 2) + "}";
    server.send(200, "application/json", jsonData);
  } else {
    server.send(500, "text/plain", "Failed to read sensor data");
  }
});


  server.begin();
  Serial.println("HTTP server started on IP: " + WiFi.localIP().toString());
}

void loop() {
 
  server.handleClient();
  // Keep fetching and updating the sensor data
  if (mpu.update()) {
    // Normally you would also handle the sensor data here
  }
  // Other code for loop...
   static unsigned long lastFetchTime = 0;
  if (millis() - lastFetchTime > 5000) { // Fetch every 5 seconds
    fetchRSSIFromESP32A();
    lastFetchTime = millis();
}}

// Function to print sensor data to Serial (call this in the loop)
void printSensorData() {
  Serial.print("Roll: ");
  Serial.print(mpu.getRoll(), 3);
  Serial.print(", Pitch: ");
  Serial.print(mpu.getPitch(), 3);
  Serial.print(", Yaw: ");
  Serial.println(mpu.getYaw(), 3);
  // ... other sensor values ...
}
