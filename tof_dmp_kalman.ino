#include <WiFi.h>
#include <WebServer.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <WebSocketsServer.h>

// Wi-Fi Configuration
const char* ssid = "HELLO1";
const char* password = "awse2313Z";
IPAddress local_IP(192, 168, 0, 20);
IPAddress gateway(192, 168, 0, 2);
IPAddress subnet(255, 255, 255, 0);

// TOF sensor configuration
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define SHT_LOX1 D7
#define SHT_LOX2 D8
#define SHT_LOX3 D6
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

// MPU6050 configuration and Kalman filter variables
MPU6050 accelgyro;
int led = 11;
unsigned long now, lastTime = 0;
float dt;
int16_t ax, ay, az, gx, gy, gz;
long axo = 0, ayo = 0, azo = 0;
long gxo = 0, gyo = 0, gzo = 0;
float aax, aay, aaz, agx, agy, agz;
float AcceRatio = 16384.0;
float GyroRatio = 131.0;
float pi = 3.1415926;
uint8_t n_sample = 8;
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};
long aax_sum, aay_sum, aaz_sum;
float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0};
float Px=1, Rx, Kx, Sx, Vx, Qx;
float Py=1, Ry, Ky, Sy, Vy, Qy;
float Pz=1, Rz, Kz, Sz, Vz, Qz;

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);  // create a websocket server on port 81

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.config(local_IP, gateway, subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println();
    Serial.print("Connected to WiFi, IP address: ");
    Serial.println(WiFi.localIP());
}

void handleRoot() {
   server.send(200, "text/plain", "ESP32 Server Running...");
}

void handleData() {
    String data = "{";
    data += "\"sensor1\":" + String(measure1.RangeMilliMeter) + ",";
    data += "\"sensor2\":" + String(measure2.RangeMilliMeter) + ",";
    data += "\"sensor3\":" + String(measure3.RangeMilliMeter) + ",";
    data += "\"ax\":" + String(aax) + ",";
    data += "\"ay\":" + String(aay) + ",";
    data += "\"az\":" + String(aaz) + ",";
    data += "\"gx\":" + String(agx) + ",";
    data += "\"gy\":" + String(agy) + ",";
    data += "\"gz\":" + String(agz) + "}";
    server.send(200, "application/json", data);
}

void setupServer() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.begin();
}

void setID() {
  // All reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);

  // All unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // Activating LOX1 and resetting LOX2 & LOX3
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // Initializing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // Activating LOX2 and resetting LOX3
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);

  // Initializing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  delay(10);

  // Activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // Initializing LOX3
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            }
            break;
        case WStype_TEXT:
            // You can add logic here to handle any text-based commands from the client.
            break;
        case WStype_BIN:
            Serial.printf("[%u] get binary length: %u\n", num, length);
            break;
    }
}
void setup() {
    Serial.begin(115200);
    setupWiFi();
    setupServer();
    webSocket.begin();  // start the websocket server
    webSocket.onEvent(webSocketEvent);  // if there's an incoming websocket message, go to function 'webSocketEvent'
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    Serial.println(F("Shutdown pins inited..."));
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    Serial.println(F("All in reset mode...(pins are low)"));
    Serial.println(F("Starting..."));
    setID();

    // MPU6050 initialization
    Wire.begin();
    accelgyro.initialize();
    unsigned short times = 200;
    for(int i=0; i<times; i++) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        axo += ax; ayo += ay; azo += az;
        gxo += gx; gyo += gy; gzo += gz;
    }
    axo /= times; ayo /= times; azo /= times;
    gxo /= times; gyo /= times; gzo /= times;

    
}

void loop() {
    webSocket.loop(); 
    server.handleClient();
    read_triple_sensors();

    // Kalman filtering logic
    now = millis();
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float accx = ax / AcceRatio;
    float accy = ay / AcceRatio;
    float accz = az / AcceRatio;

    aax = atan(accy / accz) * (-180) / pi;
    aay = atan(accx / accz) * 180 / pi;
    aaz = atan(accz / accy) * 180 / pi;

    aax_sum = 0;
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1; i<n_sample; i++) {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0;

    aays[n_sample-1] = aay;
    aay_sum += aay * n_sample;
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;

    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * dt;
    float gyroy = - (gy-gyo) / GyroRatio * dt;
    float gyroz = - (gz-gzo) / GyroRatio * dt;
    agx += gyrox;
    agy += gyroy;
    agz += gyroz;
    
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1; i<10; i++) {
        a_x[i-1] = a_x[i];
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;

    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;

    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;

    for(int i=0; i<10; i++) {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    }
    
    Rx = Rx / 9;
    Ry = Ry / 9;
    Rz = Rz / 9;
  
    Px = Px + 0.0025;
    Kx = Px / (Px + Rx);
    agx = agx + Kx * (aax - agx);
    Px = (1 - Kx) * Px;

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;

    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;

    delay(10);

    String data = "{";
    data += "\"sensor1\":" + String(measure1.RangeMilliMeter) + ",";
    data += "\"sensor2\":" + String(measure2.RangeMilliMeter) + ",";
    data += "\"sensor3\":" + String(measure3.RangeMilliMeter) + ",";
    data += "\"ax\":" + String(aax) + ",";
    data += "\"ay\":" + String(aay) + ",";
    data += "\"az\":" + String(aaz) + ",";
    data += "\"gx\":" + String(agx) + ",";
    data += "\"gy\":" + String(agy) + ",";
    data += "\"gz\":" + String(agz) + "}";

    webSocket.broadcastTXT(data);  // send data to all connected clients

    delay(10);
}
