#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_SSD1306.h>
#include "QMC5883LCompass.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64// OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address
MPU6050 accelgyro; 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool dotVisible = false;
QMC5883LCompass compass;

unsigned long now, lastTime = 0;
float dt;                                   //time 

int16_t ax, ay, az, gx, gy, gz;             //Accelerometer Gyro Raw Data
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    //Angle variable
long axo = 0, ayo = 0, azo = 0;             //Accelerometer offset
long gxo = 0, gyo = 0, gzo = 0;             //Gyroscope offset

float pi = 3.1415926;
float AcceRatio = 16384.0;                  //Accelerometer scaling factor

float GyroRatio = 131.0;                    //Gyro scale factor

uint8_t n_sample = 8;                       //Accelerometer Filter Algorithm Samples
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x,yAxis sampling queue
long aax_sum, aay_sum,aaz_sum;                      //x,yAxis sampling and

float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; //加速度计协方差计算队列
float Px=1, Rx, Kx, Sx, Vx, Qx;             //x Axis Kalman Variable
float Py=1, Ry, Ky, Sy, Vy, Qy;             //y Axis Kalman Variable
float Pz=1, Rz, Kz, Sz, Vz, Qz;

void setup(void) {
  Serial.begin(115200);
  compass.init();
  accelgyro.initialize(); 
  unsigned short times = 200;             //Number of samples
    for(int i=0;i<times;i++)
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Read six-axis raw values
        axo += ax; ayo += ay; azo += az;      //Sampling and
        gxo += gx; gyo += gy; gzo += gz;
    
    }
    axo /= times; ayo /= times; azo /= times; //Calculate accelerometer offset
    gxo /= times; gyo /= times; gzo /= times; //Calculating Gyro Offset
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();
  delay(10);
}
String getDirection(float headingDegrees) {
  if (headingDegrees >= 22.5 && headingDegrees < 67.5) {
    return "NE";
  } else if (headingDegrees >= 67.5 && headingDegrees < 112.5) {
    return "EAST";
  } else if (headingDegrees >= 112.5 && headingDegrees < 157.5) {
    return "SE";
  } else if (headingDegrees >= 157.5 && headingDegrees < 202.5) {
    return "SOUTH";
  } else if (headingDegrees >= 202.5 && headingDegrees < 247.5) {
    return "SW";
  } else if (headingDegrees >= 247.5 && headingDegrees < 292.5) {
    return "WEST";
  } else if (headingDegrees >= 292.5 && headingDegrees < 337.5) {
    return "NW";
  } else {
    return "NORTH";
  }
}

void loop(void) {
  compass.read();
  dotVisible = !dotVisible;

  float heading = atan2(compass.getY(), compass.getX());

  // Update this value with the current declination for Dhaka, Mirpur
  float declinationAngle = -0.53; // Magnetic declination in degrees
  heading += declinationAngle * PI / 180; // Convert declination to radians

  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;

  float headingDegrees = heading * 180 / M_PI;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Display angle in degrees
  display.setTextSize(1); // Use larger text size for angle
  display.setCursor(98, 42);
  display.print(headingDegrees, 0);
  display.print((char)247);

  // Draw compass
  int centerX = SCREEN_WIDTH - 19;
  int centerY = SCREEN_HEIGHT / 3;
  int radius = min(centerX, centerY) - 3;

  display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
  display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
  display.drawLine(centerX, centerY, centerX + radius * cos(heading - PI / 2), centerY + radius * sin(heading - PI / 2), SSD1306_WHITE);

  // Display current direction
  display.setTextSize(1.5); // Use larger text size for direction
  display.setCursor(98, 52);
  display.print(getDirection(headingDegrees));
    
  

    // Adjust the delay to control the blink speed
    delay(10);

    unsigned long now = millis();             //current time(ms)
    dt = (now - lastTime) / 1000.0;           //Differential time(s)
    lastTime = now;                           //Last sampling time(ms)

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Read six-axis raw values

    float accx = ax / AcceRatio;              //x Axis acceleration
    float accy = ay / AcceRatio;              //y Axis acceleration
    float accz = az / AcceRatio;              //z Axis acceleration

    aax = atan(accy / accz) * (-180) / pi;    //y Angle of the axis to the z axis
    aay = atan(accx / accz) * 180 / pi;       //x Angle of the axis to the z axis
    aaz = atan(accz / accy) * 180 / pi;       //z Angle of the axis to the z axis

    aax_sum = 0;                              // Sliding weighted filtering algorithm for accelerometer raw data
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; //角度调幅至0-90°
    aays[n_sample-1] = aay;                        //此处应用实验法取得合适的系数
    aay_sum += aay * n_sample;                     //本例系数为9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * dt; //x轴角速度
    float gyroy = - (gy-gyo) / GyroRatio * dt; //y轴角速度
    float gyroz = - (gz-gzo) / GyroRatio * dt; //z轴角速度
    agx += gyrox;                             //x轴角速度积分
    agy += gyroy;                             //x轴角速度积分
    agz += gyroz;
    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 //测量值平均值运算
        a_x[i-1] = a_x[i];                      //即加速度平均值
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 //x轴加速度平均值
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 //y轴加速度平均值
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;

    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              //得到方差
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         // 0.0025在下面有说明...
    Kx = Px / (Px + Rx);                      //计算卡尔曼增益
    agx = agx + Kx * (aax - agx);             //陀螺仪角度与加速度计速度叠加
    Px = (1 - Kx) * Px;                       //更新p值

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Gx: "); display.println(agx);
  display.setCursor(0,10);
  display.print("Gy: "); display.println(agy);

  // Display accelerometer data
  display.setCursor(0,20);
  display.print("Xacc: "); display.println(accx);
  display.setCursor(0,30);
  display.print("Yacc: "); display.println(accy);

  display.display();
  delay(100); // Update rate

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);

    // Display MPU6050 and Compass data
    // ... (your code for displaying sensor data)

    // Display a blinking dot
    if (dotVisible) {
        display.drawPixel(94, 53, SSD1306_WHITE);
        display.drawPixel(94, 56, SSD1306_WHITE);
        display.drawPixel(94, 59, SSD1306_WHITE);
        display.setCursor(50,52);
        display.print("HEADING");
    }

    display.display();
    
}



