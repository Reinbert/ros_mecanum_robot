/*
  
  Arduino code to control a robot with mecanum wheels via ROS Twist messages using
  a NodeMcu or ESP8266 and a PCA 9685 servo controller, like this one from Adafruit
  (https://www.adafruit.com/product/815).
  
  If you have questions or improvements email me at reinhard.sprung@gmail.com

  Launch a ros serial server to connect to:
    roslaunch rosserial_server socket.launch

  Launch a teleop gamepad node:
    roslaunch teleop_twist_joy teleop.launch joy_config:="insert gamepad type"
 
  MIT License

  Copyright (c) 2019 Reinhard Sprung

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include "fastPCA9685.h"

#define LED_BUILTIN 2 // Remapping the built-in LED since the NodeMcu apparently uses a different one.
#define LED_BUILTIN_RED 16 // If using a NodeMcu v1, then there's another red onboard led.
// The min amount of PWM the motors need to move. Depends on the battery, motors and controller.
// The max amount is defined by PWMRANGE in Arduino.h, but we redefine it to our 12 bit range.
#define PWM_MIN 400
#define PWMRANGE 4095

// The name of the OTA port
#define OTA_HOSTNAME "SMARTCAR"

// If access point is defined, a Wifi network with this name will be created.
// Remove if you want to connect to an existing network.
//#define ACCESS_POINT_SSID "SMARTCAR"

#ifdef ACCESS_POINT_SSID
  // The ROS serial server address assigned by the NodeMcu access point.
  IPAddress server(192, 168, 4, 2);
#else
  // Override standard settings with user data. Example:
  // const char* SSID = "SSID";
  // const char* PASSWORD = "PASSWORD";
  // IPAddress server(192, 168, 0, 20);
  #include "user.h"
#endif


// Declare functions
void setupPins();
void setupSerial();
void setupWifi();
void setupOta();
void setupRos();
void setupPwm();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
void handleMovement();
float mapPwm(float x, float out_min, float out_max);

// How the PWM pins are connected to the motor controllers.
// LF_BACK = left front wheel forward direction, etc...
const uint8_t LF_FORW = 15;
const uint8_t LF_BACK = 14;
const uint8_t RF_FORW = 12;
const uint8_t RF_BACK = 13;
const uint8_t LB_FORW = 0;
const uint8_t LB_BACK = 1;
const uint8_t RB_FORW = 3;
const uint8_t RB_BACK = 2;

// PWM library
FastPCA9685 pwm;// = FastPCA9685();

// ROS serial server
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

bool connected = false;
// Once per loop those values are transformed into pwm data.
bool movement = false;
bool updating = false;
float x, y, z = 0;

void setup()
{
  setupPins();
  setupPwm();
  setupSerial();
  setupWifi();
  setupOta();
  setupRos();
}

void setupPins()
{
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void setupPwm()
{
  Wire.setClock(400000);
  pwm.reset();
}

void setupSerial()
{
  Serial.begin(115200);
  Serial.println();
}

void setupWifi()
{
  // Don't write Wifi config to EEPROM to extend its lifetime. 
  WiFi.persistent(false);
  
#ifdef ACCESS_POINT_SSID

  WiFi.disconnect();
  Serial.println("Creating Wifi network");
  if (WiFi.softAP(ACCESS_POINT_SSID))
  {
    Serial.println("Wifi network created");
    Serial.print("SSID: ");
    Serial.println(WiFi.softAPSSID());
    Serial.print("IP:   ");
    Serial.println(WiFi.softAPIP());
  }

#else

  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  Serial.println("Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());

#endif
}

void setupOta()
{
  ArduinoOTA.onStart([]() {
    // Stop and handle it immediately, then set updating to true to prevent further commands.
    stop();
    handleMovement();
    updating = true;
  });
  
  ArduinoOTA.onEnd([]() {
    updating = false;
    digitalWrite(LED_BUILTIN, HIGH);
  });

  // Blink the progress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    digitalWrite(LED_BUILTIN, (100 * progress / total) % 2); // false -> on, true -> off
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();
  Serial.printf("OTA ready\n");
}

void setupRos()
{
  // Connect to rosserial socket server and init node. (Using default port of 11411)
  Serial.printf("Connecting to ROS serial server at %s\n", server.toString().c_str());
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
}

void stop()
{
  x = y = z = 0;
  movement = true;
}

void onTwist(const geometry_msgs::Twist &msg)
{
  if (connected)
  {
    // Cap values at [-1 .. 1]
    x = max(min(msg.linear.x, 1.0f), -1.0f);
    y = max(min(msg.linear.y, 1.0f), -1.0f);
    z = max(min(msg.angular.z, 1.0f), -1.0f);
    movement = true;
  }
  else
    stop();
}

void handleMovement()
{
  if (!movement || updating)
    return;

  // Mecanum drive:
  // ------------------------
  
  // Taken and simplified from: http://robotsforroboticists.com/drive-kinematics/
  float lf = x - y - z * 0.5;
  float rf = x + y + z * 0.5;
  float lb = x + y - z * 0.5;
  float rb = x - y + z * 0.5;

  // Map values to PWM intensities. 
  // PWMRANGE = full speed, PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lfPwm = mapPwm(fabs(lf), PWM_MIN, PWMRANGE);
  uint16_t rfPwm = mapPwm(fabs(rf), PWM_MIN, PWMRANGE);
  uint16_t lbPwm = mapPwm(fabs(lb), PWM_MIN, PWMRANGE);
  uint16_t rbPwm = mapPwm(fabs(rb), PWM_MIN, PWMRANGE);

//  Serial.printf("%f, %f, %f, %f\n", lf, rf, lb, rb);
//  Serial.printf("---%d, %d, %d, %d\n", lfPwm, rfPwm, lbPwm, rbPwm);
//  Serial.printf("------%d, %d, %d, %d\n", lfPwm * (lf > 0), lfPwm * (lf < 0), lbPwm * (lb > 0), lbPwm * (lb < 0));

  // Each wheel has a channel for forward and backward movement
  pwm.writePWM(LF_FORW, lfPwm * (lf > 0));
  pwm.writePWM(LF_BACK, lfPwm * (lf < 0));
  pwm.writePWM(LB_FORW, lbPwm * (lb > 0));
  pwm.writePWM(LB_BACK, lbPwm * (lb < 0));
  
  pwm.writePWM(RF_FORW, rfPwm * (rf > 0));
  pwm.writePWM(RF_BACK, rfPwm * (rf < 0));
  pwm.writePWM(RB_FORW, rbPwm * (rb > 0));
  pwm.writePWM(RB_BACK, rbPwm * (rb < 0));

  movement = false;
}

void loop()
{
  ArduinoOTA.handle();
  rosConnected();
  node.spinOnce();
  handleMovement();
  //delay(10);
}

bool rosConnected()
{
  // If value changes, notify via LED and console.
  bool conn = node.connected();
  if (connected != conn)
  {
    connected = conn;
    if (!connected)
      stop();
      
    digitalWrite(LED_BUILTIN, !connected); // false -> on, true -> off
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
