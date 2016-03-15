// IOT RalliDroid  
// Copyright Jukka Raivio and Jussi Salonen 2016
// MIT License
// 
// Example code is based on Arduino Mega2560 
//
// This arduino demo setup is connected to esp8266 Wifi via serial. 
// The http services are managed by esp-link firmware in ESP. The esp unit is working as Wifi-Serial bridge.
// No code changes are required for the esp-link
// Github repository for esp-link https://github.com/jeelabs/esp-link

// Temperature & Humidity sensor DHT11 lib & setup
// Adafruit version, https://github.com/adafruit/DHT-sensor-library
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
#include "DHT.h"
#include <ELClient.h>
#include <ELClientMqtt.h>
#include <SPI.h> // Arduino default SPI library
#include <MFRC522.h> 
#include <Ultrasonic.h>
#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <ArduinoJson.h>

//#define SERIAL_RX_BUFFER_SIZE 256

char bufff[512];

bool connected;

//static int count;
//static uint32_t last;

ELClient esp(&Serial1, &Serial);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);


#define DHTPIN  2       // 2 digital pin 
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);


// I2C library
// RTC library by adafruit https://github.com/adafruit/RTClib
#include "RTClib.h"
RTC_DS1307 RTC;
String TimeStr ="";

// Multitasking functions by Arduino millis
// Initialization to enable multithreading features by using millis() functions  
// Sensor update interval
unsigned long previousMillis = 0;       // will store last time sensor updated
unsigned long tuneMillis = 0;
const long tempo = 180;
bool tuneOn = false;
static uint8_t phase = 0;

unsigned int tune[32] = {659,587,370,370,415,415,
                         554,494,294,294,330,330,
                         494,440,277,277,330,330,
                         440,440,0,0,
                         0,0,0,0
                         };  //
/* E5 
 * D5
 * Fh4
 * Gh4
 * 
 * Ch5
 * B4
 * D4
 * E4
 * 
 * B4
 * A4
 * Ch4
 * E4
 * 
 * A4
 */
  

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
const long interval = 1000;            // Sensor data sending interval

// Json parser lib Copyright Benoit Blanchon 2014-2016, MIT License
// https://github.com/bblanchon/ArduinoJson
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
const char* command;
// Serial data handling, global variables
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char inChar; // a char to handle incoming data

// RFID tag reader (NFC) MFRC522
/* ------------------------------------
  * RFID Card reader MFRC522 library by Miguel Balboa https://github.com/miguelbalboa/rfid
  * Signal      MFRC522      Mega 2560
  *             Pin          Pin       ´
  * -------------------------------------
  * RST/Reset   RST          49         
  * SPI SS      SDA(SS)      53       
  * SPI MOSI    MOSI         51        
  * SPI MISO    MISO         50        
  * SPI SCK     SCK          52        
*/
#define RST_PIN   49     // Configurable, see typical pin layout above (valk)
#define SS_PIN    53    // Configurable, see typical pin layout above (vihr)
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key; 
// Init array that will store new NUID 
byte nuidPICC[3];

/*
 * Gyroscope + accelerometer MPU-6050 library for Arduino by Jeff Rowberg
 * https://github.com/jrowberg/i2cdevlib
 * 
 */

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// We use I2C address 0x69 as the default (0x68) is used by RTC
// Due to this AD0 pin must be pulled high
MPU6050 accelgyro(0x69);
HMC5883L magneto;

// Global variables to store the values
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;


// Ultrasonic HC-SR04 library for Arduino by J.Rodrigo https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
// Max distance 51 cm
#define TRIG_PIN 10     // Configurable Arduino pins for HC-SR04 Trig PIN
#define ECHO_PIN 9  // Configurable Arduino pins for HC-SR04 Echo pins
Ultrasonic ultrasonic(TRIG_PIN,ECHO_PIN); 

// Edge sensor pinouts
#define right_edge 23 // Right sensor
#define left_edge 22 // Left sensor

// Motor driver Setup
// Pins L298N -> Mega board
int enA = 8;
int in1 = 4;
int in2 = 5;
int in3 = 6;
int in4 = 7;
int enB = 3;
boolean motor_active = false;
// for motor delay
unsigned long mStartMillis = 0;      // will store last time motor delay update
int course; // direction
int mspeed; // motor speed
int mdelay; // motor delay
// millis for sensor reading interval while moving, delay 500 ms
boolean sensor_moving = false;
int sensor_moving_reports = 500;
unsigned long sensor_moving_previousMillis = 0;      // will store last time light delay update

// light blinking 
boolean pin13_blinking = false;
boolean pin12_blinking = false;
boolean pin11_blinking = false;
int pin13_delay;
int pin12_delay;
int pin11_delay;
unsigned long pin13_previousMillis = 0;      // will store last time light delay update
unsigned long pin12_previousMillis = 0;      // will store last time light delay update
unsigned long pin11_previousMillis = 0;      // will store last time light delay update





void HandleIncomingJson() {
    StaticJsonBuffer<512> jsonInBuffer;                 
    const char *JsonChar = inputString.c_str(); // 1 KB
    JsonObject& root = jsonInBuffer.parseObject(JsonChar);
    int pin;
    int value;
    int ldelay;
    
    // Verify Json 
    if (JsonChar!=NULL && !root.success()) {
      Serial.println("parseObject() failed: ");
      Serial.println(JsonChar);
    }
    else {
        // Led pins 13-11
        command = root["command"];
        pin = root["data"][0];
        value = root["data"][1];
        ldelay = root["data"][2];
        
        course = root["mdata"][0];
        mspeed = root["mdata"][1];
        mdelay = root["mdata"][2];
    if (memcmp(command, "lights", 5) == 0) {
      //Serial.println("Valot osu!");
    // Lights on, not blinking
      if (value == 1 && (ldelay == 0 || ldelay == NULL)){
        if (pin == 13) { pin13_blinking = false; }
        if (pin == 12) { pin12_blinking = false; }   
        if (pin == 11) { pin11_blinking = false; }  
        digitalWrite(pin, HIGH);              
      }
      // blinking lights
      else if (value == 1 && (!ldelay == 0 || !ldelay == NULL)){
        if (pin == 13) { pin13_blinking = true; pin13_delay = ldelay;}
        if (pin == 12) { pin12_blinking = true; pin12_delay = ldelay;}   
        if (pin == 11) { pin11_blinking = true; pin11_delay = ldelay;}              
      }
      // Lights off
      else if (value == 0 ){
        if (pin == 13) { pin13_blinking = false; }
        if (pin == 12) { pin12_blinking = false; }   
        if (pin == 11) { pin11_blinking = false; }  
        digitalWrite(pin, LOW);
        }
      }
    if (memcmp(command, "drive", 5) == 0) {
      motor_active = true; 
      mStartMillis = millis();
    }
    if (memcmp(command, "music", 5) == 0) {
      if(pin) {
        tuneOn = true;
        phase = 0;
      }
      else {
        tuneOn = false;
        noTone(24);
      }
    }
      Serial.println("-------");
      Serial.println(command);
      Serial.println(motor_active, DEC);
      Serial.println(course, DEC);
      Serial.println(mspeed, DEC);
      Serial.println(mdelay, DEC);
    
    }
  // returning the default state of serialEvent1()
  stringComplete = false;
  // clean json incoming data buffers
  pin = NULL; value = NULL; ldelay = NULL;
  inputString="";
  inChar = NULL; JsonChar = NULL;
  return;      
}

void readTime() {

    DateTime now = RTC.now();
    String Year = String(now.year(), DEC);
    String Month = String(now.month(), DEC);
    String Day = String(now.day(), DEC);
    String Hour = String(now.hour(), DEC);
    String Minutes = String(now.minute(), DEC);
    String Seconds = String(now.second(), DEC);
    TimeStr = Year + "/" + Month + "/" + Day + ":" + Hour + ":" + Minutes + ":" + Seconds;
    // Debugging
    // Serial.print(TimeStr);
    // Serial.println();

}

void setup() {
  delay(2500);
    // initialize both serial ports:
    Serial.begin(115200); // Debugging
    delay(1000);
    Serial1.begin(115200);

    Serial.println(SERIAL_RX_BUFFER_SIZE);
    esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
    bool ok;
    do {
      ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
      if (!ok) Serial.println("EL-Client sync failed!");
    } while(!ok);
    Serial.println("EL-Client synced!");
  
    // Set-up callbacks for events and initialize with es-link.
    mqtt.connectedCb.attach(mqttConnected);
    mqtt.disconnectedCb.attach(mqttDisconnected);
    mqtt.publishedCb.attach(mqttPublished);
    mqtt.dataCb.attach(mqttData);
    mqtt.setup();
  
    //Serial.println("ARDUINO: setup mqtt lwt");
    //mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");
  
    Serial.println("EL-MQTT ready");

    
    delay(1000);
    Wire.begin(); // start I2C & RTC

    magneto.initialize();
    accelgyro.initialize();
    RTC.begin();
    if (! RTC.isrunning()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      //RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    // initialize digital pin 13-11 as an output (for LEDs)
    pinMode(13, OUTPUT); // Main lights leds
    pinMode(12, OUTPUT); // Direction lights
    pinMode(11, OUTPUT);
    // Egde sensor input pins
    pinMode(right_edge,INPUT); // Right sensor
    pinMode(left_edge,INPUT); // Left sensor
    inputString.reserve(256); // reserve 256 bytes for the inputString:
    dht.begin(); // Init DHT
    // RFID setup
    SPI.begin(); // Init SPI bus
    rfid.PCD_Init(); // Init MFRC522 
    for (byte i = 0; i < 6; i++) { key.keyByte[i] = 0xFF; } // RFID byte handling
}



void loop() {
  // update interval
  unsigned long currentMillis = millis();
  int orig_angle = 0;
  int too_much = 0;
  
  // DHT Sensor reporting
  esp.Process();

  
  if (currentMillis - previousMillis >= interval && connected) {
    // save the last time of sensor reporting
    readTime();
    previousMillis = currentMillis;    
    //JsonReportSensorDHT();
    if (motor_active == false) {
     JsonReportSensorDistance(); 
     //JsonReportSensorEdge();
     //JsonReportSensorAccAndGyro();
     //JsonReportSensorMagneto();
    } 
  }

  if (tuneOn && currentMillis - tuneMillis >= tempo) {
    // save the last time of sensor reporting
    tuneMillis = currentMillis;
    phase++;
    if(tune[phase] == 0)
      noTone(24);
    else
      tone(24, tune[phase % 32]);
  }
  /*if(tuneOn == false) //Change to callback function
  {
    noTone(24);
  
  }*/
  
  if (motor_active == true && connected) { // speed up reporting frequency in case of moving
    currentMillis = millis();
    if (currentMillis - mStartMillis >= sensor_moving_reports ) {
      readTime();
      JsonReportSensorDistance();
      //JsonReportSensorEdge();
      JsonReportSensorAccAndGyro();
      JsonReportSensorMagneto();
    }
  }
  
  // Look for new RFID cards
  if ( rfid.PICC_IsNewCardPresent()) { JsonReportSensorRFID(); }
  
  if (stringComplete == true) { HandleIncomingJson(); }
  
  // Light blinking
  if ( pin13_blinking == true){
    currentMillis = millis();
    if ( currentMillis - pin13_previousMillis >= pin13_delay ) {
        pin13_previousMillis = currentMillis; 
        digitalWrite(13, digitalRead(13)^1);
    }
  }                       
  if ( pin12_blinking == true){
    currentMillis = millis();
    if (currentMillis - pin12_previousMillis >= pin12_delay) {
      pin12_previousMillis = currentMillis; 
      digitalWrite(12, digitalRead(12)^1);
    }
  }
  if ( pin11_blinking == true){
    currentMillis = millis();
    if ( currentMillis - pin11_previousMillis >= pin11_delay) {
      pin11_previousMillis = currentMillis; 
      digitalWrite(11, digitalRead(11)^1);
    }
  }     
  // Drive motors
  if (motor_active == true ) {
     currentMillis = millis();
     orig_angle = get_angle();
     driveMotors();
     if (currentMillis - mStartMillis >= mdelay) {       
      stopMotors();
      too_much = 0;
      while(fix_angle(get_angle_delta(orig_angle))) {
        if (too_much++ > 10)
          break;
      }
      mqtt.publish("iot-uplink", "{\"sensor\":\"drivedone\"}");
     }           
  }  
}
