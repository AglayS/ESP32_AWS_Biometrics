#include <Arduino.h>
#include "../include/secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#include <OneWire.h>
#include <DallasTemperature.h> 

//pulse ox libraries
#include "MAX30105.h"
#include "heartRate.h"
//#include "spo2_algorithm.h"

MAX30105 particleSensor;
//#define MAX_BRIGHTNESS 255

//other defs
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

//////

// GPIO where the DS18B20 is connected to
const int oneWireBus = 32; 

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"


float c; //celsius float
float f; //fahrenheit float
int bpm; //bpm integer


WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void connectAWS() //connect to AWS 
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  // Create a message handler
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}

void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["time"] = millis(); //milliseconds

  //testing
  //doc["temperature"] = t;
  doc["Celsius"] = c;
  doc["Fahrenheit"] = f;
  //doc["Heartrate"] = hr;
  doc["Beats Per Minute (BPM)"] = bpm;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void setup() {
  
  Serial.begin(9600);

  //pulse ox
  
 Serial.println("Initializing...");

// Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  /////

//particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
// Start the DS18B20 sensor
  sensors.begin();

  connectAWS();


}

/*void temperature() {

/*float temperature = particleSensor.readTemperature();

  f = particleSensor.readTemperatureF();

  Serial.print(" temperatureF=");
  Serial.print(temperatureF, 4);
 //c = sensors.getTempCByIndex(0); //get temp in Celsius
 //f = sensors.getTempFByIndex(0); //get temp in Fahrenheit



  //sensors.requestTemperatures(); //request temperatures be taken 

    float temperature = particleSensor.readTemperature();

  Serial.print("temperatureC=");
  Serial.print(temperature, 4);

  float temperatureF = particleSensor.readTemperatureF(); //Because I am a bad global citizen

  Serial.print(" temperatureF=");
  Serial.print(temperatureF, 4);

  Serial.println();

} */

void loop() {

//temperature();

//pulse ox

long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();


beatsPerMinute = 4 * (delta / 1000.0);
//beatsPerMinute =  (delta / 1000.0);
//beatsPerMinute = 30 / (delta / 1000.0);
    //beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();

  bpm = beatsPerMinute;

////////////
 //t = sensors.requestTemperatures();


c = sensors.getTempCByIndex(0); //get temp in Celsius
 f = sensors.getTempFByIndex(0); //get temp in Fahrenheit

  
 //sensors.requestTemperatures(); //request temperatures be taken 

  publishMessage(); //send message to AWS
  client.loop(); 


  //delay(200); //one second readings
  
}
