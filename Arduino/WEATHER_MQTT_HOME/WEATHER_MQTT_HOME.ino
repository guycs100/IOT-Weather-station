
/*
 * Rui Santos 
 * Complete Project Details http://randomnerdtutorials.com
 */

// Load required libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <NTPClient.h>
#include <WiFiUdp.h>


// Replace with your network and MQTT credentials 
const char* ssid     = "EprSys2";
const char* password = "Epr1182Epr";
//const char* mqttServer = "m15.cloudmqtt.com";
const char* mqttServer = "192.168.43.107";
//const int mqttPort = 15856;
const int mqttPort = 1883;
const char* mqttUser = "lgmxtoso";
const char* mqttPassword = "aD054gOOMp1Q";
const char* topic = "weatherreadings";
char charVal[10];
char charBuf[50];
  
WiFiClient espClient;
PubSubClient client(espClient);

//time from NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String formattedDate;
String dayStamp;
String timeStamp;
String AllInOne;

// uncomment one of the lines below for whatever DHT sensor type you're using
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// GPIO the DHT is connected to
const int DHTPin = 14;
const int RED = 26;
const int GREEN = 13;
const int BLUE = 12;
//intialize DHT sensor
DHT dht(DHTPin, DHTTYPE);

// create a bmp object
Adafruit_BMP085 bmp;

// Web page file stored on the SD card
//File webFile; 

// Set potentiometer GPIO
//const int potPin = 2; //32

// IMPORTANT: At the moment, GPIO 4 doesn't work as an ADC when using the Wi-Fi library
// This is a limitation of this shield, but you can use another GPIO to get the LDR readings
const int LDRPin = 36;
float rawRange = 4096;
float logRange = 5.0;

// variables to store temperature and humidity
float tempC;
float tempF;
float humi;

float RawToLux(int raw)
{
float logLux = (raw * logRange) / rawRange;
return pow(10, logLux);
}

void readDHT(){
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  humi = dht.readHumidity();
  // Read temperature as Celsius (the default)
  tempC = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  tempF = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(humi) || isnan(tempC) || isnan(tempF)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(humi);
  Serial.print(" %\t Temperature (DHT11): ");
  Serial.print(tempC);
  Serial.print(" *C ");
  Serial.print(tempF);
  Serial.println(" *F");

}

void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println("-----------------------");
 
}


void setup(){ 
     
 
  // initialize serial port
  Serial.begin(9600); 
  //LED
  pinMode (RED, OUTPUT);
  pinMode (GREEN, OUTPUT);
  pinMode (BLUE, OUTPUT);
  
  // initialize DHT sensor
  dht.begin();

  // initialize BMP180 sensor
  if (!bmp.begin()){
    Serial.println("Could not find BMP180 or BMP085 sensor");
    while (1) {}
  }


//Init WiFi as Station, start SmartConfig
  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();

  //Wait for SmartConfig packet from mobile
  Serial.println("Waiting for SmartConfig.");
  while (!WiFi.smartConfigDone()) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("SmartConfig received.");

  //Wait for WiFi to connect to AP
  Serial.println("Waiting for WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Print local IP address 
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //Start web server
  //server.begin();
  
  //intialize NTP client to get date and time from an NTP server.
  timeClient.begin();
  timeClient.setTimeOffset(7200);
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  
   //Connect to MQTT server
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    
   while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
 
      Serial.println("connected");
 
    } else {
 
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);
 
    }
    client.subscribe("weather/red");
    client.subscribe("weather/green");
    client.subscribe("weather/blue");
    //client.subscribe("weather/allinone");
  }
}

void loop(){
  client.loop();
  delay(5000);
  if(1==2){
    digitalWrite(RED, HIGH);
    delay(1000);
    digitalWrite(RED, LOW);
    delay(1000);
    digitalWrite(GREEN, HIGH);
    delay(1000);
    digitalWrite(GREEN, LOW);
    delay(1000);
    digitalWrite(BLUE, HIGH);
    delay(1000);
    digitalWrite(BLUE, LOW);
    delay(1000);
  }
  formattedDate = timeClient.getFormattedDate();
  int str_len = formattedDate.length() + 1; 
  char char_array[str_len];
  formattedDate.toCharArray(char_array, str_len);
  client.publish("weather/Time",char_array);
  readDHT();
 
  client.publish("weather/tempC",dtostrf(tempC, 4, 4, charVal));
  client.publish("weather/tempF",dtostrf(tempF, 4, 4, charVal));
  client.publish("weather/humi",dtostrf(humi, 4, 4, charVal));
  
  float currentTemperatureC = bmp.readTemperature();
  client.publish("weather/currentTemperatureC",dtostrf(currentTemperatureC, 4, 4, charVal));
  
  float currentTemperatureF = (9.0/5.0)*currentTemperatureC+32.0;
  client.publish("weather/currentTemperatureF",dtostrf(currentTemperatureF, 4, 4, charVal));
  
  float Pressure=bmp.readPressure();
  client.publish("weather/Pressure",dtostrf(Pressure, 4, 4, charVal));
  
  float Altitude=bmp.readAltitude();
  client.publish("weather/Altitude",dtostrf(Altitude, 4, 4, charVal));
  
  float Altitude2=bmp.readAltitude(101500);
  client.publish("weather/Altitude2",dtostrf(Altitude2, 4, 4, charVal));

  //client.publish(analogRead(potPin));


  // IMPORTANT: Read the note about GPIO 4 at the pin assignment 
  float Light=RawToLux(analogRead(LDRPin));
  client.publish("weather/Light",dtostrf(Light, 4, 4, charVal));

  AllInOne=formattedDate+','+tempC+','+tempF+','+humi+','+currentTemperatureC+','+currentTemperatureF+','+Pressure+','+Altitude+','+Altitude2+','+Light;
  str_len = AllInOne.length() + 1; 
  char char_array2[str_len];
  AllInOne.toCharArray(char_array2, str_len);
  client.publish("weather/allinone",char_array2);

  Serial.print(" %\t Temperature (BMP180): ");
  Serial.print(bmp.readTemperature());
  Serial.print(" *C");
  Serial.print(" %\t Pressure: ");
  Serial.print(Pressure);
  Serial.println(" *bar");
  Serial.print(" %\t Light: ");
  Serial.print(RawToLux(analogRead(LDRPin)));
  Serial.println(" *lum");
  Serial.print("Altitude = ");
  Serial.print(Altitude);
  Serial.println(" M");
  Serial.print("Altitude2 = ");
  Serial.print(Altitude2);
  Serial.println(" M");



}
