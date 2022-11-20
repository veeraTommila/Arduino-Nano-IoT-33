#include <Arduino.h>

/*
Harjoitus 7, Veera Tommila

 This sketch connects to a a web server and makes a request
 using a WiFi equipped Arduino board.

 created 23 April 2012
 modified 31 May 2012
 by Tom Igoe
 modified 13 Jan 2014
 by Federico Vanzati
 modified 1 Nov 2022
 by Jukka Ihalainen
 modified 20 Nov 2022
 by Veera Tommila

 http://www.arduino.cc/en/Tutorial/WifiWebClientRepeating (ei löydy enää tästä linkistä)
 This code is in the public domain.
 */
 

#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <WiFiUdp.h>
#define SEALEVELPRESSURE_HPA (1013.25)
// Presentations of the calling functions.
void mittaa();
void printWifiStatus();
void httpRequest();

Adafruit_BME280 bme;  //BME280-anturin ilmentymä, jota käytetään anturitietojen lukemiseen.

char ssid[] = "Your_home_network_name";        // The name of your local network.
char pass[] = "Your_home_network_password";    // The password of your local network.
char apikey1[] = "LsQFUsLm7u0Tce2wU8PhCuTmmlKwzcYg"; //apikey of first sensor
int keyIndex = 0;            // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;  // Status-muuttuja, joka on määritelty WL_IDLE_STATUkseksi.

// Initialize the WiFi asiakas library
WiFiClient asiakas;

// server address:
char server[] = "asksensors.com";

int temp, pressure, humidity;
unsigned long lastConnectionTime = 0;            // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 300L * 1000L; // delay 5 min between updates, in milliseconds. Kuinka tiheästi tietoa halutaan palveluun lähettää? 5:n minuutin välein.

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //bme.begin(0x76);
  while (!Serial) {
    //;
    bme.begin(0x76); 
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

   String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);  // ssid and pass as given parameters. The info will be returned to the status variable.
    // wait 10 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the status:
  printWifiStatus();
}



void loop() {
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  while (asiakas.available()) {  //If the connection exists.
    char c = asiakas.read(); //To receive marks from a server, save them to the variable c and print them.
    Serial.write(c);
  }
  
  //After 1 min, get sensor readings and connect to server:
  if ((millis() - lastConnectionTime) > postingInterval) {    
    mittaa();
    httpRequest(); //request function for all the other sensors
  }  
}

// Tämä funktio on mittauksia varten.
void mittaa(){
    //Mittauksia tulee, jos laittaa setup()-funktioon bme.begin(0x76)nin Serial.begin(9600)nin jälkeen tai while(!Serial)-silmukkaan.
    temp = bme.readTemperature(); 
    pressure = bme.readPressure();
    humidity = bme.readHumidity();
}


//	This method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the NINA module
  asiakas.stop();

  // if there's a successful connection:
  if (asiakas.connect(server, 80)) {
    Serial.println("connecting...");

    String url = "https://asksensors.com/api.asksensors/write/"; //For sensor
    url += apikey1;
    url += "?module1="; //Pressure: line
    url += pressure;
    url += "&module2="; //Humidity: line
    url += humidity;
    url += "&module3="; //Temperature: line
    url += temp;
    url += "&module4="; //Temperature: table
    url += temp;
    
    // send the HTTP GET request:
    Serial.print("********** requesting URL: ");
      Serial.println(url);
    asiakas.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + server + "\r\n");
    asiakas.println(); 
    
    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
