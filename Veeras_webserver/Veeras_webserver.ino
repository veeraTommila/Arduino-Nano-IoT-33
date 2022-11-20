#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

//#define SEALEVELPRESSURE_HPA (1013.25)
#define SEALEVELPRESSURE_HPA (1013)
Adafruit_BME280 bme ; // I2C
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "VeeranVerkko";        // your network SSID (name)
char pass[] = "Qwertyuiopå!123";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key index number (needed only for WEP)

const int analogPin = A6; // 
const int analogPin2 = 7; // 
const int analogPin3 = 8; // 
const int pwrPin = 17;
int adcValue = 0;  // variable to store the value read
int t = 0;
int h = 0;
int p = 0;
  
unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;



int status = WL_IDLE_STATUS;
WiFiServer server(80);


void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}


void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
    Serial.println(F("Starting Adafruit TSL2591 Test!"));
  
  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB port only
    Serial.println(F("BME280 test "));
    bme.begin(0x76);
  }
  configureSensor();
  pinMode(pwrPin, OUTPUT);    //  pinni D17 lähdöksi
  digitalWrite(pwrPin, HIGH); //  virrat päälle sensorille 
  
  

  Serial.println("Access Point Web Server");
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

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
}

void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  /*Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);*/
}

/**************************************************************************/
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  Serial.print(F("[ ")); Serial.print(event.timestamp); Serial.print(F(" ms ] "));
  if ((event.light == 0) |
      (event.light > 4294966000.0) | 
      (event.light <-4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    Serial.println(F("Invalid data (adjust gain or timing)"));
  }
  else
  {
    Serial.print(event.light); Serial.println(F(" lux"));
  }
}


void loop() {
  float Temp = bme.readTemperature(); //Muunnetaan lämpötilatieto desimaaliluvuksi.  
  int Humidity = bme.readHumidity();  //Muunnetaan kosteustieto kokonaisluvuksi.
  int Hpa = bme.readPressure(); //Muunnetaan painetieto kokonaisluvuksi.  

  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

    mittaaADC();
    advancedRead();
    

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<meta charset=\"UTF-8\">");
           // CSS to style
            client.println("<style>h1{text-align:center;}table, th, td { padding-left:0px;  border: 0px solid black;  border-collapse: collapse;}table.center {  margin-left: auto;   margin-right: auto;} </style>");
            //client.println("<style>.taulukko {text-align: left; } </style>"); 
            //client.println("<style>table, th, td {  border:1px solid black;}</style>");
            // Web Page Heading                                         
            client.println("</head><body>");                         
            client.println("<div class=\"center\">"); 
            //client.println("<table style=\"width:100%\"> "); 
            client.println("<table class=\"center\"> ");  
            client.println("<h1>The measurements</h1>");
            client.println("</div>");
            client.println("<div class=\"taulukko\">");
            client.println("<tr> ");
            client.println("<td style=\"padding-right: 0px\"> ");
            client.print("Temperature ");
            client.println("</td> ");
            client.println("<td style=\"padding-left: 25px\">");
            client.print(Temp,1);
            client.println(" °C");
            client.println("</td> ");
            client.println("</tr> ");
            client.println("<br> ");
            client.println("<tr> ");
            client.println("<td> ");
            client.print("Humidity ");
            client.println("</td> ");
            client.println("<td style=\"padding-left: 25px\"> ");
            client.print(Humidity);
            client.println(" %");
            client.println("</td> ");
            client.println("</tr> ");
            client.println("<br> ");
            client.println("<tr> ");
            client.println("<td> ");
            client.print("Pressure ");
            client.println("</td> ");
            client.println("<td style=\"padding-left: 25px\"> ");            
            client.print(Hpa);
            client.println(" mBar");
            client.println("</td> ");
            client.println("</tr> ");
            client.println("<br> ");
            client.println("<tr> ");
            client.println("<td> ");
            client.print("Luminosity ");
            client.println("</td> ");
            client.println("<td style=\"padding-left: 25px\"> ");
            client.print(tsl.calculateLux(full, ir)* 1,0);
            client.println(" lux");
            client.println("</td> ");
            client.println("</tr> ");
            client.println("</table> ");
            client.println("</div>");
            client.println("</body></html>");

            // The HTTP response ends with another blank line:
            client.println();
            
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void mittaaADC(){
  adcValue = analogRead(analogPin);  // luetaan A6:n jännitearvo ja muutetaan se digitaaliseksi
}
