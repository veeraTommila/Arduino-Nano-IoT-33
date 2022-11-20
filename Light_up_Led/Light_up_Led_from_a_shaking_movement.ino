//Anturilevyn sisäisten ledien sytyttäminen kiihtyvyysanturilla.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <WiFiUdp.h>
#include "Adafruit_TSL2591.h"
#include <Wire.h>
#include <Arduino_LSM6DS3.h>
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 7; // the number of the pushbutton pin
/*const int ledPinR = 5; // the number of the RED LED pin
const int ledPinG = 6; // the number of the GREEN LED pin*/
// variables will change:
bool ledStateR = HIGH;
bool ledStateG = LOW;
int counter = 0;
long int alku, loppu, aika; // 32-bittiset aikamuuttujat.

float x, y, z;
int degreesX = 0;
int degreesY = 0;

void mittaa();
void paina();

void setup() {
 /* pinMode(ledPinR, OUTPUT); // initialize the LED pin as an output
  pinMode(ledPinG, OUTPUT); // initialize the LED pin as an output
  pinMode(buttonPin, INPUT); // initialize the pushbutton pin as an input*/
  pinMode(5, OUTPUT); // initialize the RED LED pin as an output
  pinMode(6, OUTPUT); // initialize the GREEN LED pin as an output
  pinMode(buttonPin, INPUT); // initialize the pushbutton pin as an input
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
}

void loop() {
  mittaa();  
}

void mittaa(){// Aliohjelma liikkeen mittaamiseen ja ledeille.
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    //paina();  
  }

  paina(); 

  if (x > 0.1) {
    digitalWrite(5, HIGH);
    //digitalWrite(6, LOW);        
    x = 100 * x;    
    degreesX = map(x, 0, 97, 0, 90);
    Serial.print("Tilting up ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (x < -0.1) {
    digitalWrite(6, HIGH);
    //digitalWrite(5, LOW);      
    x = 100 * x;
    degreesX = map(x, 0, -100, 0, 90);
    Serial.print("Tilting down ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }
  if (y > 0.1) {
    digitalWrite(5, HIGH);
    //digitalWrite(6, LOW);              
    y = 100 * y;
    degreesY = map(y, 0, 97, 0, 90);
    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  if (y < -0.1) {
    digitalWrite(6, HIGH);
    //digitalWrite(6, LOW);        
    degreesY = map(y, 0, -100, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }  
  delay(1000);
/*
  if(digitalRead(buttonPin)== LOW){ //Jos luetun painonapin tila on LOW (arvo on 0). Se menee nollille painonappia painettaessa.
    alku = micros();  //Alku-aikamuuttujaan luetaan kellomuuttujan tieto micros. Eli kuinka monta sekuntia on kulunut virran päällepanosta.
    counter++; // kasvatetaan counter muuttujan arvoa yhdellä
    ledStateR =! ledStateR; // ledStateR muuttujan tila vaihdetaan päinvastaiseksi. Sammutetaan, jos palaa ja sytytetään, jos on sammuksissa.
    digitalWrite(ledPinR, ledStateR); // ledPinR muuttujan arvoksi kirjoitetaan ledStateR muuttujan arvo
    //delay(200); // odotetaan 200 ms ja tehdään vihreälle ledille samat jutut
    ledStateG =! ledStateG; // ledStateG muuttujan tila vaihdetaan päinvastaiseksi. Sammutetaan, jos palaa ja sytytetään, jos on sammuksissa.
    digitalWrite(ledPinG, ledStateG);
    Serial.print("Laskuri: "); // tulostetaan monitoriin vakioteksti
    Serial.print(counter); // tulostetaan monitoriin muuttujan counter arvo
    //delay(200);
    loppu = micros();
    Serial.print(" Koodin suoritusaika: ");
    Serial.print(loppu-alku);
    Serial.println(" mikrosekunttia"); }*/
    
}

void paina(){
  if(digitalRead(buttonPin)== LOW){ //Jos luetun painonapin tila on LOW (arvo on 0). Se menee nollille painonappia painettaessa.    
    alku = micros();  //Alku-aikamuuttujaan luetaan kellomuuttujan tieto micros. Eli kuinka monta sekuntia on kulunut virran päällepanosta.
    counter++; // kasvatetaan counter muuttujan arvoa yhdellä
    ledStateR =! ledStateR; // ledStateR muuttujan tila vaihdetaan päinvastaiseksi. Sammutetaan, jos palaa ja sytytetään, jos on sammuksissa.
    digitalWrite(5, ledStateR); // ledPinR muuttujan arvoksi kirjoitetaan ledStateR muuttujan arvo
    delay(200); // odotetaan 200 ms ja tehdään vihreälle ledille samat jutut
    ledStateG =! ledStateG; // ledStateG muuttujan tila vaihdetaan päinvastaiseksi. Sammutetaan, jos palaa ja sytytetään, jos on sammuksissa.
    digitalWrite(6, ledStateG);
    //Serial.print("Laskuri: "); // tulostetaan monitoriin vakioteksti
    //Serial.print(counter); // tulostetaan monitoriin muuttujan counter arvo
    delay(200);
    loppu = micros();
    Serial.println("Pushed a button.");
  }
}
