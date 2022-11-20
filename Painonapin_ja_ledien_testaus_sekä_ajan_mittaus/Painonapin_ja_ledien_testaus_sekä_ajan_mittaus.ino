// constants won't change. They're used here to set pin numbers:
const int buttonPin = 7; // the number of the pushbutton pin
const int ledPinR = 5; // the number of the RED LED pin
const int ledPinG = 6; // the number of the GREEN LED pin
// variables will change:
bool ledStateR = HIGH;
bool ledStateG = LOW;
int counter = 0;
long int alku, loppu, aika; // 32-bittiset aikamuuttujat.
void setup() {
  pinMode(ledPinR, OUTPUT); // initialize the LED pin as an output
  pinMode(ledPinG, OUTPUT); // initialize the LED pin as an output
  pinMode(buttonPin, INPUT); // initialize the pushbutton pin as an input
  Serial.begin(9600);
}
void loop() {
// Pollausta keskeytyksen sijaan. 
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
    Serial.println(" mikrosekunttia");
  }
}