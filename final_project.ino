/*********************************************************************
 Bluetooth Setup
*********************************************************************/
#include <string.h>
#include <Arduino.h>
#include "Tone.h"
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <LiquidCrystal.h>

#include "BluefruitConfig.h"


#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"

// Create the software serial bluefruit object

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/*==========================EASTER EGG=================================*/

//Mario main theme melody
int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,

  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,

  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};
//Mario main them tempo
int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};


/*=========================================================================*/

int state = 0; // 0 -> disarmed, 1 -> armed, 2 -> alarm triggered

const int greenPin = 2;
const int redPin = 3;

const int echoPin = 4;
const int trigPin = 5;


const int buzzerPin = 6;

const int rs = A5, en = A4, d4 = A3, d5 = A2, d6 = A1, d7 = A0;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*Keypad control notes: 
 * Up = Easter Egg,
 * Down = Clear, 
 * Left = Delete, 
 * Right = Enter, 
 * Number Pad = Password
*/
//Password
const String passwordMaster = "1234";
const String easterMaster = "1223334444";
String password = "";

void setup() {
  state = 0;
  
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  

  /* Initialise Bluefruit */
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("ESE 111 Adafruit Bluefruit Lab"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset. This happens pretty freqeuntly the first time you switch computers or change the sketch.\nPlease try re- uploading the code 1 or 2 more times."));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Print Bluefruit information */
  //ble.info();

  char command[BUFSIZE+1];
  Serial.println(F("************************************************************"));
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("************************************************************"));

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // Check BLE version
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println("\n ********BLE PAIRED SUCCESSFULLY. READY TO GO!!!********\n");
  Serial.println(F("************************************************************"));




  /* LCD SETUP */
  lcd.begin(16, 2);
}

unsigned long previousMillis = 0;
int ledState = HIGH;


void loop() {
  //State controls
  String stateString = "";
  if (state == 0){ // Disarmed State
    Serial.println("ALARM IS DISARMED");
    
    //Alarm sound: Reset
    noTone(buzzerPin);
    digitalWrite(buzzerPin, LOW);
    

    //Solid Green
    digitalWrite(greenPin, HIGH);
    digitalWrite(redPin, LOW);

    //LCD Display
    stateString = "DISARMED";
  } else if (state == 1) { // Alarmed State
    Serial.println("ALARM IS ARMED");
    
    //Alarm sound: Reset
    noTone(buzzerPin);
    
    //Solid Red 
    digitalWrite(redPin, HIGH); 
    digitalWrite(greenPin, LOW); 

    //Check for movement
    long dur, dist;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    dur = pulseIn(echoPin, HIGH);
    dist = (dur / 2) / 29.1;
    Serial.println("DISTANCE: ");
    Serial.print(dist);
    Serial.println();
    if(dist < 30){
      state = 2;
      Serial.println("Movment detected!!!!");
    }

    //LCD Display
    stateString = "ARMED";
    
  } else if (state == 2){ // Alarm Triggered State
    Serial.println("ALARM IS TRIGGERED");
    
    //Alarm sound: Set
    tone(buzzerPin, 1000);
    
    //Blinking Red Light
    unsigned long currentMillis = millis();
    digitalWrite(greenPin, LOW);

    if(currentMillis - previousMillis >= 200){
      previousMillis = currentMillis;
      if (ledState == HIGH){
        ledState = LOW;
      } else {
        ledState = HIGH;
      }
    }
    digitalWrite(redPin, ledState);

    //LCD Display
    stateString = "TRIGGERED";
  }

  /* LCD Controller */
  lcd.clear();
  lcd.print("State:");
  lcd.setCursor(7, 0);
  lcd.print(stateString);

  lcd.setCursor(0, 1);
  lcd.print("Pass:");
  lcd.setCursor(6, 1);
  lcd.print(password);
  

  //Check Bluetooth connection for state change
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  /* Wait for new data to arrive */
  if (len == 0) return;

  //Check if a button was pressed
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
//    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      if (buttnum >= 1 && buttnum <= 4){
        if(password.length() < 10) {
          password += buttnum;
        }
      }
      
      if (buttnum == 5){ //Easter Egg = Up Arrow
        if (password.equals(easterMaster)){
          password = "";
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Super Mario");
          lcd.setCursor(0, 1);
          lcd.print("By Kogan, Penn");
          easterEgg();
        }
      }
      if (buttnum == 6){ //Clear = Down Arrow
        password = "";
      }
      if (buttnum == 7) { //Delete = Left arrow
        if (password.length() > 0){
          password.remove(password.length() - 1, 1);
        }
      }
      if (buttnum == 8) { //Enter = Right Arrow
        if (password.equals(passwordMaster)){
          password = "";
          if (state == 0){
            state = 1;
          } else {
            state = 0;
          }
        }
      }
    }
  } else {
    Serial.println(" released");
  }
}

void easterEgg(){
  Serial.println(" 'Mario Theme'");
  int size = sizeof(melody) / sizeof(int);
  for (int thisNote = 0; thisNote < size; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / tempo[thisNote];

    tone(buzzerPin, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    // stop the tone playing:
    tone(buzzerPin, 0, noteDuration);
//    noTone(buzzerPin);
  }
}
