/*
HackerspaceActivityMonitor https://www.hackerspace-ffm.de/wiki/index.php?title=HackffmActivitySensors

ATmega328 (Arduino UNO), see http://arduino.cc/it/Hacking/PinMapping168 with:
 PIR on 2 (INT0)
 BUTTON on 3 (INT1)
 Microphone-Modul on A0
 WizNET W5100 Modul on SPI (11,12,13) and CS on 10 and /RESET on A3
 LED as light sensor between Anode=7 Cathode=8 
 DHT11 as temp/humidity sensor on 4
 Status LEDs red on 6, green on 5

Send to MQTT broker
 

Circuit:
* Ethernet shield attached to pins 10, 11, 12, 13



*/
#include <avr/wdt.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <Ethernet.h>
#include <SPI.h>
#include "DHT.h"



byte mac[] = { 0x90, 0xA2, 0xDA, 0x04, 0x00, 0x2B}; // make sure this is unique on your network

const char* mqtt_server = "hackffmrpi";
// IPAddress mqtt_server(172, 16, 0, 2);

#define DHTPIN 4     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
DHT dht(DHTPIN, DHTTYPE);

// Led, button and PIR port here (PIR and BUTTON must be interrupt pin!)
const int ledGreen = 5;
const int ledRed = 6;
const int PIR = 2;
const int BUTTON = 3;

int errorcount = 0;

volatile int intPIRCounter = 0;
volatile int intBUTTONCounter = 0;

int PIRCounter = 0;
int BUTTONCounter = 0;

EthernetClient ethClient;
PubSubClient client(ethClient);
uint16_t lastReconnectAttempt = 0;
char tempBuffer[100];

// Counters are incremented by hardware interrupt
void PIRCount() { intPIRCounter++; }
void BUTTONCount() { intBUTTONCounter++; }


// test if one minute is over and store and reset counter
void CountMinuteReset() {
  static unsigned long timeLast = 0;
  if(millis() > (timeLast + 60000)) {
    timeLast += 60000;
    noInterrupts();
    PIRCounter = intPIRCounter;
    BUTTONCounter = intBUTTONCounter;
    intPIRCounter = 0;
    intBUTTONCounter = 0;
    
    interrupts();
    Serial.println("cmr");
  }
}
  
double getlight() {
  const int ledPinA = 7;
  const int ledPinK = 8;
  unsigned long timeStart, timeEnd;  
  pinMode(ledPinA, OUTPUT);
  pinMode(ledPinK, OUTPUT);
  digitalWrite(ledPinA, LOW);
  digitalWrite(ledPinK, HIGH);
  delay(1);
  pinMode(ledPinK, INPUT);
  
  timeStart = micros();
  digitalWrite(ledPinK, LOW);
  

  while(digitalRead(ledPinK) == HIGH) {
    if((micros() - timeStart) > 10000000) {
      break;
    }
  }
  
  timeEnd = micros();
  
  return(log10(1000000.0/(double)(timeEnd - timeStart)));
}

double getnoise() {
    unsigned long  l = 0;
    for(int i=0;i<10000;i++) { 
      l += analogRead(0);
    }
    return(log10(((double)l/10000.0)+1.0));
}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

boolean reconnect() {
  if (client.connect("ActivitySensors")) {
    // Once connected, publish an announcement...
    client.publish("reconnect","yet");
    // ... and resubscribe
   // client.subscribe("inTopic");
  }
  return client.connected();
}

void setup() {
 //  wdt_enable(WDTO_8S);
  wdt_reset();
  Serial.begin(9600);
  Serial.print("Starting net... "); 
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(PIR, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  
  // Reset ETH (for modified module with extra reset input connected to arduino pin A3)
  pinMode(A2, OUTPUT); 
  digitalWrite(A2, LOW);
  delay(100);
  digitalWrite(A2, HIGH);      
  Ethernet.begin(mac);
  delay(1500);
  lastReconnectAttempt = 0;
  wdt_reset();
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print("."); 
  }
  Serial.println();
    
  attachInterrupt(0,PIRCount,RISING);
  attachInterrupt(1,BUTTONCount,FALLING);
  dht.begin();

}

void loop() {
  int status = 200;
  float f;
  static int butstate;
  static float last_light = 0.0;
  digitalWrite(ledGreen, HIGH);
  //Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++");
  CountMinuteReset();

  Serial.print("sync state code <OK == 0> => ");
  Serial.println(client.state());  
  wdt_reset();
    
  if (!client.connected()) {
    digitalWrite(ledRed, HIGH);  
    uint16_t now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
      errorcount++;
      if(errorcount > 10) {
        cli();
        asm volatile ("jmp 0x0000"); 
      }
    }
  } else {
    digitalWrite(ledRed, LOW);
    errorcount=0;
    // Client connected

    client.loop();

    
    // Read from DHT, update only if a number is returned
    f = dht.readTemperature();
    if(!isnan(f)) client.publish("ActivitySensors/temperature", dtostrf(f, 1, 2, tempBuffer));
    f = dht.readHumidity();
    if(!isnan(f)) client.publish("ActivitySensors/humidity", dtostrf(f, 1, 2, tempBuffer));

    client.publish("ActivitySensors/noise", dtostrf((float)getnoise(), 1, 7, tempBuffer));
    client.publish("ActivitySensors/pir_count", itoa((int)PIRCounter, tempBuffer, 10));
    client.publish("ActivitySensors/button_count", itoa((int)BUTTONCounter, tempBuffer, 10));
    butstate = digitalRead(BUTTON);
    tempBuffer[1] = 0;
    tempBuffer[0] = butstate?'0':'1';
    client.publish("ActivitySensors/button_state", tempBuffer);
    
    client.publish("ActivitySensors/light", dtostrf((float)last_light, 1, 7, tempBuffer));

  } 
  
  digitalWrite(ledGreen, LOW);
  for(int i=0; i<10; i++) {
    if(butstate != digitalRead(BUTTON)) {
      butstate = digitalRead(BUTTON);
      tempBuffer[1] = 0;
      tempBuffer[0] = butstate?'0':'1';
      client.publish("ActivitySensors/button_state", tempBuffer);
    }
    delay(500);
    client.loop();
    wdt_reset();
  } 
  last_light = (float)getlight();
}


