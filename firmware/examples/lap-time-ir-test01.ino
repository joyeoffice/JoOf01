#include "JoOf01.h"
#include "JoOf01Int.h"

unsigned long currentTime;
unsigned long lastTime = 0UL;

int ledPin = D0;                // LED connected to digital pin D1
int analogPin = A0;             // potentiometer connected to analog pin A0
int digitalPin = D6;
int val = 0;                    // variable to store the read value

int RECV_PIN = D2;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup() {
  // Note: analogPin pin does not require pinMode()

    pinMode(ledPin, OUTPUT);      // sets the ledPin as output
    pinMode(digitalPin, INPUT);
       // sets the ledPin as output
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    Serial.begin(9600);
    irrecv.enableIRIn(); // Start the receiver
}

void loop() {

    if (digitalRead(digitalPin) == HIGH) {
    currentTime = millis();
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
    }
    if (irrecv.decode(&results)) {
      Serial.println(results.value, HEX);
      irrecv.resume(); // Receive the next value
    }
    delay(100);
}
