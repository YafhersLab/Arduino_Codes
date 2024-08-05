#include <SoftwareSerial.h>
int ledPin = 4;
int receivedChar;
SoftwareSerial HC05(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  HC05.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  
  if (HC05.available() != 0) {
     receivedChar = HC05.read();
     Serial.println(receivedChar);

    if (receivedChar == 'A') {
      digitalWrite(ledPin, HIGH);
    }
    else if (receivedChar == 'B') {
      digitalWrite(ledPin, LOW);
    }
    else {
      // Ignorar 
    }
    }

}

