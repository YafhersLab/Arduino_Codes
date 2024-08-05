#include <HardwareSerial.h>
#include <String.h>

HardwareSerial SerialPort(2);

const uint8_t rx2Pin = 16;
const uint8_t tx2Pin = 17;

char data[9] = {0};
int cont = 1000;

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, rx2Pin, tx2Pin);
}

void loop() {
  cont++;
  sprintf(data, "%d", cont);
  SerialPort.write(data);
  delay(1000);
}
