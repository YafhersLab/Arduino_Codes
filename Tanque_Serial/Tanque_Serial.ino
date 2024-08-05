const int analogPin = A0;
int sensorValue = 0;
int outputValue = 0;

int dato = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(analogPin);
  Serial.println(sensorValue);
  dato = sensorValue;

  delay(200);
}
