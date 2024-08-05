const int potPin = 34; // Pin GPIO34 (ADC1_CH6) para el potenciómetro

void setup() {
  Serial.begin(9600); // Inicializar la comunicación serial
  analogReadResolution(12); // Establecer resolución de lectura analógica a 12 bits
}

void loop() {
  int potValue = analogRead(potPin); // Leer el valor del potenciómetro
  Serial.println(potValue); // Imprimir el valor leído en el puerto serie
  delay(1000); // Esperar 100 milisegundos antes de la próxima lectura
}