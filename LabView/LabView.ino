//PesoSDistancia
#include <Wire.h>

// Variables de los sensores
float DataPeso = 0;
float DataDistancia = 0;
char data_UART;
 
void setup() 
{
  // UART Init
	Serial.begin(9600);

  // GPIO Init
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void loop() 
{
  // Lectura
  DataPeso = random(0, 20);
  DataDistancia = random(0, 100);

  // Envio de datos por UART
  if(Serial.available() > 0)
  {
    data_UART = Serial.read();
    if(data_UART == 'E')
    {
      Serial.print(DataPeso);
      Serial.print("s");
      Serial.print(DataDistancia);
      delay(50);
    }
  }
}