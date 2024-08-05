//Definiciones de pines
#define DirectionPin A15                              // Pin de la direccion del anemómetro
#define SpeedPin A14                                  // Pin de la velocidad del anemómetro

// Variables para la direccion  
int valorPot = 0;                                     // Valor del potenciómetro del velero
int points[] = {0,100,190,210,250,560,630,780,1023};  // 0, 45, 90, 135, 180, 225, 270, 315, 360

//Variables para la velocidad
const int tiempoMedicion = 3;                         //Tiempo de medición (s)
int contadorI = 0;                                    //Contador de Interrupciones  

void setup() {
  Serial.begin(9600);
  pinMode(SpeedPin, INPUT_PULLUP);
}

void loop() {
  float velocidad = getVelocidad();
  int direccion = getDireccion();
  mostrar(velocidad, direccion);
}

int getDireccion() {
  valorPot = analogRead(DirectionPin);
  
  if(valorPot >= points[0] && valorPot <= points[1]){
    return(map(valorPot, points[0], points[1], 0, 45));
  }
  else if(valorPot > points[1] && valorPot <= points[2]){
    return(map(valorPot, points[1], points[2], 45, 90));
  }
  else if(valorPot > points[2] && valorPot <= points[3]){
    return(map(valorPot, points[2], points[3], 90, 135));
  }
  else if(valorPot > points[3] && valorPot <= points[4]){
    return(map(valorPot, points[3], points[4], 135, 180));
  }
  else if(valorPot > points[4] && valorPot <= points[5]){
    return(map(valorPot, points[4], points[5], 180, 225));
  }
  else if(valorPot > points[5] && valorPot <= points[6]){
    return(map(valorPot, points[5], points[5], 225, 270));
  }
  else if(valorPot > points[6] && valorPot <= points[7]){
    return(map(valorPot, points[6], points[7], 270, 315));
  }
  else if(valorPot > points[7] && valorPot <= points[8]){
    return(map(valorPot, points[7], points[8], 315, 350));
  }
}

int getVelocidad() {
  contadorI = 0;
  
  attachInterrupt(digitalPinToInterrupt(SpeedPin), conteo, RISING);
  delay(1000 * tiempoMedicion);
  detachInterrupt(digitalPinToInterrupt(SpeedPin));
  
  return (float)contadorI / (float)tiempoMedicion * 2.4;
}

void conteo() {
  contadorI++;
}

void mostrar(float velocidad, int direccion) {
  Serial.print("Velocidad: ");
  // Imprimir la velocidad en m/s
  Serial.print(velocidad < 10 ? " " : "");
  Serial.print(velocidad, 1); // 1 decimal para m/s
  Serial.print("\t");

  Serial.print("Dirección: ");
  // Imprimir el valor de la dirección con una precisión de 5°
  Serial.print(direccion < 100 ? "0" : "");
  Serial.print(direccion < 10 ? "0" : "");
  Serial.println(direccion, DEC);
}