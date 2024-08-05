//fclk = 16Mhz -> T = 62.5ns
//debemos contar 5000000 / 62.5 = 80000
//80000 es muy grande ya que el timer1 de 16 bits puede contar hasta 65535
//aplicamos un prescaler de 256
//fclk = 16Mhz / 256 = 62.5KHz -> T = 16us
//entonces contemos 5000000 / 16 = 31250
//timer 1 debe contar hasta 31250 para contar 500ms

void setup() {
  nointerrupts();
  pinMode(13, OUTPUT);
  TCCR1A = 0;                                       // limpiamos el registro de control A del timer 1
  TCCR1B = 0;                                       // limpiamos el registro de control B del timer 1
  TCCR1B |= B00000100;                              // prescaler a 256
  TIMSK1 |= B00000010;                              // habilito la interrupcion por comparacion
  OCR1A = 31250;                                    // valor calculado para una interrupcion cada 500ms
  interrupts();
}

void loop() {

}

ISR(TIMER1_COMPA_vect){
  TCNT1 = 0;                                        //timer 1 a 0
  digitalWrite(13, !(digitalRead(13)));
}