String inputString = "";
bool stringComplete = false;
int internalLed = 2;

void setup() 
{
  Serial.begin(9600);
  pinMode(internalLed, OUTPUT);
  digitalWrite(internalLed, LOW);
}

void loop() 
{
  if(stringComplete)
  {
    inputString.trim();
    Serial.println(inputString);

    if(inputString.equals("$LF"))
    {
      digitalWrite(internalLed, LOW);
    }
    else if(inputString.equals("$LT"))
    {
      digitalWrite(internalLed, HIGH);
    }

    inputString = "";
    stringComplete = false;
  }

}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char) Serial.read();
    if (inChar == '\n')
    {
      stringComplete = true;
    }
    else
    {
      inputString += inChar;
    }
  }
}