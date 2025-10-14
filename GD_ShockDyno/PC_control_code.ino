int outputPin = 11;
int dutyCycle;

void setup() 
{
  pinMode(outputPin,OUTPUT);
  Serial.begin(9600); //Baud Rate
  Serial.println("PWM Control");  
}

void loop() 
{
    if (Serial.available() > 0) 
    {
    char dutyCycle = Serial.read();

    Serial.print("You send: ");
    Serial.print(dutyCycle);
    Serial.println();

    switch(dutyCycle) {
      case '0':
        analogWrite(outputPin, 0);
        break;
      case '1':
        analogWrite(outputPin, 25);       
        break;
      case '2':
        analogWrite(outputPin, 50);       
        break;
      case '3':
        analogWrite(outputPin, 75);       
        break;
      case '4':
        analogWrite(outputPin, 100);       
        break;
      case '5':
        analogWrite(outputPin, 125);       
        break;
      case '6':
        analogWrite(outputPin, 150);       
        break;
      case '7':
        analogWrite(outputPin, 175);       
        break;
      case '8':
        analogWrite(outputPin, 200);       
        break;
      case '9':
        analogWrite(outputPin, 250);       
        break;
    }
  }
}
