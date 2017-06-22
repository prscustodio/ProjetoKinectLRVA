//#include <digitalWriteFast.h>

int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
char DadoSerial;

void direita(int x)
{
  analogWrite(IN1, x);
  analogWrite(IN2, 0);
  analogWrite(IN3, x);
  analogWrite(IN4, 0);
}

void esquerda (int x)
{
  analogWrite(IN1, 0);
  analogWrite(IN2, x);
  analogWrite(IN3, 0);
  analogWrite(IN4, x);
}

void frente (int x)
{
  analogWrite(IN1, x);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, x);
}
void re (int x)
{
  analogWrite(IN1, 0);
  analogWrite(IN2, x);
  analogWrite(IN3, x);
  analogWrite(IN4, 0);
}
void parar ()
{
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void setup() 
{
  // put your setup code here, to run once:
  Serial1.begin(57600);
}

void loop()
{
    
  int x= PI/2;
  int t=(x/PI)*1000;
  direita(255);
  delay(t);
  parar();
  delay(3000);                                          
  
}
