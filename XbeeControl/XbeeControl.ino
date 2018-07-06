/*To control the rover, Copy and paste the code below into the Arduino software*/
int E1 = 6; //M1 Speed Control
int E2 = 5; //M2 Speed Control
int M1 = 8; //M1 Direction Control
int M2 = 7; //M2 Direction Control

int rawsensorValueEsq = 0;  // variable to store the v
int rawsensorValueDir = 0;  // variable to store the v
int sensorcountesq0 = 0;
int sensorcountesq1 = 0;
int sensorcountdir0 = 0;
int sensorcountdir1 = 0;
int countesq = 0;
int countdir = 0;
int dist = 9;
long inicioVoltaesq;
long fimVoltaesq;
float periodoesq;
float velocidadeesq;
long inicioVoltadir;
long fimVoltadir;
float periododir;
float velocidadedir;

void setup(void)
{
  int i;
  for(i=5;i<=8;i++)
  pinMode(i, OUTPUT);
  Serial.begin(57600);
  inicioVoltaesq = millis();
  inicioVoltadir = millis();
}

void loop(void)
{
  forward (255,255);
  rawsensorValueEsq = analogRead(0);
  rawsensorValueDir = analogRead(1);
  if (rawsensorValueEsq < 600){  //Min value is 400 an
    sensorcountesq1 = 1;
  }
  else {
    sensorcountesq1 = 0;
  }
  if (sensorcountesq1 != sensorcountesq0){
    countesq ++;
  }
  sensorcountesq0 = sensorcountesq1;
  if (countesq==8){
    countesq = 0; 
    fimVoltaesq = millis();
    periodoesq = (fimVoltaesq - inicioVoltaesq)/1000.;
    velocidadeesq = dist/periodoesq;
    if (velocidadeesq<3){
      velocidadeesq=0;
    }
    //Serial.print("Velocidade esquerda(cm/s)    ");
    //Serial.println(velocidadeesq);
    inicioVoltaesq = fimVoltaesq;
  }

  if (rawsensorValueDir < 600){  //Min value is 400 an
     sensorcountdir1 = 1;
  }
  else {
    sensorcountdir1 = 0;
  }
  if (sensorcountdir1 != sensorcountdir0){
    countdir ++;
  }
  sensorcountdir0 = sensorcountdir1;
  if (countdir==8){
    countdir = 0; 
    fimVoltadir = millis();
    periododir = (fimVoltadir - inicioVoltadir)/1000.;
    velocidadedir = dist/periododir;
    if (velocidadedir<3){
      velocidadedir=0;
    }
    //Serial.print("Velocidade direita(cm/s)    ");
    Serial.println(rawsensorValueEsq);
    inicioVoltadir = fimVoltadir;
  }
}
void stop(void) //Stop
{
  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);  
  
}
void forward(char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
  
}
void reverse (char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void left (char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void right (char a,char b)
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}

