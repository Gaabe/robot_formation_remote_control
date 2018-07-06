#include <Arduino_FreeRTOS.h>
//#include <semphr.h>
//#include <task.h>

unsigned long ulIdleCycleCount = 0UL;

// declarações de pinos
int E1 = 6; //M1 Speed Control
int E2 = 5; //M2 Speed Control
int M1 = 8; //M1 Direction Control
int M2 = 7; //M2 Direction Control

long fimVoltaEsq;
long inicioVoltaEsq;
int countEsq;
long ultimaVoltaCompletada;

void checkVel( void *pvParameters)
{
  char *pcTaskName;
  pcTaskName = (char *) pvParameters;
  for(;;){
    char dist = 9;
    int sensorCountEsq0;
    int sensorCountEsq1;
    float velocidadeEsq;
    float periodoEsq;
    if((millis()-ultimaVoltaCompletada)>2000){
      velocidadeEsq = 0;
    }    
    int rawSensorValueEsq = analogRead(1);
    if (rawSensorValueEsq < 650){  //Min value is 400 an
      sensorCountEsq1 = 1;
      //Serial.print("Menor que 650  ");
      //Serial.println(sensorCountEsq1);
  }
    else {
      sensorCountEsq1 = 0;
      //Serial.print("Maior que 650   ");
      //Serial.println(sensorCountEsq1);
  }
  if (sensorCountEsq1 != sensorCountEsq0){
    countEsq ++;
  }
  sensorCountEsq0 = sensorCountEsq1;
  if (countEsq==8){
    countEsq = 0; 
    fimVoltaEsq = millis();
    
    float periodoEsq = (fimVoltaEsq - inicioVoltaEsq)/1000.;
    //Serial.print("Periodo: ");
    //Serial.println(periodoEsq);
    inicioVoltaEsq = fimVoltaEsq;
    velocidadeEsq = dist/periodoEsq;
    ultimaVoltaCompletada = millis();
  }
    //Serial.print("Diferenca de voltas: ");
    //Serial.println(millis()-ultimaVoltaCompletada);
    //Serial.print("Velocidade esquerda(cm/s)    ");
    Serial.println(velocidadeEsq);
  vTaskDelay(5/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}

void vApplicationIdleHook(void)
{
  ulIdleCycleCount++;
}

void setup() {

  Serial.begin(115200);
  analogWrite (E1,255);
  digitalWrite(M1,LOW);
  analogWrite (E2, 255);
  digitalWrite(M2,LOW);
  inicioVoltaEsq = millis();
  countEsq = 0;
  Serial.print("Teste Serial");
  
  
  // put your setup code here, to run once:
  xTaskCreate(checkVel, "Velocímetro", 128, NULL, 1, NULL); 
  vTaskStartScheduler();

  for(;;);

  
}

void loop() {
  // put your main code here, to run repeatedly:

}
