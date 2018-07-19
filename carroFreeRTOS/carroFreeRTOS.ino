#include <Arduino_FreeRTOS.h>
//#include <semphr.h>
//#include <task.h>

/* TODO
 * ADICIONAR PID AS RODAS
 * ADICIONAR CONTROLE DE POSICAO 
 * ENVIAR MSGS PELO XBEE
 * ADICIONAR CONTROLE DE TIME
 */

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

oid checkVel( void *pvParameters){
  float lastT1 = 0, T1 = 0, T2 = 0;
  boolean highFlag, lowFlag;
  if (analogRead(0) < 650){
    highFlag = false;
    lowFlag = true;
  }
  else{
    highFlag = true;
    lowFlag = false;
  }
  char *pcTaskName;
  int rawSensorValueEsq;
  char dist = 11;
  boolean sensorTrig = false;
  boolean sensorFlag = false;
  float periodoEsq;
  pcTaskName = (char *) pvParameters;
  for(;;){
    if((millis()-lastT1)>5000){
      velocidadeEsq = 0;
    }
    rawSensorValueEsq = analogRead(0);
    if (rawSensorValueEsq < 650 && highFlag && millis()-T2 > 30){  //Min value is 400 an
      T1 = millis();
      velocidadeEsq = (dist/8)/((T1 - lastT1)/1000);
      lastT1 = T1;
      highFlag = false;
      lowFlag = true;
    }
    if(rawSensorValueEsq > 650 && lowFlag && millis()-T1 > 30){
      highFlag = true;
      lowFlag = false;
      T2 = millis();
    }
    vTaskDelay(50/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}

void vApplicationIdleHook(void)
{
  ulIdleCycleCount++;
}

void setup() {

  Serial.begin(57600);
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
