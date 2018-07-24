#include <Arduino_FreeRTOS.h>
#include <semphr.h>
//#include <task.h>
#define direita 0
#define esquerda 1

/* TODO
 * ADICIONAR PID AS RODAS
 * ADICIONAR CONTROLE DE POSICAO 
 * ENVIAR MSGS PELO XBEE
 * ADICIONAR CONTROLE DE TIME
 */

unsigned long ulIdleCycleCount = 0UL;

// declarações de pinos
int setVelEsq = 6; //M1 Speed Control
int setVelDir = 5; //M0 Speed Control
int setSentidoEsq = 8; //M1 Direction Control
int setSentidoDir = 7; //M0 Direction Control

int Kp = 1, Ki = 5;

SemaphoreHandle_t xSerialSemaphore;

QueueHandle_t xVelocidadeEsq;
QueueHandle_t xVelocidadeDir;
QueueHandle_t xSerialVelEsq;
QueueHandle_t xSerialVelDir;

QueueHandle_t xGen;


void checkVel( void *pvParameters){
  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        { 
          Serial.println("check vel chamada");
          xSemaphoreGive(xSerialSemaphore);
        }
  int motor = (int)pvParameters;
  portBASE_TYPE xStatus;
  float velocidade = 0;
  
  float lastT1 = 0, T1 = 0, T2 = 0;
  boolean highFlag, lowFlag;
    if (analogRead(motor) < 650){
      highFlag = false;
      lowFlag = true;
    }
    else{
      highFlag = true;
      lowFlag = false;
    }
    char *pcTaskName;
    int rawSensorValue;
    char dist = 11;
    boolean sensorTrig = false;
    boolean sensorFlag = false;
    float periodo;
    pcTaskName = (char *) pvParameters;
    for(;;){
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        { 
          //Serial.println("estamos na check vel");
          xSemaphoreGive(xSerialSemaphore);
        }
      if((millis()-lastT1)>5000){
        velocidade = 0;
      }
      rawSensorValue = analogRead(motor);

      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        { 
          Serial.println("antes do outro if");
          Serial.println(rawSensorValue);
          xSemaphoreGive(xSerialSemaphore);
        }
      
      if (rawSensorValue < 650 && highFlag && millis()-T2 > 30){  //Min value is 400 an
        T1 = millis();
        velocidade = 10*(dist/8)/((T1 - lastT1)/1000);
        
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        { 
          if(motor == 1)
            //  Serial.print("velE: ");
          if(motor == 0)
            //  Serial.print("velD: ");

          //Serial.println(velocidade);
          xSemaphoreGive(xSerialSemaphore);
        }
        lastT1 = T1;
        highFlag = false;
        lowFlag = true;
      }
      if(rawSensorValue > 650 && lowFlag && millis()-T1 > 30){
        highFlag = true;
        lowFlag = false;
        T2 = millis();
      }
      //Serial.print("Valor real:");
      //Serial.println(velocidade);
  
      if (motor==esquerda){
        xStatus = xQueueSendToBack( xVelocidadeEsq, &velocidade, 0);
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        { 
          //Serial.println("Enviando velocidade da roda esquerda para queue");
          //Serial.println(velocidade);
          xSemaphoreGive(xSerialSemaphore);
        }
      }
      else {
        xStatus = xQueueSendToBack( xVelocidadeDir, &velocidade, 0);  
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        { 
          //Serial.print("Enviando velocidade da roda direita para queue");
          xSemaphoreGive(xSerialSemaphore);
        }
      }
      if( xStatus != pdPASS )
      {
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        { 
          //Serial.println("Não conseguiu mandar a velocidade para a fila");
          xSemaphoreGive(xSerialSemaphore);
        }
      }
      vTaskDelay(50/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
    }
  
}



//Calculates the PI parameter
void calcPID( void *pvParameters){
  float velDir;
  float velEsq;
  int IEsq = 0;
  int IDir = 0;
  int SPDir =50;
  int SPEsq = 50;
  
  for (;;) // A Task shall never return or exit.
  {
  portBASE_TYPE xStatus;
///  xStatus = xQueueReceive(xGen, &SP1, 10/portTICK_PERIOD_MS);
 // xStatus = xQueueReceive(xGen, &SP2, 10/portTICK_PERIOD_MS);
  xStatus = xQueueReceive(xVelocidadeEsq, &velEsq, 10/portTICK_PERIOD_MS);
  xStatus = xQueueReceive(xGen, &IEsq, 10/portTICK_PERIOD_MS);
  xStatus = xQueueReceive(xGen, &IDir, 10/portTICK_PERIOD_MS);
  xStatus = xQueueReceive(xVelocidadeDir, &velDir, 10/portTICK_PERIOD_MS);
    //if( xStatus == pdPASS )
      //Serial.println("Não Leu");
  int E = SPEsq - velEsq;
  int P = Kp*E;
  IEsq = IEsq + (0.5*E)*Ki;
  int pwm = P + IEsq;
  //analogWrite (E1, pwm);
  //E = SP2 - velDir;
  pwm = P + IEsq;
  if (pwm>255){
    pwm = 255;
  }
  if (pwm<0){
    pwm = 0;
  }
  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  { 
//    Serial.print(" | pwmEsq ");
//    Serial.print(pwm);
//    Serial.print(" | iEsq ");
//    Serial.print(IEsq);
//    Serial.print(" | VelEsq");
//    Serial.print(velEsq);
//    Serial.print(" | SPEsq");
//    Serial.print(SPEsq);
//    Serial.print(" | EEsq ");
//    Serial.println(E);
    xSemaphoreGive(xSerialSemaphore);
  }
  analogWrite (setVelEsq, pwm);

        //Serial.println("Não Leu");
  E = SPDir - velDir;
  P = Kp*E;
  IDir = IDir + (0.5*E)*Ki;
  pwm = P + IEsq;
  //analogWrite (E1, pwm);
  //E = SP2 - velDir;
  pwm = P + IDir;
  if (pwm>255){ 
    pwm = 255;
  }
  if (pwm<0){
    pwm = 0;
  }
  if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  { 
//    Serial.print(" | pwmDir ");
//    Serial.print(pwm);
//    Serial.print(" | iDir ");
//    Serial.print(IDir);
//    Serial.print(" | VelDir");
//    Serial.print(velDir);
//    Serial.print(" | SPDir");
//    Serial.print(SPDir);
//    Serial.print(" | EDir ");
//    Serial.println(E);
    xSemaphoreGive(xSerialSemaphore);
  }
  analogWrite (setVelDir, pwm);




  
  //Serial.print(" | pwmDir ");
  //Serial.print(pwm);
  //Serial.print(" | SPE ");
  //Serial.print(SP1);
  //Serial.print(" | SPD ");
  //Serial.print(SP2);
  //Serial.print(" | IDir ");
  //Serial.print(IDir);
  //Serial.print(" | IEs");
  //Serial.println(IEsq);
//  xStatus = xQueueSendToBack( xGen, &SP1, 0);
//  xStatus = xQueueSendToBack( xGen, &SP2, 0);
  //Serial.println("Enviando valor de IEsq para xGen");
  xStatus = xQueueSendToBack( xGen, &IEsq, 0);
  //if( xStatus != pdPASS )
    //Serial.println("Deu Merda, fila cheia");
  //else
    //Serial.println("qwertyuiopa");
  xStatus = xQueueSendToBack( xGen, &IDir, 0);

    //taskYIELD();
  vTaskDelay(500/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}


void vApplicationIdleHook(void)
{
  ulIdleCycleCount++;
}

void setup() {
  
  Serial.begin(57600);
  while (!Serial){}

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  
  //Serial.println("Inicializando código");
//  analogWrite (E1,255);
//  digitalWrite(M1,LOW);
//  analogWrite (E2, 255);
//  digitalWrite(M2,LOW);
  //Serial.print("Teste Serial");
  
  xVelocidadeEsq = xQueueCreate( 5, 2*sizeof(float) );
  xVelocidadeDir = xQueueCreate( 5, 2*sizeof(float) );
  xSerialVelEsq = xQueueCreate( 5, 2*sizeof(float) );
  xSerialVelDir = xQueueCreate( 5, 2*sizeof(float) );

  xGen = xQueueCreate( 4, sizeof(int) );

  portBASE_TYPE xStatus;

//  int IEsq = 1;
//  int IDir = 1;
//  int SP1 = 5;
//  int SP2 = 5;
//
//  xStatus = xQueueSendToBack( xGen, &SP1, 0 );
//  xStatus = xQueueSendToBack( xGen, &SP2, 0 );
//  xStatus = xQueueSendToBack( xGen, &IEsq, 0 );
//  xStatus = xQueueSendToBack( xGen, &IDir, 0 );
//  xStatus = xQueueSendToBack( xGen, &IDir, 0);
//  if( xStatus != pdPASS )
//    Serial.println("HU");

//  if( xQueue != NULL )
//  {
    /* Create two instances of the task that will send to the queue. The task
    parameter is used to pass the value that the task will write to the queue,
    so one task will continuously write 100 to the queue while the other task
    will continuously write 200 to the queue. Both tasks are created at
    priority 1. */
    //xTaskCreate(checkVel, "Velocímetro Esquerda", 128, (void *) esquerda, 2, NULL); 
    xTaskCreate(checkVel, "Velocímetro Direita", 128, (void *) direita, 2, NULL); 
    /* Create the task that will read from the queue. The task is created with
    priority 2, so above the priority of the sender tasks. */
//    xTaskCreate(vReceiverTask, "Receiver", 128, NULL, 1, NULL );
 
    xTaskCreate(calcPID, "PID", 128, NULL, 1, NULL); 
    
    /* Start the scheduler so the created tasks start executing. */

//  }
//  else
//  {}
  // put your setup code here, to run once:
    vTaskStartScheduler();

  for(;;);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
