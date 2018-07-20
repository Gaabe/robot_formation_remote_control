#include <Arduino_FreeRTOS.h>
#include <semphr.h>
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

QueueHandle_t xQueue;


void checkVel( void *pvParameters){
  portBASE_TYPE xStatus;
  float velocidadeEsq = 0;
  
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
    Serial.print("Valor real:");
    Serial.println(velocidadeEsq);

    
    xStatus = xQueueSendToBack( xQueue, &velocidadeEsq, 5 );
    if( xStatus != pdPASS )
    {
      /* The send operation could not complete because the queue was full -
      this must be an error as the queue should never contain more than
      one item! */
      Serial.println( "Could not send to the queue.\r\n" );
    }
    vTaskDelay(2000/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}



static void vReceiverTask( void *pvParameters )
{
  /* Declare the variable that will hold the values received from the queue. */
  float lReceivedValue;
  portBASE_TYPE xStatus;
  /* This task is also defined within an infinite loop. */
  for( ;; )
  {
    /* This call should always find the queue empty because this task will
    immediately remove any data that is written to the queue. */
    if( uxQueueMessagesWaiting( xQueue ) == 0 )
    {
      Serial.println( "Queue vazia" );
    }
    /* Receive data from the queue.
    The first parameter is the queue from which data is to be received. The
    queue is created before the scheduler is started, and therefore before this
    task runs for the first time.
    The second parameter is the buffer into which the received data will be
    placed. In this case the buffer is simply the address of a variable that
    has the required size to hold the received data.
    The last parameter is the block time – the maximum amount of time that the
    task should remain in the Blocked state to wait for data to be available
    should the queue already be empty. In this case the constant
    portTICK_RATE_MS is used to convert 100 milliseconds to a time specified in
    ticks. */
    xStatus = xQueueReceive( xQueue, &lReceivedValue, 100/portTICK_PERIOD_MS);
    if( xStatus == pdPASS )
    {
      /* Data was successfully received from the queue, print out the received
      value. */
      Serial.print("Received = ");
      Serial.println(lReceivedValue);
    }
    else
    {
      /* Data was not received from the queue even after waiting for 100ms.
      This must be an error as the sending tasks are free running and will be
      continuously writing to the queue. */
      Serial.println( "Could not receive from the queue.\r\n" );
    }
    vTaskDelay(1000/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
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
  
  xQueue = xQueueCreate( 5, 2*sizeof(float) );

  if( xQueue != NULL )
  {
    /* Create two instances of the task that will send to the queue. The task
    parameter is used to pass the value that the task will write to the queue,
    so one task will continuously write 100 to the queue while the other task
    will continuously write 200 to the queue. Both tasks are created at
    priority 1. */
    xTaskCreate(checkVel, "Velocímetro", 128, NULL, 1, NULL); 
    /* Create the task that will read from the queue. The task is created with
    priority 2, so above the priority of the sender tasks. */
    xTaskCreate(vReceiverTask, "Receiver", 128, NULL, 2, NULL );
    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();
  }
  else
  {}
  // put your setup code here, to run once:

  vTaskStartScheduler();

  for(;;);

  
}

void loop() {
  // put your main code here, to run repeatedly:

}
