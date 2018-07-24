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

int Kp = 1, Ki = 1;


QueueHandle_t xVelocidadeEsq;
QueueHandle_t xVelocidadeDir;
QueueHandle_t xSerialVelEsq;
QueueHandle_t xSerialVelDir;

QueueHandle_t xGen;


void checkVel( void *pvParameters){
  int motor = (int)pvParameters;
  portBASE_TYPE xStatus;
  float velocidade = 0;
  
  float lastT1 = 0, T1 = 0, T2 = 0;
  boolean highFlag, lowFlag;


  for (;;) // A Task shall never return or exit.
  {
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
      if((millis()-lastT1)>5000){
        velocidade = 0;
      }
      rawSensorValue = analogRead(motor);
      if (rawSensorValue < 650 && highFlag && millis()-T2 > 30){  //Min value is 400 an
        T1 = millis();
        velocidade = (dist/8)/((T1 - lastT1)/1000);
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
  
      if (motor==0){
        xStatus = xQueueSendToBack( xVelocidadeEsq, &velocidade, 5 );
        xStatus = xQueueSendToBack( xSerialVelEsq, &velocidade, 5 );
      }
      else {
        xStatus = xQueueSendToBack( xVelocidadeDir, &velocidade, 5 );  
        xStatus = xQueueSendToBack( xSerialVelDir, &velocidade, 5 );
      }
      if( xStatus != pdPASS )
      {
        /* The send operation could not complete because the queue was full -
        this must be an error as the queue should never contain more than
        one item! */
        //Serial.println( "Could not send to the queue.\r\n" );
        //Serial.println('0');
      }
      vTaskDelay(50/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
    }
  }
}


//static void vReceiverTask( void *pvParameters )
//{
//  /* Declare the variable that will hold the values received from the queue. */
//  float lReceivedValue;
//  portBASE_TYPE xStatus;
//  /* This task is also defined within an infinite loop. */
//  for( ;; )
//  {
//    /* This call should always find the queue empty because this task will
//    immediately remove any data that is written to the queue. */
//    if( uxQueueMessagesWaiting( xQueue ) == 0 )
//    {
//      //Serial.println( "Queue vazia" );
//    }
//    /* Receive data from the queue.
//    The first parameter is the queue from which data is to be received. The
//    queue is created before the scheduler is started, and therefore before this
//    task runs for the first time.
//    The second parameter is the buffer into which the received data will be
//    placed. In this case the buffer is simply the address of a variable that
//    has the required size to hold the received data.
//    The last parameter is the block time – the maximum amount of time that the
//    task should remain in the Blocked state to wait for data to be available
//    should the queue already be empty. In this case the constant
//    portTICK_RATE_MS is used to convert 100 milliseconds to a time specified in
//    ticks. */
//    xStatus = xQueueReceive( xQueue, &lReceivedValue, 100/portTICK_PERIOD_MS);
//    if( xStatus == pdPASS )
//    {
//      /* Data was successfully received from the queue, print out the received
//      value. */
//      //Serial.print("Received = ");
//      //Serial.println(lReceivedValue);
//    }
//    else
//    {
//      /* Data was not received from the queue even after waiting for 100ms.
//      This must be an error as the sending tasks are free running and will be
//      continuously writing to the queue. */
//      //Serial.println( "Could not receive from the queue.\r\n" );
//    }
//    Serial.println('0');
//    vTaskDelay(1000/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
//  }
//}


//Calculates the PI parameter
void calcPID( void *pvParameters){
  float velDir;
  float velEsq;
  int IEsq = 0;
  int IDir = 0;
  int SP1 = 5;
  int SP2 = 5;
  
  for (;;) // A Task shall never return or exit.
  {
  portBASE_TYPE xStatus;
///  xStatus = xQueueReceive(xGen, &SP1, 10/portTICK_PERIOD_MS);
 // xStatus = xQueueReceive(xGen, &SP2, 10/portTICK_PERIOD_MS);
//  xStatus = xQueueReceive(xVelocidadeEsq, &velEsq, 10/portTICK_PERIOD_MS);
  xStatus = xQueueReceive(xGen, &IEsq, 10/portTICK_PERIOD_MS);
//  xStatus = xQueueReceive(xVelocidadeDir, &velDir, 10/portTICK_PERIOD_MS);
    if( xStatus == pdPASS )
      Serial.println("Não Leu");
  int E = SP1 - velEsq;
  int P = Kp*E;
  
    Serial.print(" | IEsq1 ");
    Serial.print(IEsq);
    IEsq = IEsq + (0.5*E)*Ki;
    Serial.print(" | IEsq2 ");
    Serial.print(IEsq);
    int pwm = P + IEsq;
    analogWrite (E1, pwm);
    Serial.print(" | pwmEsq ");
    Serial.print(pwm);
    E = SP2 - velDir;
    P = Kp*E;
    /*
    Serial.print(" | IDir1 ");
    Serial.print(IDir);
    */
    IDir = IDir + (0.5*E)*Ki;
    Serial.print(" | IDir2 ");
    Serial.print(IDir);
    pwm = P + IDir;
    analogWrite (E2, pwm);
    Serial.print(" | pwmDir ");
    Serial.print(pwm);
    Serial.print(" | SPE ");
    Serial.print(SP1);
    Serial.print(" | SPD ");
    Serial.print(SP2);
    Serial.print(" | IDir ");
    Serial.print(IDir);
    Serial.print(" | IEsq ");
    Serial.println(IEsq);
  //  xStatus = xQueueSendToBack( xGen, &SP1, 0);
  //  xStatus = xQueueSendToBack( xGen, &SP2, 0);
    xStatus = xQueueSendToBack( xGen, &IEsq, 0);
  //  xStatus = xQueueSendToBack( xGen, &IDir, 0);
    if( xStatus != pdPASS )
      Serial.println("Deu Merda");
    xStatus = xQueueSendToBack( xGen, &IDir, 0);
    if( xStatus != pdPASS )
      Serial.println("fila chea");
    vTaskDelay(500/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}


void vApplicationIdleHook(void)
{
  ulIdleCycleCount++;
}

void setup() {
  
  Serial.begin(57600);
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
    xTaskCreate(checkVel, "Velocímetro Esquerda", 128, (void *) 0, 1, NULL); 
    xTaskCreate(checkVel, "Velocímetro Direita", 128, (void *) 1, 1, NULL); 
    /* Create the task that will read from the queue. The task is created with
    priority 2, so above the priority of the sender tasks. */
//    xTaskCreate(vReceiverTask, "Receiver", 128, NULL, 2, NULL );
 
    xTaskCreate(calcPID, "PID", 128, NULL, 2, NULL); 
    
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
