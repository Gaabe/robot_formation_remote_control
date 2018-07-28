/******************************************************
 ****************** Robo Bombeiro *********************
   Programação de tempo real para sistemas embarcados
 ******************************************************/

#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#define ENC_DIR 0
#define ENC_ESQ 1

#define DISTANCIA_ENTRE_EIXOS 100

// definições de pinos
#define setVelEsq 6 //M1 Speed Control
#define setVelDir 5 //M0 Speed Control
#define setSentidoEsq 8 //M1 Direction Control
#define setSentidoDir 7 //M0 Direction Control
#define dist 11 //Perimetro da roda

#define Kp 3
#define Ki 1

unsigned long ulIdleCycleCount = 0UL;

//Struct utilizada para guardar Velocidades
typedef struct {
  int esq[5] = {0, 0, 0, 0, 0};
  int dir[5] = {0, 0, 0, 0, 0};
  int i = 0;
} Vel;

//Struct utilizada para guardar OS  Set Points
typedef struct {
  int esq = 50;
  int dir = 50;
} SetPoint;

//Struct utilizada para guardar Posições de Referência
typedef struct {
  int X = 0;
  int Y = 0;
  int THETA = 0;
} Position;

//Semafor para a serial
SemaphoreHandle_t xSerialSemaphore;

//Semaforo para a Queue da Posição atual
SemaphoreHandle_t xAPSemaphore;

//Semaforo para a Queue da Posição de Referencia
SemaphoreHandle_t xPREFSemaphore;

//Semaforo para a Queue da Velocidade
SemaphoreHandle_t xVELSemaphore;

//Semaforo para a Queue do SetPoint atual de Velocidade
SemaphoreHandle_t xSPSemaphore;

//Declaração global das filas
QueueHandle_t xVelocidade; //Fila que guarda a velocidade das rodas
QueueHandle_t xSP; //Fila usada para armazenar os valores do SetPoint
QueueHandle_t xReferencePosition; //Fila usada para armazenar as posições para as quais o robo deve se mover.
QueueHandle_t xActualPosition; //Fila usada para armazenar as posições para as quais o robo deve se mover.

int med(int vel[5]) {
  int result;
  for (int e = 0 ; e < 5 ; e++)
    result = result + vel[e];

  return result / 5;
}


/******************************************************************************
   Função que faz a leitura da velocidade de uma das rodas através do encoder.
 ******************************************************************************/
void checkVel( void *pvParameters) {
  analogWrite(setVelDir, 255);
  analogWrite(setVelEsq, 255);
  Vel VEL, aux;
  int iE = 0, iD = 0;
  portBASE_TYPE xStatus;
  float velocidadeE = 0, velocidadeD = 0;
  float lastT1E = 0, T1E = 0, T2E = 0;
  float lastT1D = 0, T1D = 0, T2D = 0;
  boolean highFlagE = false, lowFlagE = true, flagesqE = true, flagVelE = false;
  boolean highFlagD = false, lowFlagD = true, flagesqD = true, flagVelD = false;
  int rawSensorValueE, rawSensorValueD;

  for (;;) {
    /**********************************************
     *** CALCULO DA VELOCIDADE DA RODA ESQUERDA ***
     **********************************************/
    if ((millis() - lastT1E) > 1000) {
      velocidadeE = 0;
      lastT1E = millis();
      flagVelE = true;
    }
    rawSensorValueE = analogRead(ENC_ESQ);

    if (rawSensorValueE < 650 && highFlagE && millis() - T2E > 30) { //Min value is 400 an
      T1E = millis();
      velocidadeE = 10 * (dist / 8) / ((T1E - lastT1E) / 1000);
      lastT1E = T1E;
      highFlagE = false;
      lowFlagE = true;
      flagVelE = true;
    }
    if (rawSensorValueE > 650 && lowFlagE && millis() - T1E > 30) {
      highFlagE = true;
      lowFlagE = false;
      T2E = millis();
    }
    if (flagVelE) {
      VEL.esq[iE] = velocidadeE;
      iE = iE + 1;
      if (iE >= 5)
        iE = 0;
      flagVelE = false;
    }
    //          Serial.print("VEC_ESQ ");
    //          Serial.print(VEL.esq[0]); Serial.print(" "); Serial.print(VEL.esq[1]); Serial.print(" "); Serial.print(VEL.esq[2]); Serial.print(" "); Serial.print(VEL.esq[3]); Serial.print(" "); Serial.print(VEL.esq[4]);
    //    Serial.print(" VEL.esq[");
    //    Serial.print(iE);
    //    Serial.print("]= ");
    //    Serial.print(VEL.esq[iE]);

    /*********************************************
     *** CALCULO DA VELOCIDADE DA RODA DIREITA ***
     *********************************************/
    if ((millis() - lastT1D) > 1000) {
      velocidadeD = 0;
      lastT1D = millis();
      flagVelD = true;
    }
    rawSensorValueD = analogRead(ENC_DIR);

    if (rawSensorValueD < 650 && highFlagD && millis() - T2D > 30) { //Min value is 400 an
      T1D = millis();
      velocidadeD = 10 * (dist / 8) / ((T1D - lastT1D) / 1000);
      lastT1D = T1D;
      highFlagD = false;
      lowFlagD = true;
      flagVelD = true;
    }
    if (rawSensorValueD > 650 && lowFlagD && millis() - T1D > 30) {
      highFlagD = true;
      lowFlagD = false;
      T2D = millis();
    }
    if (flagVelD) {
      VEL.dir[iD] = velocidadeD;
      iD = iD + 1;
      if (iD >= 5)
        iD = 0;
      flagVelD = false;
    }
    //        Serial.print(" || VEC_DIR ");
    //        Serial.print(VEL.dir[0]); Serial.print(" "); Serial.print(VEL.dir[1]); Serial.print(" "); Serial.print(VEL.dir[2]); Serial.print(" "); Serial.print(VEL.dir[3]); Serial.print(" "); Serial.println(VEL.dir[4]);
    //   Serial.print(" || VEL.dir[");
    //    Serial.print(iD);
    //    Serial.print("]= ");
    //    Serial.println(VEL.dir[iD]);



    //       if ((millis() - lastT1D) > 1000) {
    //          velocidadeD = 0;
    //          lastT1D;
    //          flagVelD = true;
    //        }
    //        rawSensorValueD = analogRead(ENC_ESQ);
    //        if (rawSensorValueD < 650 && highFlagD && millis() - T2D > 10) { //Min value is 400 an
    //          T1D = millis();
    //          velocidadeD = 10 * (dist / 8) / ((T1D - lastT1D) / 1000);
    //          lastT1D = T1D;
    //          highFlagD = false;
    //          lowFlagD = true;
    //          flagVelD = true;
    //        }
    //        if (rawSensorValueD > 650 && lowFlagD && millis() - T1D > 10) {
    //          highFlagD = true;
    //          lowFlagD = false;
    //          T2D = millis();
    //        }
    //        if(flagVelD){
    //          VEL.dir[iD] = velocidadeD;
    //
    //          iD = iD + 1;
    //          if (iD > 4)
    //            iD = 0;
    //          flagVelD = false;
    //        }
    //          Serial.print(" VEL.esq[");
    //          Serial.print(iE);
    //          Serial.print("]= ");
    //          Serial.println(VEL.esq[iE]);
    //
    if ( xSemaphoreTake( xVELSemaphore, ( TickType_t ) 10 ) == pdTRUE )
    {
      xStatus = xQueueReceive(xVelocidade, &aux, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueSendToBack(xVelocidade, &VEL, 0);
      xSemaphoreGive( xVELSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(30 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}


/******************************************************************************
 *** Faz o calculo dos parametro PI para estabilizar a velocidade das rodas.***
 ******************************************************************************/
void calcPID( void *pvParameters) {
  Vel VEL;
  int iEsq = 0;
  int iDir = 0;
  int velEsq;
  int velDir;
  SetPoint sp;
  int E = 0, P = 0, pwm = 0;
  portBASE_TYPE xStatus;

  for (;;) // A Task shall never return or exit.
  {
    //Leitura da Fila para verifiar os SetPoint Esquerdo e Direito
    if ( xSemaphoreTake( xSPSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
      xStatus = xQueuePeek(xSP, &sp, 5 / portTICK_PERIOD_MS);
      xSemaphoreGive( xSPSemaphore );
    }

    //Leitura da Fila para saber o valor da ultima velocidade lida na roda Esquerda e Direita
    if ( xSemaphoreTake( xVELSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
      xStatus = xQueuePeek(xVelocidade, &VEL, 5 / portTICK_PERIOD_MS);
      xSemaphoreGive( xVELSemaphore );
    }

    velEsq = med(VEL.esq);
    velDir = med(VEL.dir);

    //    Serial.print(" ESQ ");
    //    Serial.print(VEL.esq[0]); Serial.print(" "); Serial.print(VEL.esq[1]); Serial.print(" "); Serial.print(VEL.esq[2]); Serial.print(" "); Serial.print(VEL.esq[3]); Serial.print(" "); Serial.print(VEL.esq[4]);
    //    Serial.print(" DIR ");
    //    Serial.print(VEL.dir[0]); Serial.print(" "); Serial.print(VEL.dir[1]); Serial.print(" "); Serial.print(VEL.dir[2]); Serial.print(" "); Serial.print(VEL.dir[3]); Serial.print(" "); Serial.println(VEL.dir[4]);
    /************************************
     *** CALCULOS PARA A RODA ESQUERDA***
     ************************************/
    E = sp.esq - velEsq;   //Calculo do Erro da roda Esquerda
    P = Kp * E;           //Calculo do termo P do PI da roda Esquerda
    iEsq = iEsq + (0.5 * E) * Ki; //Incremento do termo I
    pwm = P + iEsq;       //Calculo do PWM resultante
    //Condições de contorno para saturação do PWM
    if (pwm > 255) {
      pwm = 255;
    }
    if (pwm < 0) {
      pwm = 0;
    }
    //Modifica o pwm da roda Esquerda, baseado no PI.
    analogWrite (setVelEsq, pwm);

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
      Serial.print("pwmEsq ");
      Serial.print(pwm);
      Serial.print("\tvelEsq  ");
      Serial.print(velEsq);
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    /***********************************
     *** CALCULOS PARA A RODA DIREITA***
     ***********************************/
    E = sp.dir - velDir;         //Calculo do Erro da roda Esquerda
    P = Kp * E;                 //Calculo do termo P do PI da roda Esquerda
    iDir = iDir + (0.5 * E) * Ki; //Incremento do termo I
    pwm = P + iDir;             //Calculo do PWM resultante

    //Condições de contorno para saturação do PWM
    if (pwm > 255) {
      pwm = 255;
    }
    if (pwm < 0) {
      pwm = 0;
    }
    //Modifica o pwm da roda Direita, baseado no PI.
    analogWrite (setVelDir, pwm);

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
      Serial.print("\tpwmDir ");
      Serial.print(pwm);
      Serial.print("\tvelDir ");
      Serial.println(velDir);
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    //taskYIELD();
    //Tempo de espera para que esta função seja chamada novamante.
    vTaskDelay(250 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}

/*********************************************************************
 *** Task responsavel por fazer a comunicação via serial com o XBEE***
 *********************************************************************/
void SerialComunication( void *pvParameters) {

  portBASE_TYPE xStatus;
  Position PREF, auxPREF, AP;
  Vel VEL;
  int velEsq, velDir;

  for (;;) // A Task shall never return or exit.
  {
    //Leitura dos valores da serial enviados pelo servidor.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if (Serial.available() > 0) {
        PREF.X = Serial.parseInt();      //Posição em X
        PREF.Y = Serial.parseInt();      //Posição em Y
        PREF.THETA = Serial.parseInt();  //Posição angular em Theta
      }
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    if ( xSemaphoreTake( xPREFSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
      xStatus = xQueueReceive(xReferencePosition, &auxPREF, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueSendToBack(xReferencePosition, &PREF, 0);
      xSemaphoreGive( xPREFSemaphore );
    }

    /*********************************************************************
     *** Envia via serial a posição atual do robo para o XBEE***
     *********************************************************************/
    if ( xSemaphoreTake( xAPSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
      xStatus = xQueuePeek(xActualPosition, &AP, 10 / portTICK_PERIOD_MS);
      xSemaphoreGive( xAPSemaphore );
    }

    if ( xSemaphoreTake( xVELSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
      xStatus = xQueuePeek(xVelocidade, &VEL, 10 / portTICK_PERIOD_MS);
      xSemaphoreGive( xVELSemaphore );
    }

    velEsq = med(VEL.esq);
    velDir = med(VEL.dir);

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.print("X");
      Serial.print(AP.X);
      Serial.print("\tY");
      Serial.print(AP.Y);
      Serial.print("\tT");
      Serial.print(AP.THETA);
      Serial.print("\tvelEsq");
      Serial.print(velEsq);
      Serial.print("\tvelDir");
      Serial.println(velDir);
      xSemaphoreGive( xSerialSemaphore );
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
}

/********************************************************************************
 *** Implementa o modelo cinemático do robo, para que ele possa se movimentar.***
 *******************************************************************************/
void calcPosition( void *pvParameters) {

  portBASE_TYPE xStatus;
  Position PREF, AP, auxAP;
  Vel VEL;
  SetPoint sp, auxSP;
  int velEsq, velDir;
  float setPointEsq = 0, setPointDir = 0;
  float velCentroMassa = 0;
  float wAngular = 0;
  float erro1 = 0, erro2 = 0, erro3 = 0;
  float newWAngular = 0, newVel = 0;
  int ki1 = 1, ki2 = 1, ki3 = 10;

  for (;;) // A Task shall never return or exit.
  {

    //Leitura da Fila para saber o valor da ultima velocidade lida na roda Esquerda e Direita
    //    if ( xSemaphoreTake( xVELSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
    //      xStatus = xQueuePeek(xVelocidade, &VEL, 10 / portTICK_PERIOD_MS);
    //      xSemaphoreGive( xVELSemaphore );
    //    }
    //
    //    //Pega da fila os valores de referencia passado pelo servidor
    //    if ( xSemaphoreTake( xPREFSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
    //      xStatus = xQueuePeek(xReferencePosition, &PREF, 10 / portTICK_PERIOD_MS);
    //      xSemaphoreGive( xPREFSemaphore );
    //    }

    velEsq = med(VEL.esq);
    velDir = med(VEL.dir);

    velCentroMassa = (velDir + velEsq) / 2;               //Velocidade do centro de massa.
    wAngular = (velDir - velEsq) / DISTANCIA_ENTRE_EIXOS; //Calculo da velocidade angular do carrinho.

    //Calcula a nova posição X e Y e o angulo theta
    AP.X = AP.X + 0.5 * velCentroMassa * cos(AP.THETA);
    AP.Y = AP.Y + 0.5 * velCentroMassa * sin(AP.THETA);
    AP.THETA = AP.THETA + 0.5 * wAngular;

    //Realiza o calculo dos erros de posicionamento em relação às referências.
    erro1 = cos(AP.THETA) * (PREF.X - AP.X) + sin(AP.THETA) * (PREF.Y - AP.Y);
    erro2 = -sin(AP.THETA) * (PREF.X - AP.X) + cos(AP.THETA) * (PREF.Y - AP.Y);
    erro3 = PREF.THETA - AP.THETA;

    //Baseado nos erros calculados, a baixo são calculadas as novas velocidades do centro de massa e angular do robo.
    newVel = 50 * cos(erro3) + ki1 * erro1;
    newWAngular = 10 + ki2 * 50 * erro2 + ki3 * 50 * sin(erro3);

    //Calculo dos novos setPoints das rodas Esquerda e Direita.
    sp.esq = 2 * DISTANCIA_ENTRE_EIXOS * (newVel - newWAngular) / (2 + DISTANCIA_ENTRE_EIXOS);
    sp.esq = 2 * DISTANCIA_ENTRE_EIXOS * (newVel + newWAngular) / (2 + DISTANCIA_ENTRE_EIXOS);

    //    //Atualisa os valores de setPoint das velocidades de cada roda
    //    if ( xSemaphoreTake( xSPSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
    //      xStatus = xQueueReceive(xSP, &auxSP, 10 / portTICK_PERIOD_MS);
    //      xStatus = xQueueSendToBack(xSP, &sp, 0);
    //      xSemaphoreGive( xSPSemaphore );
    //    }
    //
    //    //Atualisa os valores das posições atuais do robo
    //    if ( xSemaphoreTake( xAPSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
    //      xStatus = xQueueReceive(xActualPosition, &auxAP, 10 / portTICK_PERIOD_MS);
    //      xStatus = xQueueSendToBack(xActualPosition, &AP, 0);
    //      xSemaphoreGive( xAPSemaphore );
    //    }
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
}


void vApplicationIdleHook(void)
{
  ulIdleCycleCount++;
}

void setup() {

  Serial.begin(57600);
  while (!Serial) {}

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  /***************
  *** Semaforos **
  ****************/
  //Semafor para a serial
  if (xSerialSemaphore == NULL) { // Check to confirm that the Serial Semaphore has not already been created.
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if (xSerialSemaphore != NULL)
      xSemaphoreGive(xSerialSemaphore);  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  //Semaforo para a Queue da Posição atual
  if (xAPSemaphore == NULL) { // Check to confirm that the Serial Semaphore has not already been created.
    xAPSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if (xAPSemaphore != NULL)
      xSemaphoreGive(xAPSemaphore);  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  //Semaforo para a Queue da Posição de Referencia
  if (xPREFSemaphore == NULL) { // Check to confirm that the Serial Semaphore has not already been created.
    xPREFSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if (xPREFSemaphore != NULL)
      xSemaphoreGive(xPREFSemaphore);  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  //Semaforo para a Queue da Velocidade
  if (xVELSemaphore == NULL) { // Check to confirm that the Serial Semaphore has not already been created.
    xVELSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if (xVELSemaphore != NULL)
      xSemaphoreGive(xVELSemaphore);  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  //Semaforo para a Queue do SetPoint atual de Velocidade
  if (xSPSemaphore == NULL) { // Check to confirm that the Serial Semaphore has not already been created.
    xSPSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if (xSPSemaphore != NULL)
      xSemaphoreGive(xSPSemaphore);  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  /***********
  *** FILAS **
  ************/
  //Fila que guarda a velocidade da roda esquerda
  xVelocidade = xQueueCreate( 1, 1 * sizeof(Vel) );

  //Fila usada para armazenar os valores do SetPoint
  xSP = xQueueCreate(2, sizeof(SetPoint) );

  //Fila usada para armazenar as posições para as quais o robo deve se mover;
  xReferencePosition = xQueueCreate(3, sizeof(Position) );

  //Fila usada para armazenar as posições para as quais o robo deve se mover;
  xActualPosition = xQueueCreate(3, sizeof(Position) );


  /************
   *** TASKS **
   ************/
  //Task que calcula a velocida da roda Esquerda
  xTaskCreate(checkVel, "Velocímetro", 128, NULL, 1, NULL);

  //Task que Gerencia a comunicação com o XBEE
  //xTaskCreate(SerialComunication, "Comunication", 128, NULL, 1, NULL );

  //Task que Calcula o modeo cinemático do robo, para saber a posição atual e o setPont das rodas
  //xTaskCreate(calcPosition, "Position", 128, NULL, 1, NULL );

  //Task que Calcula o PID das rodas, dado um SetPoinr.
  xTaskCreate(calcPID, "PID", 128, NULL, 2, NULL);

  /* Start the scheduler so the created tasks start executing. */
  vTaskStartScheduler();

  for (;;);

}

void loop() {
  // put your main code here, to run repeatedly:

}
