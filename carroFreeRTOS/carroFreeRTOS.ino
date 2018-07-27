  /******************************************************
   ****************** Robo Bombeiro *********************
   * Programação de tempo real para sistemas embarcados *
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
  typedef struct{
    int esq[5] = {0,0,0,0,0};
    int dir[5] = {0,0,0,0,0};
    int i = 0;
  } vel;

  //Struct utilizada para guardar OS  Set Points
  typedef struct{
    int esq = 0;
    int dir = 0;
  } SetPoint;

  //Struct utilizada para guardar Posições de Referência
  typedef struct{
    int X;
    int Y;
    int THETA;
  } Position;

  //Semafor para a serial
  SemaphoreHandle_t xSerialSemaphore;

  //Semaforo para a Queue da Posição atual
  SemaphoreHandle_t xPSemaphore;

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

  int med(int vel[5]){
    int result;
    for(int e=0 ; e<5 ; e++)
      result = result + vel[e];

    return result/5;
  }

  
  /******************************************************************************
     Função que faz a leitura da velocidade de uma das rodas através do encoder.
   ******************************************************************************/
  void checkVel( void *pvParameters) {
    vel VEL, aux;
    int i = 0;
    portBASE_TYPE xStatus;
    float velocidade = 0;
    float lastT1 = 0, T1 = 0, T2 = 0;
    boolean highFlag = false, lowFlag = true;
    int rawSensorValue;

    for (;;) {
      /**********************************************
       *** CALCULO DA VELOCIDADE DA RODA ESQUERDA ***
       **********************************************/
      while(1){
        if ((millis() - lastT1) > 1000) {
          velocidade = 0;
          lastT1 = millis();
          break;
        }
        rawSensorValue = analogRead(ENC_ESQ);

        if (rawSensorValue < 650 && highFlag && millis() - T2 > 30) { //Min value is 400 an
          T1 = millis();
          velocidade = 10 * (dist / 8) / ((T1 - lastT1) / 1000);
          lastT1 = T1;
          highFlag = false;
          lowFlag = true;
          break;
        }
        if (rawSensorValue > 650 && lowFlag && millis() - T1 > 30) {
          highFlag = true;
          lowFlag = false;
          T2 = millis();
        }
      }
      VEL.esq[i] = velocidade;

      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }

      /*********************************************
       *** CALCULO DA VELOCIDADE DA RODA DIREITA ***
       *********************************************/
      velocidade = 0;
      lastT1 = 0;
      T1 = 0;
      T2 = 0;
      highFlag = false;
      lowFlag = true;
      
      while(1){
        if ((millis() - lastT1) > 1000) {
          velocidade = 0;
          lastT1 = millis();
          break;
        }
        rawSensorValue = analogRead(ENC_DIR);

        if (rawSensorValue < 650 && highFlag && millis() - T2 > 10) { //Min value is 400 an
          T1 = millis();
          velocidade = 10 * (dist / 8) / ((T1 - lastT1) / 1000);
          lastT1 = T1;
          highFlag = false;
          lowFlag = true;
          break;
        }
        if (rawSensorValue > 650 && lowFlag && millis() - T1 > 10) {
          highFlag = true;
          lowFlag = false;
          T2 = millis();
        }
      }
      VEL.dir[i] = velocidade;

      i = i + 1;
      if(i > 4)
        i = 0;

      xStatus = xQueueReceive(xVelocidade, &aux, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueSendToBack(xVelocidade, &VEL, 0);     

            if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
      }
       
      vTaskDelay(50 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
    }
  }
  
  
  /******************************************************************************
   *** Faz o calculo dos parametro PI para estabilizar a velocidade das rodas.***
   ******************************************************************************/
  void calcPID( void *pvParameters) {
    vel VEL;
    int iEsq = 0;
    int iDir = 0;
    int velEsq;
    int velDir;
    SetPoint sp;
    sp.esq = 50;
    sp.dir = 50;
    int E = 0, P = 0, pwm = 0;
    portBASE_TYPE xStatus;
  
    for (;;) // A Task shall never return or exit.
    {
      //Leitura da Fila para verifiar os SetPoint Esquerdo e Direito
      xStatus = xQueuePeek(xSP, &sp, 10 / portTICK_PERIOD_MS);          

      //Leitura da Fila para saber o valor da ultima velocidade lida na roda Esquerda e Direita
      xStatus = xQueuePeek(xVelocidade, &VEL, 10 / portTICK_PERIOD_MS);
  
      velEsq = med(VEL.esq);
      velDir = med(VEL.dir);

      Serial.print("esq  ");
      Serial.println(velEsq);
      Serial.print("VEC_ESQ ");
      Serial.print(VEL.esq[0]);Serial.print(" ");Serial.print(VEL.esq[1]);Serial.print(" ");Serial.print(VEL.esq[2]);Serial.print(" ");Serial.print(VEL.esq[3]);Serial.print(" ");Serial.println(VEL.esq[4]);
      Serial.print("dir ");
      Serial.println(velDir);
      Serial.print("VEC_DIR ");
      Serial.print(VEL.dir[0]);Serial.print(" ");Serial.print(VEL.dir[1]);Serial.print(" ");Serial.print(VEL.dir[2]);Serial.print(" ");Serial.print(VEL.dir[3]);Serial.print(" ");Serial.println(VEL.dir[4]);
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

      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        Serial.print("pwm Esq ");
        Serial.println(pwm);
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

      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        Serial.print("pwm Dir ");
        Serial.println(pwm);
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
    Position PREF, aux, P;
    vel VEL;
    int velEsq, velDir;
  
    for (;;) // A Task shall never return or exit.
    {
      //Leitura dos valores da serial enviados pelo servidor.
      if (Serial.available() > 0)
      {
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
          PREF.X = Serial.parseInt();      //Posição em X
          PREF.Y = Serial.parseInt();      //Posição em Y
          PREF.THETA = Serial.parseInt();  //Posição angular em Theta
          xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
        }

        xStatus = xQueueReceive(xReferencePosition, &aux, 10 / portTICK_PERIOD_MS);
        xStatus = xQueueSendToBack(xReferencePosition, &PREF, 0);
      }

      /*********************************************************************
       *** Envia via serial a posição atual do robo para o XBEE***
       *********************************************************************/
     if ( xSemaphoreTake( xPSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
          xStatus = xQueuePeek(xActualPosition, &P, 10 / portTICK_PERIOD_MS);
    
          xSemaphoreGive( xPSemaphore );
      }
  
      

      xStatus = xQueuePeek(xVelocidade, &VEL, 10 / portTICK_PERIOD_MS);

      velEsq = med(VEL.esq);
      velDir = med(VEL.dir);

      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        Serial.print("X ");
        Serial.print(P.X);
        Serial.print("Y ");
        Serial.print(P.Y);
        Serial.print("T ");
        Serial.println(P.THETA);
        Serial.print("velEsq ");
        Serial.print(velEsq);
        Serial.print("velDir ");
        Serial.print(velDir);
        xSemaphoreGive( xSerialSemaphore );
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
  
  /********************************************************************************
   *** Implementa o modelo cinemático do robo, para que ele possa se movimentar.***
   *******************************************************************************/
  void calcPosition( void *pvParameters) {
  
    portBASE_TYPE xStatus;
    Position PREF, P, auxP;
    vel VEL;
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
      xStatus = xQueuePeek(xVelocidade, &VEL, 2 / portTICK_PERIOD_MS);



      //Pega da fila os valores de referencia passado pelo servidor
      xStatus = xQueuePeek(xReferencePosition, &PREF, 10 / portTICK_PERIOD_MS);
          
            
      velEsq = med(VEL.esq);
      velDir = med(VEL.dir);
      
      velCentroMassa = (velDir + velEsq) / 2;               //Velocidade do centro de massa.
      wAngular = (velDir - velEsq) / DISTANCIA_ENTRE_EIXOS; //Calculo da velocidade angular do carrinho.
    
      //Calcula a nova posição X e Y e o angulo theta
      P.X = P.X + 0.5 * velCentroMassa * cos(P.THETA);
      P.Y = P.Y + 0.5 * velCentroMassa * sin(P.THETA);
      P.THETA = P.THETA + 0.5 * wAngular;
  
      //Realiza o calculo dos erros de posicionamento em relação às referências.
      erro1 = cos(P.THETA) * (PREF.X - P.X) + sin(P.THETA) * (PREF.Y - P.Y);
      erro2 = -sin(P.THETA) * (PREF.X - P.X) + cos(P.THETA) * (PREF.Y - P.Y);
      erro3 = PREF.THETA - P.THETA;
  
      //Baseado nos erros calculados, a baixo são calculadas as novas velocidades do centro de massa e angular do robo.
      newVel = 50 * cos(erro3) + ki1 * erro1;
      newWAngular = 10 + ki2 * 50 * erro2 + ki3 * 50 * sin(erro3);
  
      //Calculo dos novos setPoints das rodas Esquerda e Direita.
      sp.esq = 2 * DISTANCIA_ENTRE_EIXOS * (newVel - newWAngular) / (2 + DISTANCIA_ENTRE_EIXOS);
      sp.esq = 2 * DISTANCIA_ENTRE_EIXOS * (newVel + newWAngular) / (2 + DISTANCIA_ENTRE_EIXOS);
  
      //Tira os valores que estavam na fila de SetPoint
      xStatus = xQueueReceive(xSP, &auxSP, 10/portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xActualPosition, &auxP, 10/portTICK_PERIOD_MS);
  
      //Atualisa os valores de setPoint das velocidades de cada roda
      xStatus = xQueueSendToBack(xSP, &sp, 0);
      xStatus = xQueueSendToBack(xActualPosition, &P, 0);
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

              /***********
              *** FILAS **
              ************/
    //Fila que guarda a velocidade da roda esquerda
    xVelocidade = xQueueCreate( 1, 1 * sizeof(vel) );
  
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
