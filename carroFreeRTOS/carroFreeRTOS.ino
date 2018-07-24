  /******************************************************
   ****************** Robo Bombeiro *********************
   * Programação de tempo real para sistemas embarcados *
   ******************************************************/
  
  #include <Arduino_FreeRTOS.h>
  #include <semphr.h>
  
  #define direita 0
  #define esquerda 1
  
  #define DISTANCIA_ENTRE_EIXOS 100
  
  /* TODO
     ADICIONAR PID AS RODAS
     ADICIONAR CONTROLE DE POSICAO
     ENVIAR MSGS PELO XBEE
     ADICIONAR CONTROLE DE TIME
  */
  
  unsigned long ulIdleCycleCount = 0UL;
  
  // declarações de pinos
  int setVelEsq = 6; //M1 Speed Control
  int setVelDir = 5; //M0 Speed Control
  int setSentidoEsq = 8; //M1 Direction Control
  int setSentidoDir = 7; //M0 Direction Control
  
  int Kp = 1, Ki = 5;
  
  SemaphoreHandle_t xSerialSemaphore;
  
  //Declaração global das filas
  QueueHandle_t xVelocidadeEsq; //Fila que guarda a velocidade da roda esquerda
  QueueHandle_t xVelocidadeDir; //Fila que guarda a velocidade da roda direita
  
  QueueHandle_t xSP; //Fila usada para armazenar os valores do SetPoint
  QueueHandle_t xI; //Fila usada para armazenar os valores do termo I do PID e poder incrementalos.
  QueueHandle_t xReferencePosition; //Fila usada para armazenar as posições para as quais o robo deve se mover.
  QueueHandle_t xPosition; //Fila usada para armazenar as posições atuais do robo;
  
  /******************************************************************************
     Função que faz a leitura da velocidade de uma das rodas através do encoder.
   ******************************************************************************/
  void checkVel( void *pvParameters) {
    int motor = (int)pvParameters;
    portBASE_TYPE xStatus;
    float velocidade = 0;
  
    float lastT1 = 0, T1 = 0, T2 = 0;
    boolean highFlag, lowFlag;
    if (analogRead(motor) < 650) {
      highFlag = false;
      lowFlag = true;
    }
    else {
      highFlag = true;
      lowFlag = false;
    }
    char *pcTaskName;
    int rawSensorValue;
    char dist = 11;
    boolean sensorTrig = false;
    boolean sensorFlag = false;
    float periodo;
    int aux;
    pcTaskName = (char *) pvParameters;
    for (;;) {
      if ((millis() - lastT1) > 5000) {
        velocidade = 0;
      }
      rawSensorValue = analogRead(motor);
  
      if (rawSensorValue < 650 && highFlag && millis() - T2 > 30) { //Min value is 400 an
        T1 = millis();
        velocidade = 10 * (dist / 8) / ((T1 - lastT1) / 1000);
  
        lastT1 = T1;
        highFlag = false;
        lowFlag = true;
      }
      if (rawSensorValue > 650 && lowFlag && millis() - T1 > 30) {
        highFlag = true;
        lowFlag = false;
        T2 = millis();
      }
  
      if (motor == esquerda) {
        xStatus = xQueueReceive(xVelocidadeEsq, &aux, 5 / portTICK_PERIOD_MS);
        xStatus = xQueueSendToBack(xVelocidadeEsq, &velocidade, 0);
      }
      else {
        xStatus = xQueueReceive(xVelocidadeDir, &aux, 10 / portTICK_PERIOD_MS);
        xStatus = xQueueSendToBack(xVelocidadeDir, &velocidade, 0);
      }
      vTaskDelay(50 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
    }
  }
  
  
  /******************************************************************************
   *** Faz o calculo dos parametro PI para estabilizar a velocidade das rodas.***
   ******************************************************************************/
  void calcPID( void *pvParameters) {
    float velDir;
    float velEsq;
    int IEsq = 0;
    int IDir = 0;
    int SPDir = 50;
    int SPEsq = 50;
    int E = 0, P = 0, pwm = 0;
    portBASE_TYPE xStatus;
  
    for (;;) // A Task shall never return or exit.
    {
  
      //Leitura da Fila para verifiar os SetPoint Esquerdo e Direito
      xStatus = xQueueReceive(xSP, &SPEsq, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xSP, &SPDir, 10 / portTICK_PERIOD_MS);
  
      //Leitura da Fila para verifiar o termo I do PI Esquero e Direito
      xStatus = xQueueReceive(xI, &IEsq, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xI, &IDir, 10 / portTICK_PERIOD_MS);
  
      //Leitura da Fila para saber o valor da ultima velocidade lida na roda Esquerda e Direita
      xStatus = xQueuePeek(xVelocidadeEsq, &velEsq, 10 / portTICK_PERIOD_MS);
      xStatus = xQueuePeek(xVelocidadeDir, &velDir, 10 / portTICK_PERIOD_MS);
  
      /************************************
       *** CALCULOS PARA A RODA ESQUERDA***
       ************************************/
      E = SPEsq - velEsq;   //Calculo do Erro da roda Esquerda
      P = Kp * E;           //Calculo do termo P do PI da roda Esquerda
      IEsq = IEsq + (0.5 * E) * Ki; //Incremento do termo I
      pwm = P + IEsq;       //Calculo do PWM resultante
      //Condições de contorno para saturação do PWM
      if (pwm > 255) {
        pwm = 255;
      }
      if (pwm < 0) {
        pwm = 0;
      }
      //Modifica o pwm da roda Esquerda, baseado no PI.
      analogWrite (setVelEsq, pwm);
  
      /***********************************
       *** CALCULOS PARA A RODA DIREITA***
       ***********************************/
      E = SPDir - velDir;         //Calculo do Erro da roda Esquerda
      P = Kp * E;                 //Calculo do termo P do PI da roda Esquerda
      IDir = IDir + (0.5 * E) * Ki; //Incremento do termo I
      pwm = P + IDir;             //Calculo do PWM resultante
      //Condições de contorno para saturação do PWM
      if (pwm > 255) {
        pwm = 255;
      }
      if (pwm < 0) {
        pwm = 0;
      }
      //Modifica o pwm da roda Direita, baseado no PI.
      analogWrite (setVelDir, pwm);
  
      //Grava os valores dos SetPoints e dos termos I do PI de volta na fila, para poder ler na proxima vez que chamar a função.
      xStatus = xQueueSendToBack( xSP, &SPEsq, 0);
      xStatus = xQueueSendToBack( xSP, &SPDir, 0);
      xStatus = xQueueSendToBack( xI, &IEsq, 0);
      xStatus = xQueueSendToBack( xI, &IDir, 0);
  
      //taskYIELD();
      //Tempo de espera para que esta função seja chamada novamante.
      vTaskDelay(200 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
    }
  }
  
  /*********************************************************************
   *** Task responsavel por fazer a comunicação via serial com o XBEE***
   *********************************************************************/
  void SerialComunication( void *pvParameters) {
  
    portBASE_TYPE xStatus;
    float xR = 0, yR = 0, thetaR = 0;
    float aux;
    float X = 0, Y = 0, THETA = 0;
  
    for (;;) // A Task shall never return or exit.
    {
      //Leitura dos valores da serial enviados pelo servidor.
      if (Serial.available() > 0)
      {
        if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
        {
          xR = Serial.parseInt();      //Posição em X
          yR = Serial.parseInt();      //Posição em Y
          thetaR = Serial.parseInt();  //Posição angular em Theta
          xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
        }
  
        xStatus = xQueueReceive(xReferencePosition, &aux, 10 / portTICK_PERIOD_MS);
        xStatus = xQueueReceive(xReferencePosition, &aux, 10 / portTICK_PERIOD_MS);
        xStatus = xQueueReceive(xReferencePosition, &aux, 10 / portTICK_PERIOD_MS);
  
        xStatus = xQueueSendToBack(xReferencePosition, &xR, 0);
        xStatus = xQueueSendToBack(xReferencePosition, &yR, 0);
        xStatus = xQueueSendToBack(xReferencePosition, &thetaR, 0);
      }
  
      /*********************************************************************
       *** Envia via serial a posição atual do robo para o XBEE***
       *********************************************************************/
  
      //Pega da fila os valores da ultima posição calculada
      xStatus = xQueueReceive(xPosition, &X, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xPosition, &Y, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xPosition, &THETA, 10 / portTICK_PERIOD_MS);
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        Serial.print("X ");
        Serial.print(X);
        Serial.print("Y ");
        Serial.print(Y);
        Serial.print("T ");
        Serial.println(THETA);
        xSemaphoreGive( xSerialSemaphore );
      }
      //Reenvia os valores das posições X e Y e o angulo Theta para a fila.
      xStatus = xQueueSendToBack(xReferencePosition, &X, 0);
      xStatus = xQueueSendToBack(xReferencePosition, &Y, 0);
      xStatus = xQueueSendToBack(xReferencePosition, &THETA, 0);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  }
  
  /********************************************************************************
   *** Implementa o modelo cinemático do robo, para que ele possa se movimentar.***
   *******************************************************************************/
  void calcPosition( void *pvParameters) {
  
    portBASE_TYPE xStatus;
    float xR = 0, yR = 0, thetaR = 0;
    float X = 0, Y = 0, THETA = 0;
    float velEsq = 0, velDir = 0;
    float setPointEsq = 0, setPointDir = 0;
    float velCentroMassa = 0;
    float wAngular = 0;
    float erro1 = 0, erro2 = 0, erro3 = 0;
    float newWAngular = 0, newVel = 0;
    int ki1 = 1, ki2 = 1, ki3 = 10;
    int aux;
  
    for (;;) // A Task shall never return or exit.
    {
      //Leitura da Fila para saber o valor da ultima velocidade lida na roda Esquerda e Direita
      xStatus = xQueuePeek(xVelocidadeEsq, &velEsq, 2 / portTICK_PERIOD_MS);
      xStatus = xQueuePeek(xVelocidadeDir, &velDir, 2 / portTICK_PERIOD_MS);
  
      velCentroMassa = (velDir + velEsq) / 2;               //Velocidade do centro de massa.
      wAngular = (velDir - velEsq) / DISTANCIA_ENTRE_EIXOS; //Calculo da velocidade angular do carrinho.
  
      //Pega da fila os valores de referencia passado pelo servidor
      xStatus = xQueueReceive(xReferencePosition, &xR, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xReferencePosition, &yR, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xReferencePosition, &thetaR, 10 / portTICK_PERIOD_MS);
  
      //Envia de volta para a fila os valores de referência.
      xStatus = xQueueSendToBack(xReferencePosition, &xR, 0);
      xStatus = xQueueSendToBack(xReferencePosition, &yR, 0);
      xStatus = xQueueSendToBack(xReferencePosition, &thetaR, 0);
  
      //Pega da fila os valores da ultima posição calculada
      xStatus = xQueueReceive(xPosition, &X, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xPosition, &Y, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xPosition, &THETA, 10 / portTICK_PERIOD_MS);
  
      //Calcula a nova posição X e Y e o angulo theta
      X = X + 0.5 * velCentroMassa * cos(THETA);
      Y = Y + 0.5 * velCentroMassa * sin(THETA);
      THETA = THETA + 0.5 * wAngular;
  
      //Salva os novos valores das posições X e Y e o angulo Theta na fila.
      xStatus = xQueueSendToBack(xReferencePosition, &X, 0);
      xStatus = xQueueSendToBack(xReferencePosition, &Y, 0);
      xStatus = xQueueSendToBack(xReferencePosition, &THETA, 0);
  
      //Realiza o calculo dos erros de posicionamento em relação às referências.
      erro1 = cos(THETA) * (xR - X) + sin(THETA) * (yR - Y);
      erro2 = -sin(THETA) * (xR - X) + cos(THETA) * (yR - Y);
      erro3 = thetaR - THETA;
  
      //Baseado nos erros calculados, a baixo são calculadas as novas velocidades do centro de massa e angular do robo.
      newVel = 50 * cos(erro3) + ki1 * erro1;
      newWAngular = 10 + ki2 * 50 * erro2 + ki3 * 50 * sin(erro3);
  
      //Calculo dos novos setPoints das rodas Esquerda e Direita.
      setPointEsq = 2 * DISTANCIA_ENTRE_EIXOS * (newVel - newWAngular) / (2 + DISTANCIA_ENTRE_EIXOS);
      setPointDir = 2 * DISTANCIA_ENTRE_EIXOS * (newVel + newWAngular) / (2 + DISTANCIA_ENTRE_EIXOS);
  
      //Tira os valores que estavam na fila de SetPoint
      xStatus = xQueueReceive(xSP, &aux, 10 / portTICK_PERIOD_MS);
      xStatus = xQueueReceive(xSP, &aux, 10 / portTICK_PERIOD_MS);
  
      //Atualisa os valores de setPoint das velocidades de cada roda
      xStatus = xQueueSendToBack(xSP, &setPointEsq, 0);
      xStatus = xQueueSendToBack(xSP, &setPointDir, 0);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
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
  
    //Fila que guarda a velocidade da roda esquerda
    xVelocidadeEsq = xQueueCreate( 1, 1 * sizeof(float) );
    //Fila que guarda a velocidade da roda direita
    xVelocidadeDir = xQueueCreate( 1, 1 * sizeof(float) );
  
    //Fila usada para armazenar os valores do SetPoint
    xSP = xQueueCreate(2, sizeof(int) );
    //Fila usada para armazenar os valores do termo I do PID e poder incrementalos.
    xI = xQueueCreate(2, sizeof(int) );
  
    //Fila usada para armazenar as posições para as quais o robo deve se mover;
    xReferencePosition = xQueueCreate(3, sizeof(float) );
    //Fila usada para armazenar as posições atuais do robo;
    xPosition = xQueueCreate(3, sizeof(float) );
  
  
    //Task que calcula a velocida da roda Esquerda
    xTaskCreate(checkVel, "Velocímetro Esquerda", 128, (void *) esquerda, 1, NULL);
  
    //Task que calcula a velocida da roda Direita
    xTaskCreate(checkVel, "Velocímetro Direita", 128, (void *) direita, 1, NULL);
  
    //Task que Gerencia a comunicação com o XBEE
    xTaskCreate(SerialComunication, "Comunication", 128, NULL, 1, NULL );
  
    //Task que Calcula o modeo cinemático do robo, para saber a posição atual e o setPont das rodas
    xTaskCreate(calcPosition, "Position", 128, NULL, 1, NULL );
  
    //Task que Calcula o PID das rodas, dado um SetPoinr.
    xTaskCreate(calcPID, "PID", 128, NULL, 1, NULL);
  
    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();
  
    for (;;);
  
  }
  
  void loop() {
    // put your main code here, to run repeatedly:
  
  }
