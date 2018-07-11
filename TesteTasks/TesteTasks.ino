#include <Arduino_FreeRTOS.h>
//#include <semphr.h>
//#include <task.h>

unsigned long ulIdleCycleCount = 0UL;

void checkVel( void *pvParameters)
{
  char *pcTaskName;

  pcTaskName = (char *) pvParameters;


  for(;;)
  {
  Serial.print(pcTaskName);
  //Serial.println(ulIdleCycleCount);
  
  // Serial.println(pdTICKS_PER_MS());
  vTaskDelay(250/portTICK_PERIOD_MS); //delay em num de ticks, se usar o / fica em ms
  
  }
}

void vApplicationIdleHook(void)
{
  ulIdleCycleCount++;
}

void setup() {

  Serial.begin(9600);
  
  // put your setup code here, to run once:
  static const char *pcTextForTask1 = "0\r\n";
  static const char *pcTextForTask2 = "1\t\n";
  //coloquei as tasks com prioridades diferentes
  xTaskCreate(checkVel, "Velocímetro", 128, (void*)pcTextForTask1, 1, NULL); 
  xTaskCreate(checkVel, "Task 2", 128, (void*)pcTextForTask2, 2, NULL);

  vTaskStartScheduler();

  for(;;);

  
}

void loop() {
  // put your main code here, to run repeatedly:

}
