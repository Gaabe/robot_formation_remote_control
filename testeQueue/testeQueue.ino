#include <Arduino_FreeRTOS.h>
#include <queue.h>

#define mainSENDER_1 1
#define mainSENDER_2 2

xQueueHandle xQueue;
typedef struct
{
  unsigned char ucValue;
  unsigned char ucSource; 
} xData;

static const xData xStructsToSend[2] = 
{
  { 100, mainSENDER_1},
  { 200, mainSENDER_2}
};

static void vSenderTask(void *pvParameters)
{
  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 100 / portTICK_RATE_MS;

  for(;;)
  {
    xStatus = xQueueSendToBack(xQueue, pvParameters, xTicksToWait);

    if (xStatus != pdPASS)
    {
      Serial.println("Could not send to the queue");
    }
    taskYIELD();
  }
}

static void vReceiverTask(void *pvParameters)
{
  portBASE_TYPE xStatus;
  xData xReceivedStructure;

  for (;;)
  {
    if (uxQueueMessagesWaiting(xQueue) != 3)
    {
      Serial.println("Queue should have been full!");
    }

    xStatus = xQueueReceive(xQueue, &xReceivedStructure, 0);

    if (xStatus=pdPASS)
    {
      if (xReceivedStructure.ucSource == mainSENDER_1)
      {
        Serial.print("Sender 1 : ");
        Serial.println(xReceivedStructure.ucValue);
      }
      else
      {
        Serial.print("Sender 2 : ");
        Serial.println(xReceivedStructure.ucValue);
      }  
    }
    
    else
    {
      Serial.println("Could not receive from the queue");
    }
  }
}

void setup() {

  Serial.begin(9600);
  
  // put your setup code here, to run once:
  xQueue = xQueueCreate(3, sizeof(xData));

  if (xQueue != NULL)
  {
    xTaskCreate(vSenderTask, "Sender 1", 128, &(xStructsToSend[0]), 2, NULL);
    xTaskCreate(vSenderTask, "Sender 2", 128, &(xStructsToSend[1]), 3, NULL);

    xTaskCreate(vReceiverTask, "Receiver", 128, NULL, 1, NULL);

    vTaskStartScheduler();

  }
  else
  {
    
  }
  
  for(;;);

  
}

void loop() {
  // put your main code here, to run repeatedly:

}
