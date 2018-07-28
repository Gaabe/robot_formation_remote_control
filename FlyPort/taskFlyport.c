#include "taskFlyport.h"

//Function to send a message throug the established TCP conection.
int sendTCP(char* msg, TCP_SOCKET sock);

//Struct para salvar a sincronização do flyport
struct tm sinc;



//Main function of the FlyPort
void FlyportTask(){
	
	TCP_SOCKET sock = INVALID_SOCKET;
	char msg[500];
	int cnt = 0, i = 0, pwmValue = 0;
	BOOL flagErr = FALSE;
	double adcValue[10], temp = 0;

	UARTInit(1, 19200);
	UARTOn(1);

	//Flyport conexion with the Wizard predefined network
	WFConnect(WF_DEFAULT);
	while(WFGetStat() != CONNECTED);
	while(!DHCPAssigned);
	vTaskDelay(25);

	UARTWrite(1,"Flyport Wi-fi G connected...hello world!\r\n");

	//Connects the TCP Client to the server
	sock = TCPClientOpen("192.168.0.105","5000");

	PWMInit(1,1000,100);//Initialize PWM1 to work at 1000 Hz, 100% of duty
	PWMOn(p4, 1);//Assign pin 13 as PWM1 and turns it on

	//Timeout function
	while(!TCPisConn(sock)) {
		if(cnt==10){
		  flagErr = TRUE;
		  break;
		}
		vTaskDelay(50);
		cnt++;
	}
	if(flagErr)
		UARTWrite(1,"\r\nTimeout error...\r\n");
	else{
		//LOOP to colect the 10 first ADC Values for the moving average.
		for(i=0 ; i<10 ; i++){
			adcValue[i] = ADCVal(3);//acquire the value from ADC channel 1
			vTaskDelay(50);
		}
		
		sprintf(msg,"ID1");
		sendTCP(msg, sock);
	
		while(1){
			adcValue[cnt++] = ADCVal(3); //Subtitutes the oldest value of temperature for the new one
			if(cnt > 9)	//The moving average uses 10 values
				cnt = 0;
				
			//LOOP to calculate recalculate the moving average.
			for(i=0, temp=0 ; i<10 ; i++){ 
				temp += adcValue[i];
			}
			temp = temp/10; //Gets the mean.
			
			
			//Prints the temperature to the Serial
			//sprintf(msg, "\r\nTEMP = %d", temp);
			//UARTWrite(1, msg);			
			temp = temp/5;  //Turns the mean into temperature
			
			//Turns the temparature value into PWM value.
			pwmValue = (int)((temp/150) * 100);

			//Prints the PWM to the Serial
			PWMDuty(pwmValue, 1);//Change duty of the PWM
				sprintf(msg, "\r\nPWM = %d\r\n", pwmValue);
				UARTWrite(1, msg);
			
			sprintf(msg,"%f", temp);
			if(!sendTCP(msg, sock)) //Sends the temperature throug the TCP conection
				break;
		}
	}
}

int sendTCP(char* msg, TCP_SOCKET sock){
	int RxLen=0;
	char bff[250];
	
	//Sends the temperature through the TCP conection
	TCPWrite(sock, msg, strlen(msg));
	
	//Waits until the ACK mesage is recived from the server
	vTaskDelay(50);
	while((RxLen=TCPRxLen(sock))<=0){
		sprintf(msg,"Wating\r\n");
		UARTWrite(1,msg);	
	}
	
	//Reads the recived mesage form the server
	msg[0] = '\0';
	while((RxLen=TCPRxLen(sock))>0){
	  TCPRead(sock,bff,RxLen);
	  strcat(msg,bff);
	}
	
	//Saves the syncing time 
	if(msg[0] != '\0'){
		char buf[4], msg1[100];	
		if(msg[2] != '\0'){
			if(msg[0] == 'O' && msg[1] == 'K'){
				sprintf(msg1,"Bads\r\n");
				UARTWrite(1,msg1);
				strncpy(msg, msg+2, 14);
			}
			UARTWrite(1,msg);
			memset(buf, '\0', sizeof(buf));
			strncpy(buf,msg,4);
			sinc.tm_year = atoi(buf);
			memset(buf, '\0', sizeof(buf));
			strncpy(buf,msg+4,2);
			sinc.tm_mon = atoi(buf);
			memset(buf, '\0', sizeof(buf));
			strncpy(buf,msg+6,2);
			sinc.tm_mday = atoi(buf);
			memset(buf, '\0', sizeof(buf));
			strncpy(buf,msg+8,2);
			sinc.tm_hour = atoi(buf);
			memset(buf, '\0', sizeof(buf));
			strncpy(buf,msg+10,2);
			sinc.tm_min = atoi(buf);
			memset(buf, '\0', sizeof(buf));
			strncpy(buf,msg+12,2);
			sinc.tm_sec = atoi(buf);
			
			RTCCSet(&sinc);
		}
		RTCCGet(&sinc);
		sprintf(msg,"\r\n%d\r\n", sinc.tm_sec);
		UARTWrite(1,msg);	
	}
	else{
		sprintf(msg,"Bads\r\n");
		UARTWrite(1,msg);
		return 0;
	}
	return 1;
}
