#include "taskFlyport.h"

//Function to send a message throug the established TCP conection.
int sendTCP(char* msg, TCP_SOCKET sock);

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

	// Flyport connects to default network
	WFConnect(WF_DEFAULT);
	while(WFGetStat() != CONNECTED);
	while(!DHCPAssigned);
	vTaskDelay(25);
	UARTWrite(1,"Flyport Wi-fi G connected...hello world!\r\n");

	//Connects the TCP Client to the server
	sock = TCPClientOpen("192.168.0.133","5000");

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
		for(i=0 ; i<10 ; i++){
			adcValue[i] = ADCVal(3);
			vTaskDelay(50);
		}
		
		sprintf(msg,"ID4");
		sendTCP(msg, sock);
		//TCPWrite(sock, msg, strlen(msg));
		//UARTWrite(1,msg);
	
		while(1){
			//DEBUG do valor substituido no array da media movel
				//sprintf(msg,"\r\ncnt = %d\r\nadcValue[10] = ", cnt);
				//UARTWrite(1,msg);
			adcValue[cnt++] = ADCVal(3);//acquire the value from ADC channel 1
			if(cnt > 9)
				cnt = 0;

			for(i=0, temp=0 ; i<10 ; i++){ //LOOP to colect the 10 first ADC Values.
				temp += adcValue[i];
				//DEBUG Valores utilizados na MEDIA MOVEL
					//sprintf(msg,"[%d]=%.0f  ", i, adcValue[i]);
					//UARTWrite(1,msg);					
			}
			temp = temp/10; //Gets the mean.
			
			pwmValue = (int)((temp/150) * 100);				
			PWMDuty(pwmValue, 1);//Change duty of the PWM
			//DEBUG PWM
				sprintf(msg, "\r\nPWM = %d\r\n", pwmValue);
				UARTWrite(1, msg);
			
			temp = temp/5;  //Turns the mean into temperature
			sprintf(msg,"%f", temp);
			if(!sendTCP(msg, sock)) //Sends the temperature throug the TCP conection
				break;
		}
	}
}

int sendTCP(char* msg, TCP_SOCKET sock){
	int RxLen=0;
	char bff[250];
	
	TCPWrite(sock, msg, strlen(msg));
	
	//With the following code it is possible to receive data
	//from the server and print on the UART the message	while((RxLen=TCPRxLen(sock))>0)
	vTaskDelay(50);
	while((RxLen=TCPRxLen(sock))<=0){
		sprintf(msg,"Wating\r\n");
		UARTWrite(1,msg);
	}
	msg[0] = '\0';
	while((RxLen=TCPRxLen(sock))>0){
	  TCPRead(sock,bff,RxLen);
	  strcat(msg,bff);
	}
	if(msg[0] != '\0'){
		char buf[4];
		if(msg[3] != '\0'){
			if(msg[0] == 'O' && msg[1] == 'K'){
				strncpy(msg, msg+2, 14);
			}
			strncpy(buf,msg+2,4);
			sinc.tm_year = atoi(buf);
			strncpy(buf,msg+4,2);
			sinc.tm_mon = atoi(buf);
			strncpy(buf,msg+6,2);
			sinc.tm_mday = atoi(buf);
			strncpy(buf,msg+8,2);
			sinc.tm_hour = atoi(buf);
			strncpy(buf,msg+10,2);
			sinc.tm_min = atoi(buf);
			strncpy(buf,msg+12,2);
			sinc.tm_sec = atoi(buf);
			
			RTCCSet(&sinc);
		}
		sprintf(msg,"%d", sinc.tm_mon);
		UARTWrite(1,msg);
	}
	else{
		sprintf(msg,"Bads\r\n");
		UARTWrite(1,msg);
		return 0;
	}
	return 1;
}
