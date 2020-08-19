#include "GlobalVariable.h"

//imu
unsigned char Re_buf[8],counter=0;
float 				dataYaw,dataPitch,dataRoll;
float 				dataYawAsli;
//char 					flagReset=0;
float 				calibYaw;
//unsigned char flagRepYaw;
//int 					manualyaw;
//char 					flagmanualyaw;
uint8_t 			DataReceivedIMU[8]={0,0,0,0,0,0};//REceive ONly 8 bit

extern UART_HandleTypeDef huart1;

//============================================IMU====================================================================
void InitSerialIMU_DMA(void){
	HAL_UART_Receive_DMA(&huart1, DataReceivedIMU,sizeof(DataReceivedIMU));
}

void resetIMU(void){
	uint8_t reset[2]={0xA5,0x55};
	uint8_t reset1[2]={0xA5,0x54};
	uint8_t reset2[2]={0xA5,0x52};
	HAL_UART_DMAStop(&huart1);
	HAL_Delay(50);
	HAL_UART_Transmit(&huart1,(uint8_t *)reset,sizeof(reset),100);
	HAL_Delay(170);
	HAL_UART_Transmit(&huart1,(uint8_t *)reset1,sizeof(reset1),100);
	HAL_Delay(170);
	HAL_UART_Transmit(&huart1,(uint8_t *)reset2,sizeof(reset2),100);
	InitSerialIMU_DMA();
}

void resetBufferIMU(void){
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET)  {
   HAL_UART_DMAStop(&huart1);
   HAL_UART_Receive_DMA(&huart1,DataReceivedIMU,sizeof(DataReceivedIMU)); //ReInisisasi 
		}	

}

void Parsing_IMU(void){
	
	HAL_UART_Receive_DMA(&huart1,DataReceivedIMU,sizeof(DataReceivedIMU));
	float YPR[3];
	if(DataReceivedIMU[0] == 0xAA && DataReceivedIMU[7] == 0x55)
		{
			YPR[0] = (DataReceivedIMU[1] << 8 | DataReceivedIMU[2])*0.01f;	//YAW
			YPR[1] = (DataReceivedIMU[3] << 8 | DataReceivedIMU[4])*0.01f;	//PITCH
			YPR[2] = (DataReceivedIMU[5] << 8 | DataReceivedIMU[6])*0.01f;	//ROLL
			
			//Setelah nilai IMU 179 lalu melompat ke 475
			//range sudut 0-179 lalu 475-655 
			if (YPR[0] >179)		{dataYaw = dataYawAsli = (655-YPR[0]);
																if(YPR[0]>=475){dataYaw = dataYawAsli = -dataYaw;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataYaw = dataYawAsli = YPR[0]; 
			
			if (YPR[1] > 179)		{dataPitch = (655-YPR[1]);
															if(YPR[1]>475){dataPitch=-dataPitch;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataPitch = YPR[1];
			
			if (YPR[2] > 179)		{dataRoll = (655-YPR[2]);
																if(YPR[2]>475){dataRoll=-dataRoll;} //RANGE 0 <-> 180 || -179 <-> -1
													}
			else									dataRoll = YPR[2];
	
	}
		
	if(dataYawAsli<0) {dataYawAsli+=360;}
	
	if(dataYaw<0) {dataYaw+=360;}
	if(dataPitch<0) dataPitch += 360;
	if(dataRoll<0) dataRoll += 360;

}
//====================================================END OF IMU===============================================
