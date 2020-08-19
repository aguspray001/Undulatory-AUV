/***************************************************************
Tugas Akhir : Mobile Robot Sebagai Landing Pad Drone
Version 		: 1
Date    		: 5/16/2020
Author  		: -
Company 		: Meka 16 PENS

Code    		: Deklarasi variabel yang digunakan
*****************************************************************/

#ifndef _Global_Variable_
#define _Global_Variable_

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

#define PI 3.14159265

//==================================Rotary Encoder Motor=================================================
extern int 			pulse_ms1, pulse_ms2, pulse_ms3;
extern int 			RPS1, RPS2, RPS3;
extern uint16_t lastTick1, lastTick2, lastTick3;
extern int 			dCount1, dCount2, dCount3;
//=======================================================================================================

//=====================================PID Kecepatan Motor===============================================
extern int 				error1, error2, error3;
extern float	 		setpoint1, setpoint2, setpoint3;
extern float 			kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3;
extern int 				Vpid1,  Vpid2, Vpid3;
extern int				lastVpid1, lastVpid2, lastVpid3;
//=======================================================================================================

//======================================Variable PID posisi==============================================
//extern float integral;
//extern float derivative;
//extern float Kp, Ki, Kd;
extern float Output;
//extern float sum_error;
//extern float sum_error_h;
//extern float last_error;
//extern float error;
extern float vx, vy, vt;
extern float error_h;
extern float Output_h;
extern float Kp_h, Ki_h,Kd_h;
extern float last_error_h;
extern float derivative_h, integral_h;
//=======================================================================================================
/*
extern float anu, lastAnu, hitungX, hitungY, lastX, lastY, equation;
extern int hit_x[100];
extern int flag;
*/
//===========================================Variable Motion=============================================
extern int map_posy,map_posx, map_post;
extern float xdist, ydist, tdist;
extern double xdist1, ydist1, tdist1, distance, angle, beta, alfa; 
extern double x_circ, y_circ;
//=======================================================================================================

//====================================ODOMETRY===================================
extern long		posX,posY,Teta;
/*
extern int 		kecX,kecY;
extern float 	posisiX, posisiY;
extern float 	VX, VY, VT;
extern float 	VR1, VR2, VR3;
extern float 	dis1, dis2, dis3; 
extern int 		Count1, Count2, Count3;
extern int 		kalibrasiX,kalibrasiY;
extern float 	posx,posy;
extern int 		hitung;
extern int		delta1, delta2, delta3;
*/
extern int bacaEncoder1(int now);
extern int bacaEncoder2(int now);
extern int bacaEncoder3(int now);

//extern void OdometryReset1(void);
//extern void OdometryKalibrasi(void);
extern void OdometryReset(void);
extern void Odometry(void);
//extern void resetOdo(int data1);
//===============================================================================

//================================IMU============================================
extern float dataYaw,dataPitch,dataRoll;
//extern float dataYawAsli;
extern float calibYaw;
//extern unsigned char flagRepYaw;
//extern int manualyaw;
//extern char flagmanualyaw;

extern void InitSerialIMU_DMA(void);
extern void Parsing_IMU(void);
//extern void resetBufferIMU(void);
extern void resetIMU(void);
//===============================================================================
extern void dwmseribu(void);
extern int head, dwmx, dwmy;
extern char buff[100];
//=================================motor=========================================
extern void motor1(void);
extern void motor2(void);
extern void motor3(void);
extern void PID1(void);
extern void PID2(void);
extern void PID3(void);
extern int 	readEncoder1(int now);
extern int 	readEncoder2(int now);
extern int 	readEncoder3(int now);
extern void rotary1(void);
extern void rotary2(void);
extern void rotary3(void);
//================================================================================

//=================================Motion=========================================
extern void pid_target(long x,long y, long t);
extern void kinematic(float vx, float vy, float vt);
extern void motion(void);
//================================================================================

extern int cek1, cek2, cek3;

//=====================================ToF========================================
extern float tof1, tof2, tof3, tof4, tof5, tof6, tof7, tof8, tof9, tof10, tof11, tof12;
extern long newAvg_x, newAvg_y;
extern long vec_obs_x, vec_obs_y;
extern long vec_dir_x, vec_dir_y;
extern void obstacle(void);
//================================================================================

//====================================SPLINE======================================
extern void SPLINE();
//================================================================================

#endif
