#ifndef _RTOS_DEFINATIONS_H
#define _RTOS_DEFINATIONS_H

#include "mbed.h"
#include "pin_assignments.h"

//*********** Thread Definations BEGIN ***********//
//EVENT Signals 
#define IMU_UPDATE_SIGNAL 0x04
#define PID_UPDATE_SIGNAL 0x05
#define COMMUNIATION_UPDATE_SIGNAL 0x06

//GPIO Pins
DigitalOut pin_30(DEBUG_PIN1); //Yellow Channel
DigitalOut pin_5(DEBUG_PIN2); //Pink Channel
DigitalOut pin_6(DEBUG_PIN3); //Green


void imu_update_thread(void const *args);
osThreadId imu_update_thread_ID;
osThreadDef(imu_update_thread, osPriorityHigh, DEFAULT_STACK_SIZE);

void pid_update_thread(void const *args);
osThreadId pid_update_thread_ID;
osThreadDef(pid_update_thread, osPriorityAboveNormal, DEFAULT_STACK_SIZE);

void communication_update_thread(void const *args);
osThreadId communication_update_thread_ID;
osThreadDef(communication_update_thread, osPriorityNormal, DEFAULT_STACK_SIZE);


//*********** Thread Definations END *************//
#endif