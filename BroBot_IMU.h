//BroBot_IMU.h
// Contains everyting needed to interface with the IMU for BroBot
//Source links: https: developer.mbed.org/users/Sissors/code/MPU6050/docs/5c63e20c50f3/classMPU6050.html
// https://developer.mbed.org/users/paulbartell/code/MPU6050-DMP/file/95449a48c5c0/MPU6050_6Axis_MotionApps20.h

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 `high
// MPU control/status vars
#include "rtos_definations.h"


bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

InterruptIn checkpin(CHECKPIN);

// ================================================================
// ===                       IMU                                ===
// ================================================================
// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
    // INV_KEY_0_96
    mpu.setMemoryBank(0);
    mpu.setMemoryStartAddress(0x60);
    mpu.writeMemoryByte(0);
    mpu.writeMemoryByte(gain);
    mpu.writeMemoryByte(0);
    mpu.writeMemoryByte(0);
}

// Quick calculation to obtein Phi angle from quaternion solution (from DMP internal quaternion solution)
void dmpGetReadings(float * angle, float *theta)
{
    mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.resetFIFO();  // We always reset FIFO

    //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
    //return Phi angle (robot orientation) from quaternion DMP output
    *angle = (atan2(2 * ((q.y * q.z) + (q.w * q.x)), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
    *theta = (atan2(2 * (q.y * q.x + q.z * q.w),1 - 2 * (q.w * q.w + q.x * q.x)) * RAD2GRAD); 
    return;
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
Serial pc1(USBTX, USBRX);
void dmpDataReady()
{
    mpuInterrupt = true;
    osSignalSet(imu_update_thread_ID,IMU_UPDATE_SIGNAL);
}