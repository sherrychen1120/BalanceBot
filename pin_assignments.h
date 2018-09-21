//pin_assignments.h

//HARDWARE
//The main processor is an mbed LPC1768
//There are two motor drivers A4988(1) and A4988(2)
//There is one 6 axis IMU MP6050
//There is one RF transciever MRF24J40

//Pin Declarations

//Motor Driver Pins
#define MOTOR1_STEP p19
#define MOTOR1_DIR p20
#define MOTOR2_STEP p15
#define MOTOR2_DIR p17
#define MOTOR_ENABLE p18  //For both motor divers 

//MPU6050 SPI PINS
#define I2C_SDA_1 p28
#define I2C_SCL_1 p27
#define CHECKPIN p29  //for IMU interupt pin (INT)

//MRF24J40
#define SDI p11
#define SDO p12
#define SCK p13
#define CS p7
#define RESET p8

#define DEBUG_PIN1 p30
#define DEBUG_PIN2 p5
#define DEBUG_PIN3 p6
