/******************************* README USAGE *******************************
* This robot must be powered on while it is laying down flat on a still table
* This allows the robot to calibrate the IMU (~5 seconds)
* The motors are DISABLED when the robot tilts more then +-45 degrees from
* vertical.  To ENABLE the motors, lift the robot to 0 degres and
* press the joystick button.
* To reset the motor positions, press the josystick button anytime.
******************************************************************************/

//Controller Values
float knob1, knob2, knob3, knob4;
float jstick_h, jstick_v;

//Button
bool button;

//BroBot Begin
#include "cmsis_os.h"
#include "rtos_definations.h"
#include "pin_assignments.h"
#include "I2Cdev.h"
#include "JJ_MPU6050_DMP_6Axis.h"
#include "BroBot.h"
#include "BroBot_IMU.h"
#include "stepper_motors.h"
#include "MRF24J40.h"
#include "communication.h"

//Angle Offset is used to set the natural balance point of the robot.
#define ANGLE_OFFSET 99
#define THETA_OFFSET 0

#define MRF_CHANNEL 11

//Knobs
#define POT1 p17
#define POT2 p18
#define POT3 p16
#define POT4 p15
//JoyStick
#define POTV p19
#define POTH p20

//PID
#define MAX_THROTTLE 580
#define MAX_STEERING 150
#define MAX_TARGET_ANGLE 12     

//Joystick Control
#define THROTTLE_DAMPNER 10
#define STREEING_DAMPNER 10

//*********** Local Function Definations BEGIN **************//
void init_system();
void init_imu();
//*********** Local Function Definations END **************//

Timer timer;
int timer_value = 0;
int timer_old = 0;

//balance control variables
float angle = 0;
float theta = 0;
float totalTheta = 0;
float deltaTheta = 0;
float integralTheta = 0;

//position control variables
float target_pos = 0;
float pos_error = 0;
float delta_pos_error = 0;

Serial pc(USBTX, USBRX);

// LEDs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

//Used to set angle upon startup
bool initilizeAngle;
bool initilizeTheta;

// ================================================================
// ===                      IMU Thread                          ===
// ================================================================
void imu_update_thread(void const *args)
{   
    float dAngle = 0;
    float dTheta = 0;
    float new_angle = 0;
    float new_theta = 0;
    long long timeOut = 0;
    
    pc.printf("Starting IMU Thread..\r\n");
    while (1) {
        osSignalWait(IMU_UPDATE_SIGNAL,osWaitForever);
        if(mpuInterrupt) {
            mpuInterrupt = false;
            led3 = !led3;
            /********************* IMU Handling *****************/
            mpuIntStatus = mpu.getIntStatus();

            //get current FIFO count
            fifoCount = mpu.getFIFOCount();

            // check for overflow (this should never happen unless our code is too inefficient)
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                pc.printf("MPU FIFO overflow!");
                //otherwise, check for DMP data ready interrupt (this should happen frequently)
            } else if (mpuIntStatus & 0x02) {
                //wait for correct available data length, should be a VERY short wait
                timeOut = 0;
                while (fifoCount < packetSize) {
                    timeOut++;
                    fifoCount = mpu.getFIFOCount();
                    if(timeOut > 100000000){
                        break;
                    }
                }

                //read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);

                //track FIFO count here in case there is > 1 packet available
                //(this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;

                //Read new angle from IMU
                dmpGetReadings(&new_angle,&new_theta);
                //pc.printf("read_in angle: %f \r\n", new_angle);//debug
                new_angle = (float)(new_angle - ANGLE_OFFSET);
                new_theta = float(new_theta + THETA_OFFSET);
                
                //pc.printf("our own print statement: new_angle: %f; old angle: %f. \r\n", new_angle, angle);
                
                //Calculate the delta angle
                dAngle = new_angle - angle;
                dTheta = new_theta - theta;

                //Filter out angle readings larger then MAX_ANGLE_DELTA
                if(initilizeAngle) {
                    angle = new_angle;
                    initilizeAngle = false;
                    pc.printf("\t\t\t Setting Initial Value: %f\r\n",angle);
                } else if( ((dAngle < 15) && (dAngle > -15))) {
                    angle = new_angle;
                    theta = new_theta;
                } else {
                    pc.printf("\t\t\t Delta Angle too Large: %f. Ignored \r\n",dAngle);
                    pc.printf("\t\t\t New Angle: %f Old Angle: %f\r\n",new_angle,angle);
                }
                
                if(initilizeTheta) {
                    theta = new_theta;
                    initilizeTheta = false;
                } else if( ((dTheta < 15) && (dTheta > -15))) {
                    theta = new_theta;
                } else {
                    //pc.printf("\t\t\t Delta Theta too Large: %f. Ignored \r\n",dTheta);
                    //pc.printf("\t\t\t New Theta: %f Old Theta: %f\r\n",new_theta,theta);
                    theta = new_theta;
                    dTheta = dTheta > 0 ? -1 : 1;
                    dTheta = 1;
                }
                 totalTheta += dTheta;
            }
            else{
                pc.printf("\t\t\t IMU Error. MPU Status: %d!!\r\n",mpuIntStatus);
            }
            /********************* IMU Handling *****************/
        }//End of if(mpuInterrupt) loop
        osSignalClear(imu_update_thread_ID,IMU_UPDATE_SIGNAL);
        osSignalSet(pid_update_thread_ID,PID_UPDATE_SIGNAL);
    }
}

// ================================================================
// ===                      PID Thread                          ===
// ================================================================
void pid_update_thread(void const *args)
{
    float Kp1 = 0;
    float Ki1 = 0;
    float Kd1 = 0;
    float Kp2 = 0;
    float Kd2 = 0;
    float target_angle = 0;
    float target_angle_old1 = 0;
    float target_angle_old2 = 0;
    float target_angle_old3 = 0;
    float change_in_target_angle = 0;
    float change_in_angle = 0;
    float angle_old1 = 0;
    float angle_old2 = 0;
    float angle_old3 = 0;
    float kp_term = 0;
    float kd_term = 0;
    float throttle = 0;
    float steering = 0;
    float control_output = 0;
    float pos_control_output = 0;
    float robot_pos = 0;
    float robot_pos_old1 = 0;
    float robot_pos_old2 = 0;
    float robot_pos_old3 = 0;
    float target_pos_old1 = 0;
    float target_pos_old2 = 0;
    //int loop_counter = 0;
    int dt = 0;
    int dt_old1 = 0;
    int dt_old2 = 0;
    float error = 0;
    bool fallen = true;
    
    // --- For Position controller  --- //
    float kp_pos_term = 0;
    float kd_pos_term = 0;
    float change_in_target_pos = 0;
    
    pc.printf("Starting PID Thread..\r\n");
    while(1) {
         osSignalWait(PID_UPDATE_SIGNAL,osWaitForever);
          
        //Button Press on the remote initilizes the robot position.
        if(button) {
            pos_M1 = 0;
            pos_M2 = 0;
            target_pos = 0;
            motor1 = 0;
            motor2 = 0;
            control_output = 0;
            fallen = false;
        }
        
        //Get the time stamp as soon as it enters the loop
        timer_value = timer.read_us();
        led4 = !fallen;
        led2 = button;
        
        //These are balance & position gains
        //ts: Tuesday 2:53am Sent| jstick_h: 0 jstick_v: 0 knob1: 3.15 knob2: 998.78 knob3: 0.0055 knob4: 157.7534 button: 0
        /*Kp1 = 3.15;
        Kd1 = 998.78;
        Kp2 = 0.0055;
        Kd2 = 157.7534;*/
        
        //Set Gains With Knobs
        Kp1 = ((float)knob1);
        Kd1 = ((float)knob2);
        Kp2 = ((float)knob3);
        Kd2 = ((float)knob4);
        
        //These are motor speed gains
        /*Kp_M1 = ((float)knob1);
        Kd_M1 = ((float)knob2);
        Kp_M2 = ((float)knob3);
        Kd_M2 = ((float)knob4);*/
        
        //DEBUG
        //pc.printf("Kp1: %f; Kd1: %f; Kp2: %f; Kd2: %f \r\n", Kp1, Kd1, Kp2, Kd2);
        //pc.printf("button: %d; pos_M1: %d; pos_M2: %d. \r\n", button, pos_M1, pos_M2);

        //Joystick Control
        throttle = (float)jstick_v / THROTTLE_DAMPNER;
        steering = (float)jstick_h / STREEING_DAMPNER;

        //Calculate the delta time
        dt = (timer_value - timer_old);
        timer_old = timer_value;
        
        // STANDING: Motor Control Enabled
        //******************** PID Control BEGIN ********************** //
        if(((angle < 45) && (angle > -45)) && (fallen == false)) {

            //Robot Position
            robot_pos = ((float)(pos_M1 + pos_M2)) / 2.0;
            
            //Target Position Incremented with Joystick
            target_pos += throttle / 2.0;
            /*target_pos = cap(target_pos, target_pos_old + 3);
            target_pos_old = target_pos; */
            
            /***************** Position Controller *********************/
            //Inputs: robot_position, target_pos 
            //Error: distance from target position 
            //Output: target_angle
            // You know this is working when you can push your bot and it returns back
            // to its original location
            
            //note: going in the front direction is negative control output. Going in the rear direction is positive control output
            //If the robot position is positive (to the front of origin), position error is negative, we want pos_control_output to be positive (go towards rear).
            
            pos_error = target_pos - robot_pos;
            //damping the error to prevent it from overshooting
            /*if ((pos_error < -500) || (pos_error > 350)){
                pos_error = pos_error / 1.3;
            }*/
            delta_pos_error = pos_error - (target_pos_old1 - robot_pos_old1);
            pos_control_output = pos_error * Kp2 + (delta_pos_error/(float) dt) * Kd2;  // combine P and D terms together here
            pos_control_output = CAP(pos_control_output, 100); //Cap the values
            
            //This Control Output Will Feed Into The Stability Controller
            target_angle = pos_control_output;
         

            /************ PD Stability CONTROLLER ****************/
            //Inputs: angle, target_angle
            //Error: distance from target_angle
            //Output: control_output (motor speed) 

            theta = target_angle - angle;
            deltaTheta = theta - (target_angle_old2 - angle_old2);
            //integralTheta += theta * dt;
            control_output = theta * Kp1 + (deltaTheta / (float) (dt + dt_old1)) * Kd1; // combine P and D terms together here
            //deltaTheta = theta - (target_angle_old1 - angle_old1);
            //control_output = theta * Kp1 + (deltaTheta / (float) dt) * Kd1; // combine P and D terms together here
            control_output = CAP(control_output, MAX_CONTROL_OUTPUT); // Limit max output from control
            pc.printf("target_pos: %f; pos_M1: %d; pos_M2: %d; robot_pos: %f; pos_error: %f; control_output: %f \r\n", target_pos, pos_M1, pos_M2, robot_pos, pos_error, control_output);

            //*************** Set Motor Speed *************************
            motor1 = (int16_t)(control_output + (steering / 5.0));
            motor2 = (int16_t)(control_output - (steering / 5.0));
            motor1 = CAP(motor1, MAX_CONTROL_OUTPUT);
            motor2 = CAP(motor2, MAX_CONTROL_OUTPUT);
            setMotor1Speed(-motor1);
            setMotor2Speed(-motor2);

            //Update variables
            target_angle_old3 = target_angle_old2;
            target_angle_old2 = target_angle_old1;
            target_angle_old1 = target_angle;
            angle_old3 = angle_old2;
            angle_old2 = angle_old1;
            angle_old1 = angle;
            dt_old2 = dt_old1;
            dt_old1 = dt;
            
            target_pos_old1 = target_pos;
            robot_pos_old3 = robot_pos_old2;
            robot_pos_old2 = robot_pos_old1;
            robot_pos_old1 = robot_pos;

            //pc.printf("m1: %d m2: %d angle: %0.1f, controlout: %f tAngle: %f dt: %f timer: %d \r\n", motor1, motor2, angle, control_output, target_angle, dt, timer_value);
            
            //Enable Motors 
            enable = ENABLE;
            
            if(abs(motor1) == MAX_CONTROL_OUTPUT || abs(motor2) == MAX_CONTROL_OUTPUT) {
                pc.printf("Max Speed Reached. Killing the Robot\n");
                enable = DISABLE;
                fallen = true;
            }
        } else { //[FALLEN}
            //Disable Motors
            enable = DISABLE;

            //Set fallen flag
            fallen = true;
        }
        osSignalClear(pid_update_thread_ID,PID_UPDATE_SIGNAL);
    } //end main loop
}

// ================================================================
// ===                Communication Thread                      ===
// ================================================================
void communication_update_thread(void const *args)
{
    pc.printf("Starting Communication Thread..\r\n");
    while(1) {
        //Recieve Data
        rxLen = rf_receive(rxBuffer, 128);
        if(rxLen > 0) {
            led1 = led1^1;
            //Process data with our protocal
            communication_protocal(rxLen);
            
            //DEBUG
            //pc.printf("knob1: %f; knob2: %f; knob3: %f; knob4: %f \r\n", knob1, knob2, knob3, knob4);
        }
    }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void init_imu()
{
    pc.printf("Start IMU Initilization.. \r\n");

    // Manual MPU initialization... accel=2G, gyro=2000º/s, filter=20Hz BW, output=200Hz
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setDLPFMode(MPU6050_DLPF_BW_10);  //10,20,42,98,188  // Default factor for BROBOT:10
    mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz [4=200hz]default
    mpu.setSleepEnabled(false);
    wait_ms(500);

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    if(devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
    } else {
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        pc.printf("DMP INIT error \r\n");
    }

    //Gyro Calibration
    wait_ms(500);
    pc.printf("Gyro calibration!!  Dont move the robot in 1 second... \r\n");
    wait_ms(500);

    // verify connection
    pc.printf(mpu.testConnection() ? "Connection Good \r\n" : "Connection Failed\r\n");

    //Adjust Sensor Fusion Gain
    dmpSetSensorFusionAccelGain(0x20);

    wait_ms(200);
    mpu.resetFIFO();
}

void init_system()
{
    initilizeAngle = true;
    initilizeTheta = true;
    totalTheta = 0;
    target_pos = 0;
    pos_error = 0;
    //Set the Channel. 0 is default, 15 is max
    mrf.SetChannel(MRF_CHANNEL);
    enable = DISABLE; //Disable Motors
    pc.baud(115200);
    pc.printf("Booting BroBot mbed RTOS..\r\n");

    //Initilize the IMU
    init_imu();
    //Attach Interupt for IMU on rising edge of checkpin
    checkpin.rise(&dmpDataReady);
    pc.printf("IMU Initilized..\r\n");

    //Init Stepper Motors
    //Attach Motor Control Timer Call Back Functions
    timer_M1.attach_us(&ISR1, ZERO_SPEED);//1s Period
    timer_M2.attach_us(&ISR2, ZERO_SPEED);//1s Period
    step_M1 = 1;
    dir_M1 = 1;
    pc.printf("Motors Initilized..\r\n");

    //Timers initilized
    timer.start();
    timer_value = timer.read_us();

    enable = ENABLE; //Enable Motors
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int main()
{
    init_system();

    //Create IMU Thread
    imu_update_thread_ID = osThreadCreate(osThread(imu_update_thread), NULL);

    //Create PID Thread
    pid_update_thread_ID = osThreadCreate(osThread(pid_update_thread), NULL);

    //Create Communication Thread
    communication_update_thread_ID = osThreadCreate(osThread(communication_update_thread), NULL);

} //End Main()