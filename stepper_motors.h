//STEPPER MOTORS
#define ZERO_SPEED 1000000 //65535
#define MAX_ACCEL 2
#define ENABLE 0
#define DISABLE 1

Ticker timer_M1, timer_M2; //Timers for speed Control

// MOTOR 1
DigitalOut step_M1(MOTOR1_STEP);
DigitalOut dir_M1(MOTOR1_DIR);
DigitalOut enable(MOTOR_ENABLE); //enable for both motors
int16_t speed_M1;  //Speed of motor 1
//Variables for M1 PID control (added)
int16_t target_speed_M1_old;
int16_t speed_M1_old;
float theta_M1;
float Kp_M1, Kd_M1;

//MOTOR 2
DigitalOut step_M2(MOTOR2_STEP);
DigitalOut dir_M2(MOTOR2_DIR);
int16_t speed_M2;  //Speed of motor 2
int16_t motor1, motor2;
//Variables for M1 PID control (added)
int16_t target_speed_M2_old;
int16_t speed_M2_old;
float theta_M2;
float Kp_M2, Kd_M2;

//Motor Position
volatile int pos_M1 = 0;
volatile int pos_M2 = 0;

// =============================================================================
// ===                   Interrupt Service Soutine                           ===
// =============================================================================
//ISR to step motor 1
void ISR1(void)
{
    //Step Motor
    step_M1 = 1;
    wait_us(1);
    step_M1 = 0;
    
    //Update Postion
    if(dir_M1)
        pos_M1++;
    else
        pos_M1--;    
}
//ISR to step motor 2
void ISR2(void)
{
    //Step Motor
    step_M2 = 1;
    wait_us(1);
    step_M2 = 0;
    
    //Update Position
    if(dir_M2)
        pos_M2++;
    else
        pos_M2--;
}

//Set motor 1 speed. Speed [-100, +100]
void setMotor1Speed(int16_t speed)
{
    long timer_period;
    //If the speed has not changed, do not update the timer!
    if(speed_M1 == speed){
     return;   
    }
    /*int16_t output_speed;
    theta_M1 = (float)(speed - speed_M1);
    output_speed = (int16_t) (Kp_M1 * theta_M1 + Kd_M1 * (theta_M1 - (float)(speed_M1_old - target_speed_M1_old)));
    output_speed = CAP(output_speed, MAX_CONTROL_OUTPUT);*/
    //original
    speed = CAP(speed, MAX_CONTROL_OUTPUT);
    
    //Calculate acceleration from the desired speed
    int16_t desired_accel = speed_M1 - speed;
    //int16_t desired_accel = speed_M1 - output_speed;
    if(desired_accel > MAX_ACCEL)
        speed_M1 -= MAX_ACCEL; //Change speed of motor by max acceleration
    else if(desired_accel < -MAX_ACCEL)
        speed_M1 += MAX_ACCEL; //Change speed of motor by max acceleration
    else{
        speed_M1 = speed; 
        //speed_M1 = output_speed;
    }
    
    //pass on values
    /*speed_M1_old = speed_M1;
    target_speed_M1_old = speed;*/

    if(speed_M1 == 0) {
        timer_period = ZERO_SPEED;
        dir_M1 = 0; ////sets motor direction
    } else if (speed_M1 > 0) {
        timer_period = 10000 / speed_M1;
        dir_M1 = 1; //sets motor direction
    } else {
        timer_period = 10000 / -speed_M1;
        dir_M1 = 0; ////sets motor direction
    }

    // Update Timer period
    timer_M1.attach_us(&ISR1, timer_period); //This is what sets motor speed
}

//Set motor 2 speed. Speed [-100, +100]
void setMotor2Speed(int16_t speed)
{
    long timer_period;
    //If the speed has not changed, do not update the timer!
    if(speed_M2 == speed){
     return;   
    }
    /*int16_t output_speed;
    theta_M2 = (float)(speed - speed_M2);
    output_speed = (int16_t) (Kp_M2 * theta_M2 + Kd_M2 * (theta_M2 - (float)(speed_M2_old - target_speed_M2_old)));
    output_speed = CAP(output_speed, MAX_CONTROL_OUTPUT);*/
    //original
    speed = CAP(speed, MAX_CONTROL_OUTPUT);

    //Calculate acceleration from the desired speed
    int16_t desired_accel = speed_M2 - speed;
    if(desired_accel > MAX_ACCEL)
        speed_M2 -= MAX_ACCEL; //Change speed of motor by max acceleration
    else if(desired_accel < -MAX_ACCEL)
        speed_M2 += MAX_ACCEL; //Change speed of motor by max acceleration
    else{
        speed_M2 = speed;
        //speed_M2 = output_speed;
    }
    
    //pass on values
    /*speed_M2_old = speed_M2;
    target_speed_M2_old = speed;*/

    if(speed_M2 == 0) {
        timer_period = ZERO_SPEED;
        dir_M2 = 0; ////sets motor direction
    } else if (speed_M2 > 0) {
        timer_period = 10000 / speed_M2;
        dir_M2 = 1; //sets motor direction
    } else {
        timer_period = 10000 / -speed_M2;
        dir_M2 = 0; ////sets motor direction
    }

    // Update Timer period
    timer_M2.attach_us(&ISR2, timer_period); //This is what sets motor speed
}