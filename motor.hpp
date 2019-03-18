#ifndef MOTOR_HPP
#define MOTOR_HPP
#endif
#include <math.h>

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0


//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//motor's state, we made it global but is there a bette way?
int8_t intState = 0;
int8_t intStateOld = 0;
int32_t motorPos = 0;

//Rotot offset at motor state 0
//We have found that our initial state is when I1=1, I2=0, I3=0, which corresponds to 0x5
int8_t orState = 0x5;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//PWM control for D9
PwmOut pwmControl(D9);

//Thread executed every 100ms
//normal priority, 1024 max stack size during execution
Thread motorCtrlT(osPriorityNormal, 1024);

//Timer for calculating velocity
Timer vTimer;

//Velocity control constants
#define KPS 250
#define KIS 2

//Rotation control constants
#define KDR 20
#define KPR 2

//Emperically measured max and min rotation speed
#define MAX_ROTSPEED 135
#define MIN_ROTSPEED 10

//Output torque
float sTorque;
float rTorque;
float torque;

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}
    
//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

void ISR_turn(){//check current state when the state changes and then drive motor according to the difference

    intState = readRotorState();
    if (intState != intStateOld) {

        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        if(intState - intStateOld == -5) //current state is 0, previous state is 5, so the sequence is 0->1->...->5->0, clockwise rotation
            motorPos++;
        else if(intState - intStateOld == 5)//current 5, previous 0, 5->4->...->0->5, anticlockwise rotation
            motorPos--;
        else
            motorPos += intState - intStateOld; //for other case, the difference can only be 1(clockwise) or -1(anticlockwise)
        intStateOld = intState;
        
    }    
}

//send back a signal 1 when triggered
//lower priority than ISR, so no computation to avoid blocking CPU
void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}

void motorControlFn(){
    
    //motorControlTick runs every 100000us = 100ms
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);
    
    //Calculate velocity control
    float velocity;
    float velocityErr = 0;
    float dTime;
    
    //Calculate rotation control
    float rotErr;
    float oldRotErr;
    float dRotErr;
        
    int i = 0;
    int oldMotorPos = 0;
    float pwmCycle;
    
    while(1){
        
        //wait for signal 0x1 to be received, then automatically clear the signal
        //so the loop will run only once per trigger
        vTimer.start();
        motorCtrlT.signal_wait(0x1);
        dTime = vTimer.read();
        vTimer.reset();
        
        //Use a critical region to block access to the variable from other routines
        //Velocity is in rotation/sec
        core_util_critical_section_enter();
        velocity = (motorPos - oldMotorPos) / dTime / 6;
        core_util_critical_section_exit();
        
        //Calculate velocity error and speed controller output
        velocityErr = inVelocity - abs(velocity);
        sTorque = KPS * velocityErr;
        
        if(sTorque < 0) 
            lead = -2;
        else 
            lead = 2;
            
        if(sTorque > MAX_ROTSPEED)
            sTorque = MAX_ROTSPEED;
        else if(sTorque < MIN_ROTSPEED)
            sTorque = MIN_ROTSPEED;
            
        pwmCycle = sTorque / MAX_ROTSPEED;
        
            
        pwmControl.write(pwmCycle);
     /*       
        //Calculate rotation error and rotation controller output
        rotErr = inRotation - motorPos / 6;
        dRotErr = (rotErr - oldRotErr) / dTime;
        rTorque = KPR * rotErr + KDR * dRotErr;
        
        //Choose output torque
        if(velocity >= 0)
            torque = min(rTorque, sTorque);
        else
            torque = max(rTorque, sTorque);
       */     

        
        if(++i == 10){
            pc.printf("Velocity: %f\n\r",velocity);
            pc.printf("Motor Position: %d\n\r",motorPos);
            i = 0;
        }
        
        oldRotErr = rotErr;
        oldMotorPos = motorPos;
        
        

    }
}


    