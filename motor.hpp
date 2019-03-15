#ifndef MOTOR_HPP
#define MOTOR_HPP
#endif

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
const int8_t lead = 2;  //2 for forwards, -2 for backwards

//motor's state, we made it global but is there a bette way?
int8_t intState = 0;
int8_t intStateOld = 0;
int8_t orState = 0;    //Rotot offset at motor state 0

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

volatile float newTorque;
int32_t motorPos;
Thread motorCtrlT(osPriorityNormal, 1024);

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(newTorque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(newTorque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(newTorque);
    if (driveOut & 0x20) L3H = 0;
    
    pc.printf("%9.4f", newTorque);
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
        if(intState - intStateOld == 5) motorPos--;
        else if(intState - intStateOld == -5) motorPos++;
        else motorPos += intState - intStateOld;
        
        intStateOld = intState;
        //pc.printf("%d\n\r",intState);
    }    
}

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}

int curRotation;

void motorCtrlFn(){
    
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);
    
    Timer t;
    t.start();
    
    int oldRotation = curRotation;
    float timeDif;
    float speed;
    
    while(1){
        
        motorCtrlT.signal_wait(0x1);
        
        timeDif = t.read();
        t.reset();
        
        speed = 1.0f * (curRotation - oldRotation) / timeDif;
        pc.printf("%9.6f", speed);
        //do stuff
    }
}