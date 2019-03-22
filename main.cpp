#include "mbed.h"
#include "SHA256.h"
#include <string>
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

//pid control constants
#define kpr 15
#define kdr 18

#define kpv 30
#define kiv 20
#define ivCAP 50
//note that the product/1000 is the duty cycle

//maximum velocity for using increment encoder
#define MIN_VELOCITY_IE 40
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
int8_t orState = 0x5;    //Rotot offset at motor state 0

//position, rotation, velocity
int motorRotateNum = 0; // number of rotation times 6

int microRotate = 0;
int increTable[] = {0, 3, 1, 2}; //for mapping increment encoder value to state, CHA*2 + CHB
int previousMicroRotateState = 0;


//global variables

//setting key
Mutex key_mutex;
int counter;
char buffer[17];
volatile char cmdChar;
uint64_t newKey;
uint64_t tar_rotation_tmp;
bool new_key = false;
volatile string newchar;

//setting velocity
float setVelocity = 50; //random initial value, range should be 0 - around 120
Mutex velocity_mutex;

float setRotation = 2000;
Mutex rotation_mutex;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Incremental encoder input
InterruptIn CHA(CHApin);
InterruptIn CHB(CHBpin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//PWM output
PwmOut pwm(PWMpin);

//Serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);
//Timer
Timer timer;
Timer rotationTimer;
//Mail
typedef struct {
    char type; //n:nonce h:hashCount m:motor
    uint64_t result;
    int hashCount;
    float velocity;
    float position;
    float dutyCycle;
    
} mail_t;

Mail<mail_t, 16> mail_box;

void putMessage(char type, uint64_t result, int hashCount, float velocity, float position, float dutyCycle){
    mail_t *mail = mail_box.alloc();
    mail->type = type;
    mail->result = result;
    mail->hashCount = hashCount;
    mail->velocity = velocity;
    mail->position = position;
    mail->dutyCycle = dutyCycle;
    mail_box.put(mail);
}

//Queue
Queue<void, 8> inCharQ;
//Threads
Thread out_thread;
Thread decode_thread;
Thread motor_thread(osPriorityNormal,1024);

void sendSerial(){
    pc.baud(9600);
    while(true){
        osEvent evt = mail_box.get();
        if(evt.status == osEventMail){
            mail_t *mail = (mail_t*)evt.value.p;
            switch(mail->type){
                case 'n':
                    pc.printf("Nonce found:");
                    pc.printf("%016llX\n\r", mail->result);
                    break;
                case 'h':
                    pc.printf("Hash count:%d\n\r", mail->hashCount);
                    break;
                case 'm':
                    pc.printf("Velocity:%f\r\n", mail->velocity);
                    pc.printf("Position:%f\r\n", mail->position);
                    //pc.printf("%f,", mail->dutyCycle);
                    //pc.printf("%d\n\r", lead);
                    break;
                default:
                    pc.printf("error\n\r");
                    
            }
            
            mail_box.free(mail);
        }
    }
}

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
    pwm.write(1.0f);
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}
void ISR_turn(){//check current state when the state changes and then drive motor according to the difference
    microRotate = 0;
    intState = readRotorState();
    if (intState != intStateOld) {
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        if(intState - intStateOld == 5) //current 5, previous 0, the sequence is 0->5->4->..... anti-clockwise
            motorRotateNum--;
        else if(intState - intStateOld == -5)//current0, previous 5, 4->5->0->1.... clockwise
            motorRotateNum++;
        else //then the difference can only be +-1
            motorRotateNum += (intState - intStateOld);
        
        intStateOld = intState;
    }    
}

//Incoming communication

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

//Increment Encoder

void ISR_incre(){//use increment encoder to read current rotation in higher precision
    /*
    int increTable[] = {0, 3, 1, 2};    
    */
    int currentState = increTable[CHA*2 + CHB];
    //CHA/B  00 -> 10 -> 11 -> 01 or inversed
    if(currentState - previousMicroRotateState == -3){ //3->0, the sequence is 0->1->2->3->0, anticlockwise
        microRotate++;
    }
    else if(currentState - previousMicroRotateState == 3){//0->3, the sequence is 3->2->1->0->3, clockwise
        microRotate--;
    }
    else{
        microRotate += (currentState - previousMicroRotateState);//other wise the difference can only be +-1
    }
    previousMicroRotateState = currentState;
}


void decodeInput(){
    uint64_t inKey = 0;
    float inVelocity = 0;
    float inRotation = 0;
    pc.attach(&serialISR);
    while(true){
        osEvent evt = inCharQ.get();
    
        if (evt.status == osEventMessage) {
            uint8_t newchar = (uint8_t)evt.value.p;
            if(counter > 17){
                counter = 0;
            }
            else if (newchar!= '\r'){
                buffer[counter] = newchar;
                counter++;
            }
            else if(newchar == '\r'){
                buffer[counter] = '\0';
                 // Reset for the next command
                counter = 0;
            
             
                switch(buffer[0]){
                    case 'K': //Key
                        sscanf(buffer, "K%llx",&inKey); //read 16 hex into inKey
                        key_mutex.lock();
                        newKey = inKey; //assigning global/shared variable, hence mutex
                        key_mutex.unlock();
                        break;
                    
                    case 'V': //Velocity
                        sscanf(buffer, "V%f", &inVelocity); //set target velocity
                        velocity_mutex.lock();
                        setVelocity = inVelocity;
                        velocity_mutex.unlock();
                        break;
                    
                    case 'R':
                        sscanf(buffer, "R%f", &inRotation); //set target position
                        rotation_mutex.lock();
                        setRotation = inRotation;
                        rotation_mutex.unlock();               
                    default:
                        ;          
                }
            }
        }
    }
}

//Motor
void motorCtrlTick(){ //a function called every 100ms to release the thread
    motor_thread.signal_set(0x1);
}



void motorCtrlFn(){
    float previousRotateNum = 0;
    float velocity = 0;
    float timeInterval = 0;
    int iterationCount = 0;
    float currentRotation = 0;
    float velocityError = 0;
    float cumulativeVelocityError = 0;
    float rotationError = 9;
    float previousRotationError = 0;
    float dirivativeRotationError = 0;
    float torqueVelocity = 0;
    float torqueRotation = 0;
    float dutyCycle = 1.0; // has range 0 to 1
    float rotateDutyCycle = 0.0;
    float velocityDutyCycle = 0.0;
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);
    
    pwm.period(0.002f);
    pwm.write(1.0f);
    
    while(true){
        rotationTimer.start();
        motor_thread.signal_wait(0x1); //thread blocked until received signal
        rotationTimer.stop(); 
        timeInterval = rotationTimer.read();//get the actual time interval
        rotationTimer.reset();
        core_util_critical_section_enter();
        currentRotation = (float)motorRotateNum/6.0 + (float)microRotate/1112; //in one rotation, photon interruptor increments 6 times, increment encoder increments 1112 
        core_util_critical_section_exit();
        velocity = (currentRotation - previousRotateNum)/timeInterval;
        previousRotateNum = currentRotation;
        
        //As we calculate velocity here, increment encoder is switched here

        
        //velocity controller
        velocity_mutex.lock();
        velocityError = setVelocity - velocity; //get Es
        velocity_mutex.unlock();
        cumulativeVelocityError += velocityError * timeInterval; //intergral term
        //cap intergral term
        if(cumulativeVelocityError >= ivCAP) cumulativeVelocityError = ivCAP;
        if(cumulativeVelocityError <= -ivCAP) cumulativeVelocityError = -ivCAP;
        torqueVelocity = kpv * velocityError + kiv * cumulativeVelocityError;


        //Position controller
        rotation_mutex.lock();
        rotationError = setRotation - currentRotation;
        rotation_mutex.unlock();
        dirivativeRotationError = (rotationError - previousRotationError)/timeInterval;
        previousRotationError = rotationError;
        torqueRotation = kpr * rotationError + kdr * dirivativeRotationError;
        
        //change sign of torqueVelocity
        
        if(dirivativeRotationError < 0){
            torqueVelocity = -torqueVelocity;
        }
        
        
        if(torqueVelocity >= 0){
            lead = 2;
            velocityDutyCycle = torqueVelocity/1000; 
            velocityDutyCycle = (velocityDutyCycle > 1)?1:velocityDutyCycle;
        }
        else{
            lead = -2;
            velocityDutyCycle = -torqueVelocity/1000;
            velocityDutyCycle = (velocityDutyCycle > 1)?1:velocityDutyCycle;
        }
        
        
        if(torqueRotation >= 0){
            lead = 2;
            rotateDutyCycle = torqueRotation/1000; 
            rotateDutyCycle = (rotateDutyCycle > 1)?1:rotateDutyCycle;
        }
        else{
            lead = -2;
            rotateDutyCycle = -torqueRotation/1000;
            rotateDutyCycle = (rotateDutyCycle > 1)?1:rotateDutyCycle;
        }
        
                
//the PD rotation controller is overdamped very "conservative", which means it tends to stop the motor before it reach the target
//here i use different floor values for differnet rotationError, hence the motor will not get stuck until it is close enough to the target

        if(lead == 2 && setRotation > 0){ //for rotating anticlockwise
            if(rotationError > 0.1){ 
                rotateDutyCycle = (rotateDutyCycle < 0.8)?0.8f:rotateDutyCycle; //from experiments, duty cycle of 0.8 generate just about the same torque as friction
            }
            if(rotationError > setRotation/200){ //set duty cycle to 0.85 to prevent motor get stucked by friction
                rotateDutyCycle = (rotateDutyCycle < 0.85)?0.85f:rotateDutyCycle;
            }
            if(rotationError > setRotation/50){ 
                rotateDutyCycle = (rotateDutyCycle < 0.95)?0.95f:rotateDutyCycle;
            }
        }
        
        if(lead == -2 && setRotation < 0){ //this one is for rotating clockwise
            if(rotationError < -0.1){ 
                rotateDutyCycle = (rotateDutyCycle < 0.8)?0.8f:rotateDutyCycle;
            }
            if(rotationError < setRotation/200){ 
                rotateDutyCycle = (rotateDutyCycle < 0.85)?0.85f:rotateDutyCycle;
            }
            if(rotationError < setRotation/50){ 
                rotateDutyCycle = (rotateDutyCycle < 0.95)?0.95f:rotateDutyCycle;
            }
        }
        
        if(setRotation == 0){ //special case when R = 0
            lead = 2;
            rotateDutyCycle = 1; //jsut let it rotate forever
        }
        
        if(setVelocity == 0){ //special case when V = 0
            velocityDutyCycle = 1; //remove the software limit, let the motor rotate as fast as it can
        }

        dutyCycle = (rotateDutyCycle < velocityDutyCycle)?rotateDutyCycle:velocityDutyCycle; //always choose "conservative"
        pwm.write(dutyCycle);
        iterationCount++;
        if(iterationCount >= 10){ //output velocity and current position every 1 sec
            putMessage('m',NULL,NULL,velocity,currentRotation, dutyCycle);
            iterationCount = 0;
        }
        
    }
}




   
//Main
int main() {
    //Initialise the serial port
    pc.baud(9600);

    //starting threads
    out_thread.start(sendSerial);
    decode_thread.start(decodeInput);
    motor_thread.start(motorCtrlFn);
    

    //Run the motor synchronisation
    orState = motorHome();
    previousMicroRotateState = CHA.read()*2+CHB.read();
 
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    //attach ISR to each pin's rising and falling edge
    I1.rise(&ISR_turn);
    I2.rise(&ISR_turn);
    I3.rise(&ISR_turn);
    I1.fall(&ISR_turn);
    I2.fall(&ISR_turn);
    I3.fall(&ISR_turn);
    
    //attach ISR to each increment encoder's rising and falling edge
    CHA.rise(&ISR_incre);
    CHA.fall(&ISR_incre);
    CHB.rise(&ISR_incre);
    CHB.fall(&ISR_incre);
        
    //declare SHA256 instance
    SHA256 h;
    //256 bits are 64 bytes
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64, 
                          0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,  
                          0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E, 
                          0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 
                          0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20, 
                          0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20, 
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                          
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
        
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    int hashCount = 0;
    timer.start();
    while (1) {
        key_mutex.lock(); 
        *key = newKey;
        key_mutex.unlock();
        (*nonce)++;
        h.computeHash(hash, sequence, 64);
        hashCount++;
        if(hash[0] == hash[1] && hash[0] == 0) //desired hash value
            putMessage('n', *nonce, NULL, NULL, NULL, NULL); //send nonce found
        if(timer.read() > 1){
            putMessage('h', NULL, hashCount, NULL, NULL, NULL); //send how many hash compution is made during 1 sec
            hashCount = 0;
            timer.reset();
        }
    }
}

