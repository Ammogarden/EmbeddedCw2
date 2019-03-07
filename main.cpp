#include "mbed.h"
#include "SHA256.h"
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
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);
//Timer
Timer timer;

//Mail
typedef struct {
    bool isNounce;
    uint64_t result;
    int hashCount;

} mail_t;

Mail<mail_t, 16> mail_box;

void putMessage(bool isNounce, uint64_t result, int count){
    mail_t *mail = mail_box.alloc();
    mail->isNounce = isNounce;
    mail->result = result;
    mail->hashCount = count;
    mail_box.put(mail);
}

//Queue
Queue<void, 8> inCharQ;
//Threads
Thread out_thread;
Thread decode_thread;



void sendSerial(){
    pc.baud(9600);
    while(true){
        osEvent evt = mail_box.get();
        if(evt.status == osEventMail){
            mail_t *mail = (mail_t*)evt.value.p;
            if(mail->isNounce){
                pc.printf("Nonce found:");
                pc.printf("%016llX\n\r", mail->result);
            }else{
                pc.printf("Hash count:%d\n\r", mail->hashCount);
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
    motorOut(0);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}
void ISR_turn(){//check current state when the state changes and then drive motor according to the difference
    intState = readRotorState();
    if (intState != intStateOld) {
        intStateOld = intState;
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        //pc.printf("%d\n\r",intState);
    }
}

//Incoming communication

void ISR_serial(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);

}

float velocity; //motor velocity
float rotation; //motor rotation
float tar_rotation_tmp = 0;
float tar_velocity_tmp = 0;
float tar_key_tmp = 0;
float motor_position;
char buffer[17];

Mutex key_mutex;

void decodeInput(){
    pc.attach(&ISR_serial);
    string cmd;
    osEvent evt = inCharQ.get();
    if (evt.status == osEventMessage) {
        uint8_t *message = (uint8_t*)evt.value.p;
        cmd.append((char)*message);
        if((char)*message = "\r")
        {
             counter = 0;
             buffer[counter] = "\0";
             switch(buffer[0])
             {
               case 'R': //Rotation
               sscanf(buffer, "R%f", &tar_rotation_tmp);
               rotation = ((float)motor_position/6) + tar_rotation_tmp;
               break;

               case 'V': //Velocity
               sscanf(buffer, "V%f", &tar_rotation_tmp);
               velocity = (velocity == 0) ? 500 : velocity;
               break;

               case 'K': //Key
               sscanf(buffer, "K%x", &tar_rotation_tmp);
               tar_key_tmp = key;
               break;

               case 'T'://Tune
               sscanf(buffer, "T%d", &tar_rotation_tmp);
               break;
             }
        }else
        {
          counter++;
        }
    }
}

//Main
int main() {
    //Initialise the serial port
     //TODO: check: should i still establish serial connection in main() given that there is a thread meant to use the serial port?
    pc.baud(9600);
    //pc.printf("Hello\n\r");

    out_thread.start(sendSerial);




    //
    //Run the motor synchronisation
    orState = motorHome();
    //pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    //attach ISR to each pin's rising and falling edge
    I1.rise(&ISR_turn);
    I2.rise(&ISR_turn);
    I3.rise(&ISR_turn);
    I1.fall(&ISR_turn);
    I2.fall(&ISR_turn);
    I3.fall(&ISR_turn);
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
        (*nonce)++;
        h.computeHash(hash, sequence, 64);
        hashCount++;
        if(hash[0] == hash[1] && hash[0] == 0)
            putMessage(true, *nonce, NULL);
            //pc.printf("%016llX\n", *nonce);
        if(timer.read() > 1){
            putMessage(false, NULL, hashCount);
            //pc.printf("hash count:%d\n\r",count);
            hashCount = 0;
            timer.reset();
        }
    }
}
