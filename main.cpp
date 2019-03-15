#include "mbed.h"
#include "SHA256.h"
#include "motor.hpp"
#include "message.hpp"
#include "command.hpp"

//Timer
Timer timer;

//Threads for output and decode
Thread out_thread;
Thread decode_thread;

PwmOut pwmControl(D9);
   
//Main
int main() {
    
    pwmControl.period_us(2);
    pwmControl.write(0.5f);
    
    
    //Initialise the serial port
     //TODO: check: should i still establish serial connection in main() given that there is a thread meant to use the serial port?
    pc.baud(9600);
    //pc.printf("Hello\n\r");
    
    out_thread.start(sendSerial);
    decode_thread.start(decodeInput);
    motorCtrlT.start(motorCtrlFn);
    
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
        
        key_mutex.lock(); 
        *key = newKey;
        
        //putMessage(true, *key, NULL);
        key_mutex.unlock();
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