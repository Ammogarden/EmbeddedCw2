#ifndef COMMAND_HPP
#define COMMAND_HPP
#endif

#include <string>

//Serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Queue
Queue<void, 8> inCharQ;

//Simultaneous access prevention
Mutex key_mutex;

//character array
int counter;
char buffer[17];

volatile char cmdChar;
volatile uint64_t newKey;
volatile float inRotation;
volatile float inVelocity;

bool new_key = false;
volatile string newchar;

//Incoming communication
void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

void decodeInput(){
    uint64_t inKey = 0;
    pc.attach(&serialISR);
    while(true){
        osEvent evt = inCharQ.get();
    
        if (evt.status == osEventMessage) {
            uint8_t newchar = (uint8_t)evt.value.p;
            if(counter > 17){
                counter = 0;
                //buffer[counter] = newchar;
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
                        key_mutex.lock();
                        sscanf(buffer, "K%llx",&inKey); //read 16 hex into inKey
                        newKey = inKey; //assigning global/shared variable, hence mutex
                        key_mutex.unlock();
                        break;
                                  
                    case 'V': //Set velocity
                        sscanf(buffer, "V%f", &inVelocity);
                        break;
                        
                    case 'R': //Set rotation, in revolutions(motorPos/6), not in single motor position
                        sscanf(buffer, "R%f", &inRotation);
                        break;
                          
                    default:
                        ;          
                }
            }
        }
        else{
            //counter++;
            ;
       }
    }
}