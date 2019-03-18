#ifndef MESSAGE_HPP
#define MESSAGE_HPP
#endif

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

void sendSerial(){
    pc.baud(9600);
    while(true){
        osEvent evt = mail_box.get();
        if(evt.status == osEventMail){
            mail_t *mail = (mail_t*)evt.value.p;
            if(mail->isNounce){
                //pc.printf("Nonce found:");
                //pc.printf("%016llX\n\r", mail->result);
            }else{
                //pc.printf("Hash count:%d\n\r", mail->hashCount);
            }
            mail_box.free(mail);
        }
    }
}