#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#ifndef CTRLRECV_H
#define CTRLRECV_H

class CtrlRecv
{
    public:
        // key map to generate key
        static const char* KEYS;
        static const char* CONNECT_REFUSE;
        static const char* CONNECT_ACCEPT;

        CtrlRecv();

        int bindTo();
        int listenOn();
        void standByConnect();
        void standByReceive(void* (*)(void*));

        virtual ~CtrlRecv();
    protected:
    private:
        // listen port
        static const int port = 60000;

        const char delim = '\n';

        int ctrl_socket;
        int client_socket;
        int tmp_socket;
        char* connect_key;
        char tmp_buff[34];

        void _send(char* data, int length){
            printf("Command:%s\n", data);
        }

        char* _generateConnectKey(){
            char* key = (char *)malloc(32);
            for(int i=0;i<32;i++){
                key[i] = KEYS[rand() % 32];
            }
            key[32] = '\n';
            return key;
        }
};

#endif // CTRLRECV_H
