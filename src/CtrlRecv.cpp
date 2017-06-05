#include "CtrlRecv.h"

const char* CtrlRecv::KEYS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
const char* CtrlRecv::CONNECT_REFUSE = "REFUSE\n";
const char* CtrlRecv::CONNECT_ACCEPT = "ACCEPT\n";

CtrlRecv::CtrlRecv()
{
    ctrl_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(ctrl_socket == -1){
        throw "Error create CtrlRecv socket";
    }

    connect_key = _generateConnectKey();
}

int CtrlRecv::bindTo(){
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    int ret = bind(ctrl_socket, (sockaddr *) &addr, sizeof(addr));
    if(ret == -1){
        return -1;
    }

    return 0;
}

int CtrlRecv::listenOn(){
    int ret = listen(ctrl_socket, 1);
    if(ret == -1){
        return -1;
    }

    return 0;
}

void CtrlRecv::standByConnect(){
    while(1){
        tmp_socket = accept(ctrl_socket, NULL, NULL);
        if(tmp_socket == -1){
            continue;
        } else {
            send(tmp_socket, (void *)connect_key, strlen(connect_key), 0);
            recv(tmp_socket, tmp_buff, 33, 0);
            tmp_buff[33] = '\0';
            printf("recv %s\n", tmp_buff);
            printf("compare %d\n", strcmp(tmp_buff, connect_key));

            if(strcmp(tmp_buff, connect_key) == 0){
                send(tmp_socket, (void *)CONNECT_ACCEPT, strlen(CONNECT_ACCEPT), 0);
                close(client_socket);
                client_socket = tmp_socket;
            }else{
                send(tmp_socket, (void *)CONNECT_REFUSE, strlen(CONNECT_REFUSE), 0);
                close(tmp_socket);
            }
        }
    }
}

void CtrlRecv::standByReceive(void* (*callback)(void*)){
    int bytes, i;
    int command_length = 0;

    char buff[32]; // buffer use to receive socket data
    char* command = (char *) alloca(1024);

    while(1){
        memset(buff, 0, 32);
        bytes = recv(client_socket, buff, 32, 0);
        if(bytes == 0 || bytes == -1){
            sleep(1);
            continue;
        }

        for(i=0;i<bytes;i++){
            // if get \n, send data
            if(buff[i] == '\n'){
                callback(command);
                command_length = 0;
                memset(command, 0, 1024);
                continue;
            }
            command[command_length] = buff[i];
            command_length++;
        }
    }
}

CtrlRecv::~CtrlRecv(){
    close(client_socket);
    close(ctrl_socket);
}
