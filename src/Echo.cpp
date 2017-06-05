#include "Echo.h"

Echo::Echo()
{
    echo_content = "XINYU_DRONE\n";

    echo_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(echo_socket == -1){
        throw "Error create echo socket";
    }
}

int Echo::bindTo(){
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    int ret = bind(echo_socket, (sockaddr *) &addr, sizeof(addr));
    if(ret == -1){
        return -1;
    }

    return 0;
}

int Echo::listenOn(){
    int ret = listen(echo_socket, 1);
    if(ret == -1){
        return -1;
    }

    return 0;
}

void Echo::standBy(){
    while(1){
        client_socket = accept(echo_socket, NULL, NULL);
        if(client_socket == -1){
            continue;
        }
        send(client_socket, (void *)echo_content, strlen(echo_content), 1);
        close(client_socket);
        // printf("%d byte data send(actual length: %d)\n", byte, strlen(echo_content));
    }
}

Echo::~Echo()
{
    close(echo_socket);
}
