#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#ifndef Echo_H
#define Echo_H


class Echo
{
    public:
        Echo();

        int bindTo();
        int listenOn();
        void standBy();

        virtual ~Echo();
    protected:
    private:
        const static int port = 60001;
        const char* echo_content;

        int echo_socket;
        int client_socket;
};

#endif // Echo_H
