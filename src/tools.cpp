#include <sys/time.h>
#include <string.h>

double now(){
    struct timeval time;
    gettimeofday(&time, NULL);
    return time.tv_sec + time.tv_usec / 1000000.0;
}


int min(int val1, int val2){
    if(val1 > val2){
        return val2;
    }
    else {
        return val1;
    }
}

int max(int val1, int val2){
    if(val1 > val2){
        return val1;
    }
    else {
        return val2;
    }
}
