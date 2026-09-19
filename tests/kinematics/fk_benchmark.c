#define _POSIX_C_SOURCE 200809L
#include "robot_kinematics.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
static uint64_t now_ns(void) {
    struct timespec t;
    if(clock_gettime(CLOCK_MONOTONIC,&t)!=0) return 0;
    return (uint64_t)t.tv_sec*UINT64_C(1000000000)+(uint64_t)t.tv_nsec;
}
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    FILE *f=fopen(argv[1],"rb"); if(!f) return 2;
    robot_fk_model model; double q[115][6],out[16];
    int valid=fread(&model,sizeof(model),1,f)==1 && fread(q,sizeof(q),1,f)==1 && fgetc(f)==EOF;
    fclose(f);if(!valid) return 2;
    for(size_t i=0;i<1000;i++) if(robot_forward(&model,q[i%115],6,out)) return 3;
    uint64_t elapsed[11500];int codes[11500];
    for(size_t i=0;i<11500;i++) {
        uint64_t start=now_ns();
        codes[i]=robot_forward(&model,q[i%115],6,out);
        uint64_t end=now_ns(); if(!start || end<start) return 4;
        elapsed[i]=end-start;
    }
    puts("sample_index,elapsed_ns,code");
    for(size_t i=0;i<11500;i++) printf("%zu,%llu,%d\n",i%115,(unsigned long long)elapsed[i],codes[i]);
    return 0;
}
