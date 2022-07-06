#include "perf_cnt.h"

unsigned long _uptime() {
  // TODO [COD]
  //   You can use this function to access performance counter related with time or cycle.
  return *(volatile unsigned int *)0x60010000;
}

void bench_prepare(Result *res) {
  // TODO [COD]
  //   Add preprocess code, record performance counters' initial states.
  //   You can communicate between bench_prepare() and bench_done() through
  //   static variables or add additional fields in `struct Result`
  res->msec = _uptime();
  for(int i=0; i<=7; i++){
    res->cnt[2*i] = *(volatile unsigned int *)((char *)0x60010000+(i<<12));
    res->cnt[2*i+1] = *(volatile unsigned int *)((char *)0x60010008+(i<<12));
  }
}

void bench_done(Result *res) {
  // TODO [COD]
  //  Add postprocess code, record performance counters' current states.
  //printf("Enter bench_done success\n");
  res->msec = _uptime() - res->msec;
  //printf("Update res->msec success\n");
  for(int i=0; i<=7; i++){
    //printf("for %d\n",2*i);
    res->cnt[2*i] = *(volatile unsigned int *)((char *)0x60010000+(i<<12)) - res->cnt[2*i];
    //printf("for %d\n",2*i+1);
    res->cnt[2*i+1] = *(volatile unsigned int *)((char *)0x60010008+(i<<12)) - res->cnt[2*i+1];
  }
  //printf("Leaving bench_done\n");
}

