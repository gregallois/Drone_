#include <iostream>
#include<string>
#include "Drone.h"
#include "Listener.h"
#include "ThreadStruct.h"
#include "Thread.h"
#include "BlockingQueue.h"
#include "MessageChecker.h"
#include <unistd.h>



using namespace std;



Drone::Drone(){
  
  pthread_mutex_init(&m_mutex, NULL);
  pthread_cond_init(&m_condv, NULL);
  
  
  threadList[0]=new ThreadStruct("mpr",2,1,Thread::FIFO);
  threadList[1]=new ThreadStruct("xbee",2,1,Thread::FIFO);
};

Drone::~Drone(){};


void* Drone::start(){
       
  startCom();
  
  pthread_mutex_lock(&m_mutex);
  pthread_cond_wait(&m_condv, &m_mutex);
  pthread_mutex_unlock(&m_mutex);
  
};

void* Drone::startCom(){
    moduleCom = new Communication(this);
    moduleCom->start();
    
};

void* Drone::startThread(Runnable* runnable, int id){
  threadList[id]->start(runnable);
};


void* Drone::shutOff(){
  pthread_mutex_lock(&m_mutex);
  pthread_cond_signal(&m_condv);
  pthread_mutex_unlock(&m_mutex);
};