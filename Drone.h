#ifndef Drone_Header_h
#define Drone_Header_h


#include <pthread.h>

#include "ThreadStruct.h"
#include "Communication.h"

class Communication;

class Drone {
  
  
  public:
    
    Drone();
    ~Drone();
    void* start();
    void* startCom();
    void* readConfig();
    void* readThreadConfig();
    void* startThread(Runnable* runnable, int id);
    
    void* sendMsg();
    
    void* shutOff();
    
  
  private:
    
    pthread_mutex_t m_mutex;
    pthread_cond_t  m_condv;
	
	
    ThreadStruct* threadList[20];
    Communication* moduleCom;
};

#endif

  