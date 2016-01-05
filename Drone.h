#ifndef Drone_Header_h
#define Drone_Header_h


#include <pthread.h>

#include "ThreadStruct.h"
#include "Communication.h"
#include "IMU.h"
#include "Point3D.hpp"
#include "Message.h"
#include "Autopilot.hpp"
#include "EKF.h"


//---------------------Variables Globales de Configuration--------------------//


//********Fichiers de Config*******//



//********Ports*******//
char* Radio = "/dev/tty.usbserial-AL01838T";
char* IMUport = "/dev/ttyUSB1";


//********Threads*******//
int mCheckerThread = 0;
int xbeeListenerThread = 1;
int mProcessorThread = 2;
int mSenderThread = 3;
int pingProcessorThread = 4;
int eKFthread = 5;
int autoPilotThread = 6;

//********Paramètres PID*******//





//--------------------Code du Drone-------------------------------------------//


class Communication;
class IMU;
class EKF;
class Autopilot;

class Drone {
  
  
  public:
    
    Drone();
    ~Drone();
    
//********Fonctions d'initialisation*******//
    void start();
    void startCom();
    void startIMU();//****************à implémenter***************************//
    void startAutoPilot();
    void startEKF();
    
    void readConfig();
    void readThreadConfig();
    
    
    void startThread(Runnable* runnable, int id);


//********Fonctions "carrefours" entre modules*******//
    void sendMsg(Message* msg);
    
//********Actions définies*******//
    void shutOff();
    
    int* getComId();
    
//********Modification de la config*******//
    //void changeConfig(char[] change)

//********Accès/modification des paramètres d'état*******//
    Point3D<uint16_t>* getPos();
    Point3D<uint16_t>* getSpeed();
    Point3D<uint16_t>* getTarget();
    Point3D<uint16_t>* getAngles();
    Point3D<uint16_t>* getPosIncr();
    Point3D<uint16_t>* getSpeedIncr();
    Point3D<uint16_t>* getAngleIncr();
    uint16_t* getAlt();
    uint16_t* getAltIncr();
    uint8_t* getCharge();
    uint8_t* getChargeIncr();
  
    void setPos(uint16_t x, uint16_t y, uint16_t z);
    void setTarget(uint16_t x, uint16_t y, uint16_t z);
    void setSpeed(uint16_t vx, uint16_t vy, uint16_t vz);
    void setAngles(uint16_t a, uint16_t b, uint16_t c);
    void setAlt(uint16_t z);
    void setPosIncr(uint16_t x, uint16_t y, uint16_t z);
    void setSpeedIncr(uint16_t vx, uint16_t vy, uint16_t vz);
    void setAngleIncr(uint16_t a, uint16_t b, uint16_t c);
    void setAltIncr(uint16_t z);
    void setCharge(uint8_t c);
    void setChargeIncr(uint8_t c);

/*******************************************************/
    
  private:
    
    pthread_mutex_t m_mutex;
    pthread_cond_t  m_condv;
    
    

    ThreadStruct* threadList[20];
    
//********Modules*******//
    Communication* moduleCom;
    IMU* moduleIMU;
    Autopilot* pilot;
    EKF* moduleEKF;
 

    
//********Paramètres d'état*******//
    Point3D<uint16_t>* target;
    Point3D<uint16_t>* pos;
    Point3D<uint16_t>* speed;
    Point3D<uint16_t>* angles;
    Point3D<uint16_t>* posincr;
    Point3D<uint16_t>* speedincr;
    Point3D<uint16_t>* angleincr;
    pthread_mutex_t alt_mutex;
    uint16_t* alt;
    pthread_mutex_t altincr_mutex;
    uint16_t* altincr;
    
    pthread_mutex_t charge_mutex;
    uint8_t* charge;
    pthread_mutex_t chargeincr_mutex;
    uint8_t* chargeincr;

};

#endif

  