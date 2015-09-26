#include <iostream>
#include <string>
#include "Drone.h"


using namespace std;

const char* Radio = "/dev/ttyUSB0";

int mCheckerThread = 0;
int xbeeListenerThread = 1;


//string messageType[2] = {"SYSTEM","CONTROL", "CONFIG"};


int main()
{
  Drone *d = new Drone();
  d->start();
}
