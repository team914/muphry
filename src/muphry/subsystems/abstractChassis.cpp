#include "muphry/subsystems/abstractChassis.hpp"

void AbstractChassis::initialize(){}

void AbstractChassis::loop(){
    while(true){
        printf("Chassis State = ");
        switch(state){
            case ChassisState::driver:
                printf("driver: forward %d, right %d, yaw %d\n", forward, right, yaw);
                setDone();
            break;
            case ChassisState::off:
                printf("off\n");
                setDone();
            break;
        }
        pros::delay(20);
    }
}

AbstractChassis* AbstractChassis::getChassis(){
    if(!chassis){
        chassis = new AbstractChassis();
    }
    return chassis;
}

void AbstractChassis::chassisDriver( double iforward, double iright, double iyaw ){
    forward = iforward;
    right = iright;
    yaw = iyaw;
}