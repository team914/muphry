#include "muphry/subsystems/abstractChassis.hpp"

AbstractChassis::AbstractChassis(){
    chassis = nullptr;
}

void AbstractChassis::initialize(){}

void AbstractChassis::loop(){
    while(true){
        printf("Chassis State = ");
        switch(state){
            case ChassisState::auton:
                printf("auton\n");
                setDone();
            break;
            case ChassisState::driver:
                printf("driver\n");
                setDone();
            break;
            case ChassisState::off:
                printf("off\n");
                setDone();
            break;
        }
    }
}

AbstractChassis* AbstractChassis::getChassis(){
    if(!chassis){
        chassis = new AbstractChassis();
    }
    return chassis;
}
