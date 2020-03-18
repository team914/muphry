#include "muphry/subsystems/chassis/skidSteerPIDChassis.hpp"

using namespace okapi::literals;


void SkidSteerPIDChassis::initialize(){
    MotorGroup left({topLeftPort,bottomLeftPort});
    MotorGroup right({bottomLeftPort,bottomRightPort});
    model = std::make_shared<ThreeEncoderSkidSteerModel>(
        std::make_shared<MotorGroup>(left),
        std::make_shared<MotorGroup>(right),
        std::make_shared<ADIEncoder>(leftADIEncoderPort1,leftADIEncoderPort2),
        std::make_shared<ADIEncoder>(rightADIEncoderPort1,rightADIEncoderPort2),
        nullptr,
        200,
        12000
    );
    model->setBrakeMode(chassisBrakeMode);
    model->setGearing(chassisGearset);

    controller = std::make_shared<ChassisControllerPID>(
        TimeUtilFactory().create(),
        model,
        std::make_unique<IterativePosPIDController>(
            chassisDistancekP,
            chassisDistancekI,
            chassisDistancekD,
            0,
            chassisDistanceTimeUtil
        ),
        std::make_unique<IterativePosPIDController>(
            chassisTurnkP,
            chassisTurnkI,
            chassisTurnkD,
            0,
            chassisTurnTimeUtil
        ),
        std::make_unique<IterativePosPIDController>(
            chassisAnglekP,
            chassisAnglekI,
            chassisAnglekD,
            0,
            chassisAngleTimeUtil
        ),
        chassisGearset,
        adiScales
    );
    controller->startThread();    
}

void SkidSteerPIDChassis::loop(){
    initialize();
    while(true){
        printf("Chassis State = ");
        switch(state){
            case ChassisState::pidSkidSteer:
                printf("auton\n");
                setDone();
            break;
            case ChassisState::driver:
                printf("driver\n");
                model->driveVectorVoltage(forward, yaw);
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

SkidSteerPIDChassis* SkidSteerPIDChassis::getSkidSteerPIDChassis(){
    if(!skidSteerPIDChassis){
        skidSteerPIDChassis = new SkidSteerPIDChassis();
    }
    return skidSteerPIDChassis;
}

std::shared_ptr<ThreeEncoderSkidSteerModel> SkidSteerPIDChassis::getModel(){
    if(model){
        return model;
    }
}
std::shared_ptr<ChassisControllerPID> SkidSteerPIDChassis::getController(){
    if(controller){
        return controller;
    }
}
