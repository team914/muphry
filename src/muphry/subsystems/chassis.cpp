#include "muphry/subsystems/chassis.hpp"

void Chassis::initialize(){}

void Chassis::loop(){
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

void Chassis::resetModels(){
    skidSteerModel = nullptr;
    holonomicModel = nullptr;
}

void Chassis::resetControllers(){
    pidController = nullptr;
    integratedController = nullptr;
    leftProfileController = nullptr;
    rightProfileController = nullptr;
    profileController = nullptr;
    odomController = nullptr;
    odomXController = nullptr;
    pathFollowerController = nullptr;
    pathFollowerXController = nullptr;
}

void Chassis::resetOdom(){
    odom = nullptr;
}

std::shared_ptr<ThreeEncoderSkidSteerModel> Chassis::makeSkidSteerModel(){
    MotorGroup left({topLeftPort,bottomLeftPort});
    MotorGroup right({bottomLeftPort,bottomRightPort});
    auto model = std::make_shared<ThreeEncoderSkidSteerModel>(
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

    return model;
}

std::shared_ptr<ThreeEncoderXDriveModel> Chassis::makeHolonomicModel(){
    auto model = std::make_shared<ThreeEncoderXDriveModel>(
        std::make_shared<Motor>(topLeftPort),
        std::make_shared<Motor>(topRightPort),
        std::make_shared<Motor>(bottomRightPort),
        std::make_shared<Motor>(bottomLeftPort),
        std::make_shared<ADIEncoder>(leftADIEncoderPort1,leftADIEncoderPort2),
        std::make_shared<ADIEncoder>(rightADIEncoderPort1,rightADIEncoderPort2),
        nullptr,
        200,
        12000
    );
    model->setBrakeMode(chassisBrakeMode);
    model->setGearing(chassisGearset);

    return model;
}

std::shared_ptr<ChassisControllerPID> Chassis::makePidController(){
    resetModels();
    resetControllers();
    resetOdom();
    if(!modelType){
        skidSteerModel = makeSkidSteerModel();
        auto controller = std::make_shared<ChassisControllerPID>(
            TimeUtilFactory().create(),
            skidSteerModel,
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

        return controller;
    }else{
        holonomicModel = makeHolonomicModel();
        auto controller = std::make_shared<ChassisControllerPID>(
            TimeUtilFactory().create(),
            skidSteerModel,
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

        return controller;
    }
}

std::shared_ptr<ChassisControllerIntegrated> Chassis::makeIntegratedController(){
    resetModels();
    resetControllers();
    resetOdom();

    if(!modelType){
        skidSteerModel = makeSkidSteerModel();

        MotorGroup left({topLeftPort,bottomLeftPort});
        MotorGroup right({bottomLeftPort,bottomRightPort});

        auto controller = std::make_shared<ChassisControllerIntegrated>(
            TimeUtilFactory().create(),
            skidSteerModel,
            std::make_unique<AsyncPosIntegratedController>(
                std::make_shared<MotorGroup>(left),
                chassisGearsetRatioPair,
                200,
                TimeUtilFactory().create()
            ),
            std::make_unique<AsyncPosIntegratedController>(
                std::make_shared<MotorGroup>(right),
                chassisGearsetRatioPair,
                200,
                TimeUtilFactory().create()
            )
        );

        return controller;

    }else{
        holonomicModel = makeHolonomicModel();

        MotorGroup left({topLeftPort,bottomLeftPort});
        MotorGroup right({bottomLeftPort,bottomRightPort});

        auto controller = std::make_shared<ChassisControllerIntegrated>(
            TimeUtilFactory().create(),
            holonomicModel,
            std::make_unique<AsyncPosIntegratedController>(
                std::make_shared<MotorGroup>(left),
                chassisGearsetRatioPair,
                200,
                TimeUtilFactory().create()
            ),
            std::make_unique<AsyncPosIntegratedController>(
                std::make_shared<MotorGroup>(right),
                chassisGearsetRatioPair,
                200,
                TimeUtilFactory().create()
            )
        );

        return controller;
    }
}

std::shared_ptr<AsyncLinearMotionProfileController> Chassis::makeLeftProfileController(){
    resetControllers();
    resetOdom();

    MotorGroup left({topLeftPort,bottomLeftPort});

    auto controller = std::make_shared<AsyncLinearMotionProfileController>(
        TimeUtilFactory().create(),
        PathfinderLimits{speedLimits.convert(mps),accelerationLimits.convert(mps2),jerkLimits.convert(mps2 / second) },
        std::make_shared<MotorGroup>(left),
        chassisScales.wheelDiameter,
        chassisGearsetRatioPair
    );
    controller->startThread();

    return controller;
}

std::shared_ptr<AsyncLinearMotionProfileController> Chassis::makeRightProfileController(){
    resetControllers();
    resetOdom();

    MotorGroup right({bottomLeftPort,bottomRightPort});

    auto controller = std::make_shared<AsyncLinearMotionProfileController>(
        TimeUtilFactory().create(),
        PathfinderLimits{speedLimits.convert(mps),accelerationLimits.convert(mps2),jerkLimits.convert(mps2 / second) },
        std::make_shared<MotorGroup>(right),
        chassisScales.wheelDiameter,
        chassisGearsetRatioPair
    );
    controller->startThread();

    return controller;
}

std::shared_ptr<AsyncMotionProfileController> Chassis::makeProfileController(){
    resetModels();
    resetControllers();
    resetOdom();

    if(!modelType){
        skidSteerModel = makeSkidSteerModel();
        auto controller = std::make_shared<AsyncMotionProfileController>(
            TimeUtilFactory().create(),
            PathfinderLimits{speedLimits.convert(mps),accelerationLimits.convert(mps2),jerkLimits.convert(mps2 / second) },
            skidSteerModel,
            chassisScales,
            chassisGearsetRatioPair
        );
        controller->startThread();

        return controller;

    }else{
        holonomicModel = makeHolonomicModel();
        auto controller = std::make_shared<AsyncMotionProfileController>(
            TimeUtilFactory().create(),
            PathfinderLimits{speedLimits.convert(mps),accelerationLimits.convert(mps2),jerkLimits.convert(mps2 / second) },
            holonomicModel,
            chassisScales,
            chassisGearsetRatioPair
        );
        controller->startThread();

        return controller;
    }

}

std::shared_ptr<CustomOdometry> Chassis::makeOdom(){
    resetModels();
    resetControllers();
    resetOdom();

    if(!modelType){
        skidSteerModel = makeSkidSteerModel();
        auto controller = std::make_shared<CustomOdometry>(
            skidSteerModel,
            adiScales
        );

        return controller;

    }else{
        holonomicModel = makeHolonomicModel();
        auto controller = std::make_shared<CustomOdometry>(
            holonomicModel,
            adiScales
        );


        return controller;
    }

}

std::shared_ptr<OdomController> Chassis::makeOdomController(){
    resetModels();
    resetControllers();
    resetOdom();

    odom = makeOdom();

    modelType = false;

    {
        skidSteerModel = makeSkidSteerModel();
        auto controller = std::make_shared<OdomController>(
            skidSteerModel,
            odom,
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
            driveRadius
        );

        return controller;

    }
}

std::shared_ptr<OdomXController> Chassis::makeOdomXController(){
    resetModels();
    resetControllers();
    resetOdom();

    odom = makeOdom();

    modelType = true;

    {
        holonomicModel = makeHolonomicModel();
        auto controller = std::make_shared<OdomXController>(
            holonomicModel,
            odom,
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
            driveRadius
        );

        return controller;
    }

}

std::shared_ptr<PathFollower> Chassis::makePathFollowerController(){
    resetModels();
    resetControllers();
    resetOdom();

    odom = makeOdom();

    modelType = true;

    {
        skidSteerModel = makeSkidSteerModel();
        auto controller = std::make_shared<PathFollower>(
            skidSteerModel,
            odom,
            chassisScales,
            lookahead,
            driveRadius
        );

        return controller;
    }

}

std::shared_ptr<PathFollowerX> Chassis::makePathFollowerXController(){
    resetModels();
    resetControllers();
    resetOdom();

    odom = makeOdom();

    modelType = true;

    {
        holonomicModel = makeHolonomicModel();
        auto controller = std::make_shared<PathFollowerX>(
            holonomicModel,
            odom,
            chassisScales,
            lookahead
        );

        return controller;
    }

}


Chassis* Chassis::getChassis(){
    if(!chassis){
        chassis = new Chassis();
    }
    return chassis;
}

void Chassis::chassisDriver( double iforward, double iright, double iyaw ){
    forward = iforward;
    right = iright;
    yaw = iyaw;
}
