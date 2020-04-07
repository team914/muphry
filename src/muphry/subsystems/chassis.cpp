#include "muphry/subsystems/chassis.hpp"

Chassis::Chassis(){
    MotorGroup left({topLeftPort,bottomLeftPort});
    MotorGroup right({topRightPort,bottomRightPort});
    skidSteerModel = std::make_shared<ThreeEncoderSkidSteerModel>(
        std::make_shared<MotorGroup>(left),
        std::make_shared<MotorGroup>(right),
        std::make_shared<ADIEncoder>(leftADIEncoderPort1,leftADIEncoderPort2),
        std::make_shared<ADIEncoder>(rightADIEncoderPort1,rightADIEncoderPort2),
        std::make_shared<ADIEncoder>(4,5),
        200,
        12000
    );
    skidSteerModel->setBrakeMode(chassisBrakeMode);
    skidSteerModel->setGearing(chassisGearset);

    odom = std::make_shared<CustomOdometry>(
        skidSteerModel,
        adiScales
    );

    pidController = std::make_shared<ChassisControllerPID>(
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
    pidController->setVelocityMode(false);

    leftProfileController = std::make_shared<AsyncLinearMotionProfileController>(
        TimeUtilFactory().create(),
        PathfinderLimits{ speedLimits, accelerationLimits, jerkLimits },
        std::make_shared<MotorGroup>(left),
        wheelDiameter,
        chassisGearsetRatioPair
    );
    leftProfileController->startThread();

    rightProfileController = std::make_shared<AsyncLinearMotionProfileController>(
        TimeUtilFactory().create(),
        PathfinderLimits{ speedLimits, accelerationLimits, jerkLimits },
        std::make_shared<MotorGroup>(right),
        wheelDiameter,
        chassisGearsetRatioPair
    );
    rightProfileController->startThread();

    profileController = std::make_shared<AsyncMotionProfileController>(
        TimeUtilFactory().create(),
        PathfinderLimits{speedLimits,accelerationLimits,jerkLimits },
        skidSteerModel,
        chassisScales,
        chassisGearsetRatioPair
    );

    odomController = std::make_shared<OdomController>(
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

    pursuitController = std::make_shared<PathFollower>(
        skidSteerModel,
        odom,
        chassisScales,
        lookahead,
        driveRadius
    );

    stopControllers();

}

void Chassis::stopControllers(){
    pidController->stop();

    leftProfileController->flipDisable(true);
    leftProfileController->reset();

    rightProfileController->flipDisable(true);    
    rightProfileController->reset();

    profileController->flipDisable(true);
    profileController->reset();
}