#include "muphry/subsystems/chassis.hpp"

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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

    log = std::make_shared<AsyncKinematicsLog>(
        std::make_shared<ADIEncoder>(leftADIEncoderPort1, leftADIEncoderPort2),
        adiScales.middleWheelDiameter,
        360,
        1,
        10_ms
    );
    log->startThread();
    log->flipDisable(true);

    stopControllers();
}

void Chassis::linearProfileStraight(QLength idistance, QLength icurrentPos){
    leftProfileController->generatePath ( {icurrentPos.abs(),idistance.abs()}, "straight" );
    rightProfileController->generatePath( {icurrentPos.abs(),idistance.abs()}, "straight" );
    bool backward = false;
    if(sgn(idistance.convert(meter))==-1){
        backward = true;
    }
    leftProfileController->setTarget( "straight", backward);
    rightProfileController->setTarget("straight", backward);
    linearProfileWaitTilSettled();
    leftProfileController ->removePath("straight");
    rightProfileController->removePath("straight");
}

void Chassis::linearProfileTurn(QAngle iangle, QLength icurrentPos){
    QLength turnLength = iangle.convert(radian) * (chassisScales.wheelDiameter * PI);
    leftProfileController->generatePath ( {icurrentPos.abs(),turnLength.abs()}, "turn" );
    rightProfileController->generatePath( {icurrentPos.abs(),turnLength.abs()}, "turn" );
    bool left = false;
    if(sgn(iangle.convert(radian))==-1){
        left = true;
    }
    leftProfileController->setTarget( "turn", left);
    rightProfileController->setTarget("turn", !left);
    linearProfileWaitTilSettled();
    leftProfileController ->removePath("turn");
    rightProfileController->removePath("turn");
}

bool Chassis::linearProfileWaitTilSettled(){
    while(!leftProfileController->isSettled() && !rightProfileController->isSettled()){
        pros::delay(10);
    }
    return true;
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
