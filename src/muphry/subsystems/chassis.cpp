#include "muphry/subsystems/chassis.hpp"

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Chassis::Chassis(){
    MotorGroup left({topLeftPort,bottomLeftPort});
    MotorGroup right({topRightPort,bottomRightPort});
    skidSteerModel = std::make_shared<SkidSteerModel>(
        std::make_shared<MotorGroup>(left),
        std::make_shared<MotorGroup>(right),
        std::make_shared<ADIEncoder>(leftADIEncoderPort1,leftADIEncoderPort2),
        std::make_shared<ADIEncoder>(rightADIEncoderPort1,rightADIEncoderPort2),
        200,
        12000
    );
    skidSteerModel->setBrakeMode(chassisBrakeMode);
    skidSteerModel->setGearing(chassisGearset);

    odom = std::make_shared<TwoEncoderOdometry>(
        TimeUtilFactory().create(),
        skidSteerModel,
        adiScales
    );
    odom->setState(OdomState{6_ft,6_ft,0_deg});

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
        turnLimits,
        std::make_shared<MotorGroup>(left),
        chassisScales.wheelDiameter,
        chassisGearsetRatioPair
    );
    leftProfileController->startThread();

    rightProfileController = std::make_shared<AsyncLinearMotionProfileController>(
        TimeUtilFactory().create(),
        turnLimits,
        std::make_shared<MotorGroup>(right),
        chassisScales.wheelDiameter,
        chassisGearsetRatioPair
    );
    rightProfileController->startThread();

    profileController = std::make_shared<AsyncMotionProfileController>(
        TimeUtilFactory().create(),
        turnLimits,
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

void Chassis::linearProfileStraight(QLength idistance, QLength icurrentPos){
    leftProfileController->generatePath ( {icurrentPos.abs(),idistance.abs()}, "straight", straightLimits );
    rightProfileController->generatePath( {icurrentPos.abs(),idistance.abs()}, "straight", straightLimits );
    bool backward = false;
    if(sgn(idistance.convert(meter))==-1){
        backward = true;
    }
    leftProfileController->setTarget( "straight", backward);
    rightProfileController->setTarget("straight", backward);
    while(!linearProfileWaitTilSettled());
    leftProfileController ->removePath("straight");
    rightProfileController->removePath("straight");
}

void Chassis::linearProfileTurn(QAngle iangle, QLength icurrentPos){
    QLength turnLength = iangle.convert(radian) * (chassisScales.wheelTrack/2);
    leftProfileController->generatePath ( {icurrentPos.abs(),turnLength.abs()}, "turn", turnLimits );
    rightProfileController->generatePath( {icurrentPos.abs(),turnLength.abs()}, "turn", turnLimits );
    bool left = false;
    if(sgn(iangle.convert(radian))==-1){
        left = true;
    }
    leftProfileController->setTarget( "turn", left);
    rightProfileController->setTarget("turn", !left);
    pros::delay(10);
    while(!linearProfileWaitTilSettled());
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
