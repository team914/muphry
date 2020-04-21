#pragma once

#include "okapi/api.hpp"
#include "lib7842/api.hpp"
#include "muphry/statemachine.hpp"
//#include "asyncKinematicsLog/asyncKinematicsLog.hpp"

#include <math.h>

using namespace okapi;
using namespace okapi::literals;
using namespace lib7842;
using namespace lib7842::literals;

class Chassis{
    public:
    Chassis();

    //odom
    std::shared_ptr<TwoEncoderOdometry> odom{nullptr};

    //chassis models
    std::shared_ptr<SkidSteerModel> skidSteerModel{nullptr};

    //pid controllers
    std::shared_ptr<ChassisControllerPID> pidController{nullptr};

    //profile controllers
    void linearProfileStraight(QLength idistance, QLength icurrentPos = 0_in);
    void linearProfileTurn(QAngle iangle, QLength icurrentPos = 0_in);
    bool linearProfileWaitTilSettled();
    std::shared_ptr<AsyncLinearMotionProfileController> leftProfileController{nullptr};
    std::shared_ptr<AsyncLinearMotionProfileController> rightProfileController{nullptr};
    std::shared_ptr<AsyncMotionProfileController> profileController{nullptr};

    //odom controllers
    std::shared_ptr<OdomController> odomController{nullptr};
    std::shared_ptr<PathFollower> pursuitController{nullptr};

    //log
//    std::shared_ptr<AsyncKinematicsLog> log{nullptr};

    void stopControllers();

    private:
    const int topLeftPort{6};
    const int topRightPort{-8};
    const int bottomLeftPort{5};
    const int bottomRightPort{-20};
    const int rightADIEncoderPort1{1};
    const int rightADIEncoderPort2{2};
    const int leftADIEncoderPort1{7};
    const int leftADIEncoderPort2{8};
    const AbstractMotor::gearset chassisGearset{AbstractMotor::gearset::green};
    const double ratio{1};
    const AbstractMotor::GearsetRatioPair chassisGearsetRatioPair{AbstractMotor::GearsetRatioPair{AbstractMotor::gearset::green, ratio}};
    const AbstractMotor::brakeMode chassisBrakeMode{AbstractMotor::brakeMode::coast};
    const ChassisScales adiScales{{2.8114_in,9.883_in,.01_in,2.8114_in}, 360};
    const ChassisScales chassisScales{{4.036_in,9.183748_in}, imev5GreenTPR};
    const QLength lookahead{4_in};
    const QLength driveRadius{4_in};
    const double chassisDistancekP{.003};
    const double chassisDistancekI{.001};
    const double chassisDistancekD{.00016};
    const TimeUtil chassisDistanceTimeUtil{TimeUtilFactory::withSettledUtilParams(25, 10, 100_ms)};
    const double chassisTurnkP{.0022};
    const double chassisTurnkI{.0005};
    const double chassisTurnkD{.00005};
    const TimeUtil chassisTurnTimeUtil{TimeUtilFactory::withSettledUtilParams(15, 5, 100_ms)};
    const double chassisAnglekP{.0016};
    const double chassisAnglekI{.000};
    const double chassisAnglekD{.0000};
    const TimeUtil chassisAngleTimeUtil{TimeUtilFactory::withSettledUtilParams(1000, 5, 100_ms)};
    const PathfinderLimits straightLimits{1.75,.75,12.8};
    const double turnScale{.25};
    const PathfinderLimits turnLimits{  straightLimits.maxVel*turnScale, 
                                        straightLimits.maxAccel*turnScale,
                                        straightLimits.maxJerk*turnScale};
};
//1 | 308 | 647 | 989 | 1315 | 1650