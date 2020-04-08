#pragma once

#include "okapi/api.hpp"
#include "lib7842/api.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/iterativeVelMotionProfileController.hpp"
#include "asyncKinematicsLog/asyncKinematicsLog.hpp"

#include <math.h>

using namespace okapi;
using namespace okapi::literals;
using namespace lib7842;
using namespace lib7842::literals;

class Chassis{
    public:
    Chassis();

    //odom
    std::shared_ptr<CustomOdometry> odom{nullptr};

    //chassis models
    std::shared_ptr<ThreeEncoderSkidSteerModel> skidSteerModel{nullptr};

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
    std::shared_ptr<AsyncKinematicsLog> log{nullptr};

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
    const QLength wheelDiameter{4_in};
    const ChassisScales chassisScales{{4_in,9_in}, imev5GreenTPR};
    const QLength lookahead{4_in};
    const QLength driveRadius{4_in};
    const double chassisDistancekP{.0015};
    const double chassisDistancekI{.0009};
    const double chassisDistancekD{.00003};
    const TimeUtil chassisDistanceTimeUtil{TimeUtilFactory::withSettledUtilParams(40, 10, 100_ms)};
    const double chassisTurnkP{.0026};
    const double chassisTurnkI{.000};
    const double chassisTurnkD{.00005};
    const TimeUtil chassisTurnTimeUtil{TimeUtilFactory::withSettledUtilParams(40, 5, 100_ms)};
    const double chassisAnglekP{.0002};
    const double chassisAnglekI{.0008};
    const double chassisAnglekD{.0000};
    const TimeUtil chassisAngleTimeUtil{TimeUtilFactory::withSettledUtilParams(40, 5, 100_ms)};
    const double speedLimits{1};
    const double accelerationLimits{1.4};
    const double jerkLimits{12.8};
};
