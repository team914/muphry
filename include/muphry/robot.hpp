#pragma once

#include "main.h"

#include "lib7842/api.hpp"
#include "okapi/api.hpp"

#include <memory>
#include <map>
#include <functional>

using namespace lib7842;
using namespace okapi;

//chassis
extern std::shared_ptr<ChassisController> chassis;
extern std::shared_ptr<ChassisController> backwardChassis;
extern std::shared_ptr<SkidSteerModel> model;
extern std::shared_ptr<SkidSteerModel> backwardModel;
extern std::shared_ptr<TwoEncoderOdometry> odom;
extern std::shared_ptr<TwoEncoderOdometry> backwardOdom;

extern std::shared_ptr<OdomController> controller;

//intake
extern std::shared_ptr<MotorGroup> intake;

//tray
extern std::shared_ptr<MotorGroup> tray;
extern std::shared_ptr<Potentiometer> trayPotent;
extern std::shared_ptr<AsyncPosPIDController> trayController;
extern std::shared_ptr<AsyncPosPIDController> viciousTrayController;

//controller
extern std::shared_ptr<Controller> master;

//screen
extern std::shared_ptr<GUI::Screen> screen;
extern GUI::Selector* selector;

//overheat warning
void overheatWarn();
