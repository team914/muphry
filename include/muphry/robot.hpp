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
extern std::shared_ptr<XDriveModel> model;
extern std::shared_ptr<CustomOdometry> odom;

//controllers
extern std::shared_ptr<AsyncMotionProfileController> controller;

//encoders
extern std::shared_ptr<ADIEncoder> left;
extern std::shared_ptr<ADIEncoder> right;
extern std::shared_ptr<ADIEncoder> middle;

//intake
extern std::shared_ptr<MotorGroup> intake;

//tray
extern std::shared_ptr<MotorGroup> tray;
extern std::shared_ptr<Potentiometer> trayPotent;
extern std::shared_ptr<AsyncPosPIDController> trayController;
extern std::shared_ptr<AsyncPosPIDController> viciousTrayController;

//tray vals
extern double trayUp;
extern double trayMiddleUp;
extern double trayMiddleDown;
extern double trayDown;

//lift
extern std::shared_ptr<Motor> lift;
extern std::shared_ptr<AsyncPosPIDController> liftController;

//lift vals
extern double liftUp;
extern double liftMiddle;
extern double liftDown;
extern bool liftMiddleDownToggle;
extern bool liftMiddleUpToggle;

//controller
extern std::shared_ptr<Controller> master;

//screen
extern std::shared_ptr<GUI::Screen> screen;
extern GUI::Selector* selector;

//overheat warning
void overheatWarn();
