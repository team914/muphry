#pragma once

#include "main.h"

#include "lib7842/api.hpp"
#include "okapi/api.hpp"
#include "muphry/subsystems/chassis.hpp"

#include <memory>
#include <map>
#include <functional>

using namespace lib7842;
using namespace okapi;
using namespace okapi::literals;

//controller
extern std::shared_ptr<Controller> master;

//controller buttons
extern std::shared_ptr<ControllerButton> intakeUpBtn;
extern std::shared_ptr<ControllerButton> intakeDownBtn;
extern std::shared_ptr<ControllerButton> tilterUpBtn;
extern std::shared_ptr<ControllerButton> tilterDownBtn;
extern std::shared_ptr<ControllerButton> liftUpBtn;
extern std::shared_ptr<ControllerButton> liftMidBtn;

//screen
extern std::shared_ptr<GUI::Screen> screen;
extern GUI::Selector* selector;
extern GUI::Actions* intakeActions;
extern GUI::Actions* liftActions;
extern GUI::Actions* tilterActions;

//intake
const int leftIntakePort{4};
const int rightIntakePort{-7};
const AbstractMotor::gearset intakeGearset{AbstractMotor::gearset::green};
const QLength intakeDiameter{3_in};
const double intakekP{.001};
const double intakekI{0};
const double intakekD{0};

//lift
const int liftPort{15};
const AbstractMotor::gearset liftGearset{AbstractMotor::gearset::red};
const double liftkP{.005};
const double liftkI{0};
const double liftkD{.0001};
const double midTower{680};
const double lowTower{500};
const double a2CubeStack{150};
const double a3CubeStack{350};
const double a4CubeStack{550};
const double liftDown{-20};
extern bool liftToggle;//true means the lift is up false means the lift is down

//tilter
const int tilterPort{9};
const AbstractMotor::gearset tilterGearset{AbstractMotor::gearset::red};
const double tilterkP{.01};
const double tilterkI{0};
const double tilterkD{.0};
const double tilterUp{1100};
const double tilterLiftUp{100};
const double tilterDown{0};

//chassis
extern std::shared_ptr<Chassis> chassis;
