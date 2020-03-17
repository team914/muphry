#pragma once

#include "main.h"

#include "lib7842/api.hpp"
#include "okapi/api.hpp"

#include <memory>
#include <map>
#include <functional>

using namespace lib7842;
using namespace okapi;

//controller
extern std::shared_ptr<Controller> master;

//controller buttons
extern std::shared_ptr<ControllerButton> intakeUp;
extern std::shared_ptr<ControllerButton> intakeDown;
extern std::shared_ptr<ControllerButton> tilterUp;
extern std::shared_ptr<ControllerButton> tilterDown;
extern std::shared_ptr<ControllerButton> liftUp;
extern std::shared_ptr<ControllerButton> liftMid;

//screen
extern std::shared_ptr<GUI::Screen> screen;
extern GUI::Selector* selector;
extern GUI::Actions* intakeActions;
extern GUI::Actions* liftActions;
extern GUI::Actions* tilterActions;

//intake
const int leftIntakePort{4};
const int rightIntakePort{-7};
const QLength intakeDiameter{3_in};
const double intakekP{.001};
const double intakekI{0};
const double intakekD{0};

//lift
const int liftPort{15};
const double liftkP{.2};
const double liftkI{0};
const double liftkD{.0};
const double midTower{680};
const double lowTower{500};
const double a2CubeStack{150};
const double a3CubeStack{350};
const double a4CubeStack{550};
const double down{0};
