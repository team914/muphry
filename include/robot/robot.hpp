#pragma once

#include "okapi/api.hpp"
#include "lib7842/api.hpp"
#include <memory>

using namespace lib7842;
using namespace okapi;

namespace Robot{

void init();

extern std::shared_ptr<MotorGroup> intake;
extern std::shared_ptr<MotorGroup> tray;
extern std::shared_ptr<MotorGroup> other;
extern std::shared_ptr<ChassisController> chassis;
extern std::shared_ptr<ChassisModel> model;
extern std::shared_ptr<CustomOdometry> odom;

}//Robot
