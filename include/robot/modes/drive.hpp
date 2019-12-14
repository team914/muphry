#pragma once

#include "robot/robot.hpp"

using namespace Robot;

namespace Drive{

void runIntake(double pct);
void runTray(double pct);
void runOther(double pct);
void runChassis(double pct, bool rightHanded);

void trayMacro();

enum chassis_t{tank, arcade, split_arcade, cheesy};

extern chassis_t chassisType;

extern ControllerButton trayPositive;
extern ControllerButton trayNegative;
extern ControllerButton intakePositive;
extern ControllerButton intakeNegative;
extern ControllerButton otherPositive;
extern ControllerButton otherNegative;

extern double deadzonePct;
extern double cheesyPct;

}//Drive
