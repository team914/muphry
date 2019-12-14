#pragma once

#include "autolib/api.hpp"
#include "robot/modes/routines.hpp"
#include "robot/robot.hpp"

using namespace std;
using namespace Robot;

namespace Auto{

extern map<string, function<void()>> routines;

extern string routine;

void run();
void addAuto(const string& iroutine, const function<void()>&);
void runAuto(const string& id);

}//Auto