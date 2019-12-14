#pragma once

#include "okapi/api.hpp"
#include "robot/robot.hpp"
#include "robot/screens/screen.hpp"

using namespace lib7842;

namespace Screen{

extern State resetState;
void odomResetter();
void odomResetState( const State& state );
void initOdom();

}
