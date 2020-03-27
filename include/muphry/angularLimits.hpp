#pragma once

#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QAngularAcceleration.hpp"
#include "okapi/api/units/QAngularJerk.hpp"

using namespace okapi;
using namespace okapi::literals;

struct AngularLimits{
    QAngularSpeed speed;
    QAngularAcceleration accel;
};
