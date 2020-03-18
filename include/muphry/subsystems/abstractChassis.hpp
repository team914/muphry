#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

enum class ChassisState{
    pidSkidSteer,
    driver,
    off
};

class AbstractChassis : public StateMachine<ChassisState, ChassisState::off> {
    protected:
    AbstractChassis() = default;

    virtual void initialize();

    void loop() override;

    static AbstractChassis* chassis;

    double forward{0};
    double right{0};
    double yaw{0};

    public:
    static AbstractChassis* getChassis();

    void chassisDriver( double iforward, double iright, double iyaw );

};
