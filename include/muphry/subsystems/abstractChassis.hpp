#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

enum class ChassisState{
    auton,
    driver,
    off
};

class AbstractChassis : public StateMachine<ChassisState, ChassisState::off> {
    private:
    AbstractChassis();

    virtual void initialize();

    void loop() override;

    static AbstractChassis* chassis;

    public:
    static AbstractChassis* getChassis();

};
