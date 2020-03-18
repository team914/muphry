#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

enum class LiftState{
    midTower,
    lowTower,
    a2CubeStack,
    a3CubeStack,
    a4CubeStack,
    down,
    off
};

class Lift : public StateMachine<LiftState, LiftState::off> {
    private:
    Lift();

    std::shared_ptr<Motor> liftMotor{nullptr};
    std::shared_ptr<AsyncPosPIDController> liftController{nullptr};

    virtual void initialize();

    void loop() override;

    static Lift* lift;

    public:
    static Lift* getLift();
};