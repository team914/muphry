#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

enum class TilterState{
    up,
    liftUp,
    down,
    off
};

class Tilter : public StateMachine<TilterState, TilterState::off> {
    private:
    Tilter();

    std::shared_ptr<Motor> tilterMotor{nullptr};
    std::shared_ptr<AsyncPosPIDController> tilterController{nullptr};

    virtual void initialize();

    void loop() override;

    static Tilter* tilter;

    public:
    static Tilter* getTilter();

};