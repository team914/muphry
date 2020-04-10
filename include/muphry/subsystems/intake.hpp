#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

enum class IntakeState{
    inFull,
    outFull,
    inHalf,
    outHalf,
    moveDistance,
    hold,
    off
};

class Intake : public StateMachine<IntakeState, IntakeState::off> {
    private:
    Intake();

    double circumference{.0001};
    double distance{0};

    std::shared_ptr<Motor> leftIntake{nullptr};
    std::shared_ptr<Motor> rightIntake{nullptr};
    std::shared_ptr<AsyncPosPIDController> leftController{nullptr};
    std::shared_ptr<AsyncPosPIDController> rightController{nullptr};

    virtual void initialize();

    void loop() override;

    static Intake* intake;

    public:
    static Intake* getIntake();

    void setDistance( QLength idistance );
};