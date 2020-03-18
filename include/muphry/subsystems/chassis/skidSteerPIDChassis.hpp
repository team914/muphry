#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"
#include "muphry/subsystems/abstractChassis.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

class SkidSteerPIDChassis : public AbstractChassis {
    protected:
    SkidSteerPIDChassis() = default;

    virtual void initialize();

    void loop() override;

    std::shared_ptr<ThreeEncoderSkidSteerModel> model{nullptr};
    std::shared_ptr<ChassisControllerPID> controller{nullptr};

    static SkidSteerPIDChassis* skidSteerPIDChassis;

    public:
    static SkidSteerPIDChassis* getSkidSteerPIDChassis();

    virtual void setState(const State& istate);
    virtual void setNewState(const State& istate);
    virtual void setStateBlocking(const State& istate);

    std::shared_ptr<ThreeEncoderSkidSteerModel> getModel();
    std::shared_ptr<ChassisControllerPID> getController();
    
};
