/**
 * @author     Acetousk
 * @date       2020
 */
#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

/**
 * @brief      This class describes an intake state.
 */
enum class IntakeState{
    inFull,
    outFull,
    inHalf,
    outHalf,
    moveDistance,
    hold,
    off
};

/**
 * @brief      This class describes an intake.
 */
class Intake : public StateMachine<IntakeState, IntakeState::off> {
    private:

    /**
     * @brief      Constructs a new instance.
     */
    Intake();

    /**
     * circumference of intake
     */
    double circumference{.0001};

    /**
     * set distance
     */
    double distance{0};

    /**
     * left intake
     */
    std::shared_ptr<Motor> leftIntake{nullptr};

    /**
     * right intake
     */
    std::shared_ptr<Motor> rightIntake{nullptr};

    /**
     * left intake controller
     */
    std::shared_ptr<AsyncPosPIDController> leftController{nullptr};

    /**
     * right intake controller
     */
    std::shared_ptr<AsyncPosPIDController> rightController{nullptr};

    /**
     * @brief      Initializes the object.
     */
    virtual void initialize();

    /**
     * @brief      this is the loop for the intake task
     */
    void loop() override;

    /**
     * intake singleton
     */
    static Intake* intake;

    public:
    /**
     * @brief      Gets the intake.
     *
     * @return     The intake.
     */
    static Intake* getIntake();

    /**
     * @brief      Sets the distance.
     *
     * @param[in]  idistance  The idistance
     */
    void setDistance( QLength idistance );
};