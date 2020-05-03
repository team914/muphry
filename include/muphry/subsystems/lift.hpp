/**
 * @author     Acetousk
 * @date       2020
 */
#pragma once

#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/robot.hpp"
#include "muphry/statemachine.hpp"
#include "okapi/api.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

/**
 * @brief      This class describes a lift state.
 */
enum class LiftState { midTower, lowTower, a2CubeStack, a3CubeStack, a4CubeStack, down, off };

/**
 * @brief      This class describes a lift.
 */
class Lift : public StateMachine<LiftState, LiftState::off> {
  private:
  /**
   * @brief      Constructs a new instance.
   */
  Lift();

  /**
   * lift motor
   */
  std::shared_ptr<Motor> liftMotor{nullptr};

  /**
   * lift controller
   */
  std::shared_ptr<AsyncPosPIDController> liftController{nullptr};

  /**
   * @brief      Initializes the object. (run before loop)
   */
  virtual void initialize();

  /**
   * @brief      this will loop in a task
   */
  void loop() override;

  /**
   * lift singleton
   */
  static Lift *lift;

  public:
  /**
   * @brief      Gets the lift.
   *
   * @return     The lift.
   */
  static Lift *getLift();
};