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
 * @brief      This class describes a tilter state.
 */
enum class TilterState { up, liftUp, down, off };

/**
 * @brief      This class describes a tilter.
 */
class Tilter : public StateMachine<TilterState, TilterState::off> {
  private:
  /**
   * @brief      Constructs a new instance.
   */
  Tilter();

  /**
   * tilter motor
   */
  std::shared_ptr<Motor> tilterMotor{nullptr};

  /**
   * @brief      Initializes the object. (runs before loop)
   */
  virtual void initialize();

  /**
   * @brief      this will loop in a task
   */
  void loop() override;

  /**
   * tilter singleton
   */
  static Tilter *tilter;

  public:
  /**
   * tilter controller
   */
  std::shared_ptr<AsyncPosPIDController> tilterController{nullptr};

  /**
   * @brief      Gets the tilter.
   *
   * @return     The tilter.
   */
  static Tilter *getTilter();
};