#pragma once

#include "okapi/api.hpp"
#include "muphry/angularLimits.hpp"
#include <memory>

using namespace okapi;
using namespace okapi::literals;

class IterativeVelMotionProfileController : IterativeVelocityController<double, double> {
  public:
  IterativeVelMotionProfileController(
    const TimeUtil &itimeUtil,
    const AngularLimits &ilimits,
    std::unique_ptr<VelMath> ivelMath,
    const QLength &idiameter,
    const AbstractMotor::GearsetRatioPair &ipair,
    std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

  /**
   * Do one iteration of the controller. Returns the reading in the range [-1, 1] unless the
   * bounds have been changed with setOutputLimits().
   *
   * @param inewReading new raw sensor measurement
   * @return controller output
   */
  double step(double inewReading) override;

  /**
   * Sets the target for the controller.
   *
   * @param itarget new target sensor value
   */
  void setTarget(double itarget) override;

  void setLimits(const AngularLimits &ilimits);

  /**
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller. The range of input values is expected to be [-1, 1].
   *
   * @param ivalue the controller's output in the range [-1, 1]
   */
  void controllerSet(double ivalue) override;

  /**
   * Gets the last set target, or the default target if none was set.
   *
   * @return the last target
   */
  double getTarget() override;

  /**
   * Gets the last set target, or the default target if none was set.
   *
   * @return the last target
   */
  double getTarget() const;

  /**
   * Returns the last calculated output of the controller.
   */
  double getOutput() const override;

  /**
   * Get the upper output bound.
   *
   * @return  the upper output bound
   */
  double getMaxOutput() override;

  /**
   * Get the lower output bound.
   *
   * @return the lower output bound
   */
  double getMinOutput() override;

  /**
   * @return The most recent value of the process variable.
   */
  double getProcessValue() const override;

  /**
   * Returns the last error of the controller. Does not update when disabled.
   */
  double getError() const override;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * If the controller is disabled, this method must return true.
   *
   * @return whether the controller is settled
   */
  bool isSettled() override;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops
   */
  void setSampleTime(QTime isampleTime) override;

  /**
   * Set controller output bounds. Default bounds are [-1, 1].
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(double imax, double imin) override;

  /**
   * Sets the (soft) limits for the target range that controllerSet() scales into. The target
   * computed by controllerSet() is scaled into the range [-itargetMin, itargetMax].
   *
   * @param itargetMax The new max target for controllerSet().
   * @param itargetMin The new min target for controllerSet().
   */
  void setControllerSetTargetLimits(double itargetMax, double itargetMin) override;

  /**
   * Resets the controller's internal state so it is similar to when it was first initialized, while
   * keeping any user-configured information.
   */
  void reset() override;

  /**
   * Changes whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  void flipDisable() override;

  /**
   * Sets whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   *
   * @param iisDisabled whether the controller is disabled
   */
  void flipDisable(bool iisDisabled) override;

  /**
   * Returns whether the controller is currently disabled.
   *
   * @return whether the controller is currently disabled
   */
  bool isDisabled() const override;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  QTime getSampleTime() const override;

  /**
   * Do one iteration of velocity calculation.
   *
   * @param inewReading new measurement
   * @return filtered velocity
   */
  virtual QAngularSpeed stepVel(double inewReading);

  /**
   * Sets the number of encoder ticks per revolution. Default is 1800.
   *
   * @param tpr number of measured units per revolution
   */
  virtual void setTicksPerRev(double tpr);

  /**
   * Returns the current velocity.
   */
  virtual QAngularSpeed getVel() const;

  /**
   * Returns the current acceleration.
   */
  virtual QAngularAcceleration getAccel() const;

  protected:
  std::shared_ptr<Logger> logger{nullptr};

  QTime sampleTime{10_ms};
  double updateAngularAccel{0};
  AbstractMotor::GearsetRatioPair pair;
  double maxAngularSpeed{0};
  double maxAngularAccel{0};
  double diameter{0};
  double error{0};
  double target{0};
  double processValue{0};
  double output{0};
  double outputMax{1};
  double outputMin{-1};
  double controllerSetTargetMax{1};
  double controllerSetTargetMin{-1};
  bool controllerIsDisabled{false};

  std::unique_ptr<VelMath> velMath{nullptr};
  std::unique_ptr<AbstractTimer> loopDtTimer{nullptr};
  std::unique_ptr<SettledUtil> settledUtil{nullptr};

};