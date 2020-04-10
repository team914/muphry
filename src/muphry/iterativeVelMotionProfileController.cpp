#include "muphry/iterativeVelMotionProfileController.hpp"
#include "okapi/api/util/mathUtil.hpp"

#include <algorithm>

IterativeVelMotionProfileController::IterativeVelMotionProfileController(const TimeUtil &itimeUtil,
                                                                         const AngularLimits &ilimits,
                                                                         std::unique_ptr<VelMath> ivelMath,
                                                                         const QLength &idiameter,
                                                                         const AbstractMotor::GearsetRatioPair &ipair,
                                                                         std::shared_ptr<Logger> ilogger)
    :   velMath(std::move(ivelMath)),
        logger(std::move(ilogger)),
        loopDtTimer(itimeUtil.getTimer()),
        settledUtil(itimeUtil.getSettledUtil()),
        diameter(idiameter.convert(meter)),
        pair(ipair){
    setOutputLimits(1, -1);
    setLimits(ilimits);
    setSampleTime(10_ms);//this needs to be called after setLimits
}

void IterativeVelMotionProfileController::setControllerSetTargetLimits(double itargetMax, double itargetMin) {
  // Always use larger value as max
  if (itargetMin > itargetMax) {
    const double temp = itargetMax;
    itargetMax = itargetMin;
    itargetMin = temp;
  }

  controllerSetTargetMax = itargetMax;
  controllerSetTargetMin = itargetMin;
}

QAngularSpeed IterativeVelMotionProfileController::stepVel(const double inewReading) {
  return velMath->step(inewReading);
}

double IterativeVelMotionProfileController::step(const double inewReading) {
  if (!controllerIsDisabled) {
    loopDtTimer->placeHardMark();

    processValue = inewReading;

    if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
      stepVel(inewReading);
      error = getError();//error in raw sensor value

      //if distance left is less than or equal to the distance to slow down then slow down
      if( std::abs(error) * gearsetToTPR(pair.internalGearset) * (1 / pair.ratio) <= std::pow( getVel().convert(rpm), 2 ) / 2 * getAccel().convert(rpm / second)  ){
          std::string msg = "IterativeVelMotionProfileController: Set processValue to 0";
          LOG_INFO(msg);
          processValue = 0;
      }else{
          processValue = std::clamp(error, -maxAngularSpeed, maxAngularSpeed);
          std::string msg = "IterativeVelMotionProfileController: Set processValue to " + std::to_string(processValue);
          LOG_DEBUG(msg);
      }

      outputMin = std::clamp(
          //limit output velocity and acceleration
          getVel().convert(rpm) - (updateAngularAccel), -maxAngularSpeed, maxAngularSpeed) 
          //divide by gearset rpm to limit the output to be between -1 and 1
          / toUnderlyingType(pair.internalGearset);
      outputMax = outputMin + (updateAngularAccel * 2 / toUnderlyingType(pair.internalGearset));

      std::string msg = "IterativeVelMotionProfileController: outputMin/Max to " + std::to_string(outputMax) + " | " + std::to_string(outputMin);
      LOG_DEBUG(msg);

      loopDtTimer->clearHardMark(); // Important that we only clear if dt >= sampleTime

      settledUtil->isSettled(error);
    }

    output = std::clamp(getProcessValue(), outputMin, outputMax);
    return output;
  }

  return 0; // Can't set output to zero because the entire loop in an integral
}

void IterativeVelMotionProfileController::setTarget(const double itarget) {
    std::string msg = "IterativeVelMotionProfileController: Set target to " + std::to_string(itarget);
    LOG_INFO(msg);
    target = itarget;
}

double IterativeVelMotionProfileController::getMaxOutput() {
  return outputMax;
}

double IterativeVelMotionProfileController::getMinOutput() {
  return outputMin;
}

void IterativeVelMotionProfileController::setLimits(const AngularLimits &ilimits){
    std::string msg = "IterativeVelMotionProfileController: Set speed limit to " + std::to_string(ilimits.speed.convert(rpm)) + " and acceleration limit to  " + std::to_string(ilimits.accel.convert( rpm / second ));
    LOG_INFO(msg);
    maxAngularSpeed = ilimits.speed.convert(rpm);
    maxAngularAccel = ilimits.accel.convert( rpm / second );
}

void IterativeVelMotionProfileController::controllerSet(const double ivalue) {
  target = remapRange(ivalue, -1, 1, controllerSetTargetMin, controllerSetTargetMax);
}

double IterativeVelMotionProfileController::getTarget() {
  return target;
}

double IterativeVelMotionProfileController::getTarget() const {
  return target;
}

double IterativeVelMotionProfileController::getOutput() const {
  return isDisabled() ? 0 : output;
}

double IterativeVelMotionProfileController::getProcessValue() const{
    return processValue;
}

double IterativeVelMotionProfileController::getError() const {
  return getTarget() - getProcessValue();
}

bool IterativeVelMotionProfileController::isSettled() {
  return isDisabled() ? true : settledUtil->isSettled(error);
}

void IterativeVelMotionProfileController::setSampleTime(QTime isampleTime){
    std::string msg = "IterativeVelMotionProfileController: updateAngularAccel " + std::to_string(updateAngularAccel) + " sampleTime " + std::to_string(isampleTime.convert(millisecond)) + "maxAngularAccel" + std::to_string(maxAngularAccel);
    LOG_INFO(msg); 
    updateAngularAccel = isampleTime.convert(millisecond) / 60000 * maxAngularAccel;
    sampleTime = isampleTime;
}

void IterativeVelMotionProfileController::setOutputLimits(double imax, double imin){
    outputMax = imax;
    outputMin = imin;
}

void IterativeVelMotionProfileController::reset() {
  LOG_INFO_S("IterativeVelMotionProfileController: Reset");

  output = 0;
  target = 0;
  error = 0;
  processValue = 0;
  settledUtil->reset();
}

void IterativeVelMotionProfileController::flipDisable() {
  flipDisable(!controllerIsDisabled);
}

void IterativeVelMotionProfileController::flipDisable(const bool iisDisabled) {
  LOG_INFO("IterativeVelMotionProfileController: flipDisable " + std::to_string(iisDisabled));
  controllerIsDisabled = iisDisabled;
}

bool IterativeVelMotionProfileController::isDisabled() const {
  return controllerIsDisabled;
}

void IterativeVelMotionProfileController::setTicksPerRev(const double tpr) {
  velMath->setTicksPerRev(tpr);
}

QAngularSpeed IterativeVelMotionProfileController::getVel() const {
  return velMath->getVelocity();
}

QAngularAcceleration IterativeVelMotionProfileController::getAccel() const {
  return velMath->getAccel();
}

QTime IterativeVelMotionProfileController::getSampleTime() const {
  return sampleTime;
}
