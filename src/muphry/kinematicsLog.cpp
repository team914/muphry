#include "kinematicsLog.hpp"

AsyncKinematicsLog::AsyncKinematicsLog(
    std::shared_ptr<ContinuousRotarySensor> iencoder,
    AbstractMotor::GearsetRatioPair ipair,
    QLength iwheelDiameter,
    QTime isampleTime,
    std::unique_ptr<AbstractTimer> iloopDtTimer,
    std::shared_ptr<Logger> ilogger) :
        encoder(std::move(iencoder)),
        pair(ipair),
        wheelDiameter(wheelDiameter),
        sampleTime(isampleTime.convert(second)),
        loopDtTimer(std::move(iloopDtTimer)),
        logger(std::move(ilogger)){}


void AsyncKinematicsLog::startThread(const std::string& ithreadId){
    startThread(ithreadId, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
}
void AsyncKinematicsLog::startThread(const std::string& ithreadId, 
        const int& taskPriority, const int& taskStackDepth){
    task = pros::c::task_create(task_fnc, NULL, taskPriority,
        taskStackDepth, ithreadId.c_str());
}
void AsyncKinematicsLog::flipDisable(const bool& iisDisabled){
    disabled = iisDisabled;
}
bool AsyncKinematicsLog::isDisabled(){
    return disabled;
}
double AsyncKinematicsLog::getCurrentSpeed(){
    return speed;
}
double AsyncKinematicsLog::getCurrentAcceleration(){
    return accel;
}
double AsyncKinematicsLog::getCurrentJerk(){
    return jerk;
}
double AsyncKinematicsLog::getCurrentSnap(){
    return snap;
}
std::shared_ptr<ContinuousRotarySensor> AsyncKinematicsLog::getEncoder(){
    return encoder;
}
std::string AsyncKinematicsLog::getLogMessage(){
    return msg;
}
AbstractMotor::GearsetRatioPair AsyncKinematicsLog::getPair(){
    return pair;
}
QLength AsyncKinematicsLog::getWheelDiameter(){
    return wheelDiameter * meter;
}
QTime AsyncKinematicsLog::getSampleTime(){
    return sampleTime * millisecond;
}
void AsyncKinematicsLog::setPair(const AbstractMotor::GearsetRatioPair& ipair){
    pair = ipair;
}
void AsyncKinematicsLog::setWheelDiameter(QLength iwheelDiameter){
    wheelDiameter = iwheelDiameter.convert(meter);
}
void AsyncKinematicsLog::setSampleTime(QTime isampleTime){
    sampleTime = isampleTime.convert(second);
}
void AsyncKinematicsLog::resetKinematics(){
    encoder->reset();

    pos = 0, lastPos = 0;
    speed = 0, lastSpeed = 0;
    accel = 0, lastAccel = 0;
    jerk = 0, lastJerk = 0;
    snap = 0, lastSnap = 0;
}

void task_fnc(void* params){
    while(true){

    }
}