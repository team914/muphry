/*
 * @author Michael Jones, 914M, UTAH
 */
#include "asyncKinematicsLog/asyncKinematicsLog.hpp"
#include <iostream>

AsyncKinematicsLog::AsyncKinematicsLog(
    std::shared_ptr<ContinuousRotarySensor> iencoder,
    QLength iwheelDiameter,
    const double& itpr,
    const double& iratio,
    QTime isampleTime,
    std::unique_ptr<AbstractTimer> iloopDtTimer,
    std::shared_ptr<Logger> ilogger) :
        encoder(std::move(iencoder)),
        tpr(itpr),
        ratio(iratio),
        wheelDiameter(iwheelDiameter.convert(meter)),
        sampleTime(isampleTime.convert(second)),
        loopDtTimer(std::move(iloopDtTimer)),
        logger(std::move(ilogger)){
    /**
     * These filters work to make the output smooth when graphing it,
     * however it does make the output less precise in accordance with 
     * what it outputs.
     */
    posFilter   = std::make_shared<AverageFilter<10>>();
    speedFilter = std::make_shared<AverageFilter<10>>();
    accelFilter = std::make_shared<AverageFilter<10>>();
    jerkFilter  = std::make_shared<AverageFilter<10>>();
    snapFilter  = std::make_shared<AverageFilter<10>>();
}

void AsyncKinematicsLog::startThread(const std::string& ithreadId){
    startThread(ithreadId, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
}
void AsyncKinematicsLog::startThread(const std::string& ithreadId, 
        const int& taskPriority, const int& taskStackDepth){
    LOG_INFO_S("AsyncKinematicsLog: starting thread");
    task = pros::c::task_create(task_fnc, this, taskPriority,
        taskStackDepth, ithreadId.c_str());
}
void AsyncKinematicsLog::flipDisable(const bool& iisDisabled){
    std::string msg = "AsyncKinematicsLog: flipping isDisabled to " + 
        std::to_string(iisDisabled);
    LOG_DEBUG_S(msg);
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
double AsyncKinematicsLog::getTPR(){
    return tpr;
}
double AsyncKinematicsLog::getRatio(){
    return ratio;
}
QLength AsyncKinematicsLog::getWheelDiameter(){
    return wheelDiameter * meter;
}
QTime AsyncKinematicsLog::getSampleTime(){
    return sampleTime * millisecond;
}
void AsyncKinematicsLog::setTPR(const double& itpr){
    tpr = itpr;
}
void AsyncKinematicsLog::setRatio(const double& iratio){
    ratio = iratio;
}
void AsyncKinematicsLog::setWheelDiameter(QLength iwheelDiameter){
    wheelDiameter = iwheelDiameter.convert(meter);
}
void AsyncKinematicsLog::setSampleTime(QTime isampleTime){
    sampleTime = isampleTime.convert(second);
}
void AsyncKinematicsLog::resetKinematics(){
    LOG_DEBUG_S("AsyncKinematicsLog: Resetting Kinematics");
    if(encoder->get()!=0){
        LOG_WARN_S("AsyncKinematicsLog: Encoder Not Reset");
    }

    pos = 0, lastPos = 0;
    speed = 0, lastSpeed = 0;
    accel = 0, lastAccel = 0;
    jerk = 0, lastJerk = 0;
    snap = 0, lastSnap = 0;
}

void AsyncKinematicsLog::task_fnc(void* params){
    if(!params) throw std::runtime_error("AsyncKinematicsLog::task_fnc: param is null");
    static_cast<AsyncKinematicsLog*>(params)->loop();
}

std::string AsyncKinematicsLog::getColumnHeaders(){
    return std::string("0, pos(meters)  ,       speed(mps)      ,      accel(mps2)      ,       jerk(mps3)      ,       snap(mps4)      ,     time(seconds)\n");
}

std::string AsyncKinematicsLog::getRawValues(){
    return std::string( "tpr,\t" + std::to_string(tpr) + "\tratio,\t" + std::to_string(ratio) + 
        "\twheelDiameter,\t" + std::to_string(wheelDiameter) + "\tsampleTime\t" + 
        std::to_string(sampleTime) +"\n");    
}

void AsyncKinematicsLog::loop(){
    int cnt = 1;

    pros::delay(sampleTime);
    resetKinematics();

    std::cout << getRawValues();
    std::cout << getColumnHeaders();
 
    std::ofstream f;
    f.open("/usd/test.txt", std::ios::app);
    f <<getValue1() << ","<<getValue2()<<std::endl;
    f.close();

    while(true){
        //get delta time
        dt = loopDtTimer->getDtFromMark().convert(second);

        //get position from encoder feed back and convert to meters
        pos = posFilter->filter(encoder->get()) / (tpr * ratio) * wheelDiameter * PI;

        //this will make it so that the sensors don't send crazy feedback at program beginning
        if(std::abs(pos) >= 1000){resetKinematics();}

        //calculate derivatives of speed to 4th degree
        if(lastPos != 0){speed = speedFilter->filter((pos-lastPos)/ dt);}
        if(lastSpeed != 0){accel = accelFilter->filter((speed-lastSpeed)/dt);}
        if(lastAccel != 0){jerk = jerkFilter->filter((accel-lastAccel)/dt);}
        if(lastJerk != 0){snap = snapFilter->filter((jerk-lastJerk)/dt);}

        //generate message with comma seperated values
		msg = std::to_string(cnt) + ", " + 
			std::to_string(pos) + "\t,\t" + 
			std::to_string(speed) + "\t,\t" + std::to_string(accel) +
			"\t,\t" + std::to_string(jerk) + "\t,\t" + 
			std::to_string(snap) + "\t,\t" + std::to_string(
            loopDtTimer->getDtFromStart().convert(second)) +"\n";

        //this will not log by default when resetting sensors and will not log if disabled
        if(pos != 0 && !disabled){
            cnt++;
            std::cout << getLogMessage();
        }

        //update last derivative each loop
		lastPos = pos;
		lastSpeed = speed;
		lastAccel = accel;
        lastJerk = jerk;
		lastSnap = snap;

        //place mark for timer
		loopDtTimer->placeMark();

        //delay(sample time is in seconds so need to convert it to milliseconds)
        pros::delay(sampleTime*1000);
    }
}