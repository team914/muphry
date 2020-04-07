#include "okapi/api.hpp"
#include "api.h"

using namespace okapi;
using namespace okapi::literals;

class AsyncKinematicsLog{
    public:
    AsyncKinematicsLog(
        std::shared_ptr<ContinuousRotarySensor> iencoder,
        AbstractMotor::GearsetRatioPair ipair,
        QLength iwheelDiameter,
        QTime isampleTime = 10_ms,
        std::unique_ptr<AbstractTimer> iloopDtTimer = std::make_unique<Timer>(),
        std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    void startThread(const std::string& ithreadId);
    void startThread(const std::string& ithreadId, const int& taskPriority,
        const int& taskTaskDepth);
    void flipDisable(const bool& iisDisabled);
    bool isDisabled();

    double getCurrentSpeed();
    double getCurrentAcceleration();
    double getCurrentJerk();
    double getCurrentSnap();

    std::shared_ptr<ContinuousRotarySensor> getEncoder();
    std::string getLogMessage();

    AbstractMotor::GearsetRatioPair getPair();
    QLength getWheelDiameter();
    QTime getSampleTime();

    void setPair(const AbstractMotor::GearsetRatioPair& ipair);
    void setWheelDiameter(QLength iwheelDiameter);
    void setSampleTime(QTime isampleTime);

    void resetKinematics();
    
    private:
    pros::task_t task{NULL};
    static void task_fnc(void* params);

    bool disabled{false};
    double pos{0}, lastPos{0};
    double speed{0}, lastSpeed{0};
    double accel{0}, lastAccel{0};
    double jerk{0}, lastJerk{0};
    double snap{0}, lastSnap{0};

    std::string msg{""};
    std::shared_ptr<ContinuousRotarySensor> encoder{nullptr};
    AbstractMotor::GearsetRatioPair pair{AbstractMotor::gearset::green, 1};
    double sampleTime{0};
    double wheelDiameter{0};
    std::unique_ptr<AbstractTimer> loopDtTimer{nullptr};

    std::shared_ptr<Logger> logger;
};