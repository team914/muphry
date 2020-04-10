/*
 * @author Michael Jones, 914M, UTAH
 */
#include "okapi/api.hpp"
#include "api.h"
#include "muphry/robot.hpp"

#include <string>
#include <initializer_list>

using namespace okapi;
using namespace okapi::literals;
/**
 * @brief Asyn Kinematics Logger
 * @details This is designed off of the bases of a single input (encoder) and will calculate the derivatives of the position up to snap 
 *              and print them in a CSV format. Copy and paste the data into your own spreadsheet or use my template here:
 *              https://docs.google.com/spreadsheets/d/1RLpmAHBv_iURuAgM1_kqToKQe8_21qRe4rFjFW4wzTs/edit?usp=sharing
 * 
 */
class AsyncKinematicsLog{
    public:
    /**
     * @brief      Constructs a new instance of the AsyncKinematicsLogger.
     *
     * @param[in]  iencoder        The encoder from a motor or ADI
     * @param[in]  iwheelDiameter  The wheel diameter
     * @param[in]  itpr            The tpr(turns per rotation of the encoder see okapi's mathUtil)
     * @param[in]  iratio          The iratio(ratio of the gearing see okapi's ChassisScales)
     * @param[in]  isampleTime     The isample time(sample rate of ADIPorts is 10 milliseconds)
     * @param[in]  iloopDtTimer    The iloop dt timer(timer)
     * @param[in]  ilogger         The ilogger(logger)
     */
    AsyncKinematicsLog(
        std::shared_ptr<ContinuousRotarySensor> iencoder,
        QLength iwheelDiameter,
        const double& itpr,
        const double& iratio = 1,
        QTime isampleTime = 10_ms,
        std::unique_ptr<AbstractTimer> iloopDtTimer = std::make_unique<Timer>(),
        std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    /**
     * @brief      Starts a thread. Nothing will happen if you don't do this.
     *
     * @param[in]  ithreadId  The ithread identifier
     */
    void startThread(const std::string& ithreadId = "asyncKinematicsLog");

    /**
     * @brief      Starts a thread. Nothing will happen if you don't do this.
     *
     * @param[in]  ithreadId      The ithread identifier
     * @param[in]  taskPriority   The task priority
     * @param[in]  taskTaskDepth  The task task depth
     */
    void startThread(const std::string& ithreadId, const int& taskPriority, const int& taskTaskDepth);

    /**
     * @brief      Disable or Enable whether the message is logged
     *
     * @param[in]  iisDisabled  bool for whether the logger will log
     */
    void flipDisable(const bool& iisDisabled);

    /**
     * @brief      Determines if disabled.
     *
     * @return     True if disabled, False otherwise.
     */
    bool isDisabled();

    /**
     * @brief      Gets the current speed.
     *
     * @return     The current speed.
     */
    double getCurrentSpeed();

    /**
     * @brief      Gets the current acceleration.
     *
     * @return     The current acceleration.
     */
    double getCurrentAcceleration();

    /**
     * @brief      Gets the current jerk.
     *
     * @return     The current jerk.
     */
    double getCurrentJerk();

    /**
     * @brief      Gets the current snap.
     *
     * @return     The current snap.
     */
    double getCurrentSnap();

    /**
     * @brief      Gets the encoder.
     *
     * @return     The encoder.
     */
    std::shared_ptr<ContinuousRotarySensor> getEncoder();

    /**
     * @brief      Gets the log message.
     *
     * @return     The log message.
     */
    std::string getLogMessage();

    /**
     * @brief      Gets the column headers for a spreadsheet to graph.
     *
     * @return     The column headers.
     */
    std::string getColumnHeaders();

    /**
     * @brief      Gets the raw values from the constructor.
     *
     * @return     The raw values.
     */
    std::string getRawValues();

    /**
     * @brief      Gets the tpr of the encoder.
     *
     * @return     The tpr.
     */
    double getTPR();

    /**
     * @brief      Gets the ratio of the encoder.
     *
     * @return     The ratio.
     */
    double getRatio();

    /**
     * @brief      Gets the wheel diameter.
     *
     * @return     The wheel diameter.
     */
    QLength getWheelDiameter();

    /**
     * @brief      Gets the sample time.
     *
     * @return     The sample time.
     */
    QTime getSampleTime();

    /**
     * @brief      Sets the tpr (see okapi's mathUtil.hpp).
     *
     * @param[in]  itpr  The itpr
     */
    void setTPR(const double& itpr);

    /**
     * @brief      Sets the ratio (see okapi's chassisScales).
     *
     * @param[in]  iratio  The iratio
     */
    void setRatio(const double& iratio);

    /**
     * @brief      Sets the wheel diameter (e.g. 4_in).
     *
     * @param[in]  iwheelDiameter  The iwheel diameter
     */
    void setWheelDiameter(QLength iwheelDiameter);

    /**
     * @brief      Sets the sample time (e.g. 10_ms).
     *
     * @param[in]  isampleTime  The isample time
     */
    void setSampleTime(QTime isampleTime);

    /**
     * @brief      If you see that the data is too unstable, then using okapi's filters can help solve this.
     *             The default filters are AverageFilters<10>. Sets the filters.
     *
     * @param[in]  ifilterList  The ifilter list
     */
    void setFilters(const std::initializer_list<std::shared_ptr<Filter>>& ifilterList );

    /**
     * @brief      Reset the Kinematics to 0. It is HIGHLY suggest to also reset the encoder before calling this,
     *             but not necessary.
     */
    void resetKinematics();
    
    private:
    pros::task_t task{NULL};

    /**
     * @brief      This is the task_fnc used to run the loop. This method is called a trampoline.
     *
     * @param      params  The parameters
     */
    static void task_fnc(void* params);

    /**
     * @brief      The main loop for the task.
     */
    void loop();

    bool disabled{false};
    double dt{0};
    double pos{0}, lastPos{0};
    double speed{0}, lastSpeed{0};
    double accel{0}, lastAccel{0};
    double jerk{0}, lastJerk{0};
    double snap{0}, lastSnap{0};

    std::string msg{""};
    std::shared_ptr<ContinuousRotarySensor> encoder{nullptr};
    double tpr{imev5GreenTPR};
    double ratio{1};
    double sampleTime{0};
    double wheelDiameter{0};
    std::unique_ptr<AbstractTimer> loopDtTimer{nullptr};

    std::shared_ptr<Filter> posFilter{nullptr};
    std::shared_ptr<Filter> speedFilter{nullptr};
    std::shared_ptr<Filter> accelFilter{nullptr};
    std::shared_ptr<Filter> jerkFilter{nullptr};
    std::shared_ptr<Filter> snapFilter{nullptr};

    std::shared_ptr<Logger> logger;
};