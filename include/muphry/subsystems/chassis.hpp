#pragma once

#include "okapi/api.hpp"
#include "lib7842/api/other/taskWrapper.hpp"
#include "muphry/statemachine.hpp"
#include "muphry/robot.hpp"

#include <math.h>

using namespace okapi;
using namespace lib7842;

enum class ChassisState{
    PID,
    integrated,
    linearProfile,
    profile,
    odomPID,
    purePursuit,
    driver,
    off
};

class Chassis : public StateMachine<ChassisState, ChassisState::off> {
    protected:
    Chassis() = default;

    virtual void initialize();

    void loop() override;

    static Chassis* chassis;

    //chassis models and controllers
    std::shared_ptr<ThreeEncoderSkidSteerModel> skidSteerModel{nullptr};
    std::shared_ptr<ThreeEncoderXDriveModel> holonomicModel{nullptr};

    std::shared_ptr<ChassisControllerPID> pidController{nullptr};
    std::shared_ptr<ChassisControllerIntegrated> integratedController{nullptr};
    std::shared_ptr<AsyncLinearMotionProfileController> leftProfileController{nullptr};
    std::shared_ptr<AsyncLinearMotionProfileController> rightProfileController{nullptr};
    std::shared_ptr<AsyncMotionProfileController> profileController{nullptr};

    std::shared_ptr<CustomOdometry> odom{nullptr};

    std::shared_ptr<OdomController> odomController{nullptr};
    std::shared_ptr<OdomXController> odomXController{nullptr};

    std::shared_ptr<PathFollower> pathFollowerController{nullptr};
    std::shared_ptr<PathFollowerX> pathFollowerXController{nullptr};

    //helping methods
    void resetModels();
    void resetControllers();
    void resetOdom();

    std::shared_ptr<ThreeEncoderSkidSteerModel> makeSkidSteerModel();
    std::shared_ptr<ThreeEncoderXDriveModel> makeHolonomicModel();

    std::shared_ptr<ChassisControllerPID> makePidController();
    std::shared_ptr<ChassisControllerIntegrated> makeIntegratedController();
    std::shared_ptr<AsyncLinearMotionProfileController> makeLeftProfileController();
    std::shared_ptr<AsyncLinearMotionProfileController> makeRightProfileController();
    std::shared_ptr<AsyncMotionProfileController> makeProfileController();

    std::shared_ptr<CustomOdometry> makeOdom();

    std::shared_ptr<OdomController> makeOdomController();
    std::shared_ptr<OdomXController> makeOdomXController();

    std::shared_ptr<PathFollower> makePathFollowerController();
    std::shared_ptr<PathFollowerX> makePathFollowerXController();

    double forward{0};
    double right{0};
    double yaw{0};

    bool modelType{false};

    public:
    static Chassis* getChassis();

    void setModelType( bool isHolonomic = false );
    void chassisDriver( double iforward, double iright, double iyaw );

    template<typename Model>
    Model getModel(){
        if(!modelType){
            return skidSteerModel;
        }else{
            return holonomicModel;
        }
    }

    template<typename Controller>
    Controller getController(){
        if(isDone()){
            switch(state){
                case ChassisState::PID:
                    if(pidController){
                        return pidController;
                    }
                    setDone();
                break;
                case ChassisState::integrated:
                    if(integratedController){
                        return integratedController;
                    }
                    setDone();
                break;
                case ChassisState::linearProfile:
                    if(leftProfileController && rightProfileController){
                        return std::tuple<Controller,Controller>(leftProfileController,leftProfileController);
                    }
                    setDone();
                break;
                case ChassisState::profile:
                    if(profileController){
                        return profileController;
                    }
                    setDone();
                break;              
                case ChassisState::odomPID:
                    if(!modelType){
                        if(odomController){
                            return odomController;
                        }
                    }else{
                        if(odomXController){
                            return odomXController;
                        }
                    }
                    setDone();
                break;              
                case ChassisState::purePursuit:
                    if(!modelType){
                        if(pathFollowerController){
                            return pathFollowerController;
                        }
                    }else{
                        if(pathFollowerXController){
                            return pathFollowerXController;
                        }
                    }
                    setDone();
                break;                      
                case ChassisState::driver:
                    printf("driver: forward %d, right %d, yaw %d\n", forward, right, yaw);
                    if(!modelType){
                        if(skidSteerModel){
                            skidSteerModel->driveVectorVoltage(forward,yaw);
                        }
                    }else{
                        if(holonomicModel){
                            holonomicModel->xArcade(right,forward,yaw);
                        }
                    }
                    setDone();
                break;
                case ChassisState::off:
                    printf("off\n");
                    setDone();
                break;
            }

        }
    }    

};
