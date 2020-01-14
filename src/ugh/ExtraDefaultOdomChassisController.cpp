#include "ugh/ExtraDefaultOdomChassisController.hpp"

ExtraDefaultOdomChassisController::ExtraDefaultOdomChassisController(
        const std::shared_ptr<DefaultOdomChassisController> ichassis):
            DefaultOdomChassisController(ichassis)
            controller(ichassis){
    }
    
std::unique_ptr<Odometry> ExtraDefaultOdomChassisController::getOdometry(){
    return odom;
}