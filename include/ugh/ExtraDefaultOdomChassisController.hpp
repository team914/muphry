#include "okapi/api.hpp"

using namespace okapi;

class ExtraDefaultOdomChassisController: public DefaultOdomChassisController{

    ExtraDefaultOdomChassisController(
        const std::shared_ptr<DefaultOdomChassisController> &ichassis);
    
    std::unique_ptr<Odometry> getOdometry();

    protected:
    std::shared_ptr<DefaultOdomChassisController> controller;
};