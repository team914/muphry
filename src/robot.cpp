#include "robot/robot.hpp"

#include <memory>

using namespace okapi;

namespace Robot{

void initRobot(){
  intake = std::make_shared<MotorGroup>( MotorGroup({5, -6}) );
  tray = std::make_shared<MotorGroup>( MotorGroup({1}));
  other = nullptr;

  intake->setGearing(AbstractMotor::gearset::red);
  tray->setGearing(AbstractMotor::gearset::red);

  chassis = ChassisControllerBuilder()
    .withMotors( {-9, -10}, {2,3} )
    .withSensors( ADIEncoder(1,2), ADIEncoder(4,5), ADIEncoder(7,8) )
    .withDimensions( AbstractMotor::GearsetRatioPair{ AbstractMotor::gearset::green }, { 10.2101761242_in, 10.5_in } )
    .build();

  model = chassis->getModel();
  odom = std::make_shared<CustomOdometry>(model, chassis->getChassisScales());
}

std::shared_ptr<MotorGroup> intake = nullptr;
std::shared_ptr<MotorGroup> tray = nullptr;
std::shared_ptr<MotorGroup> other = nullptr;

std::shared_ptr<ChassisController> chassis = nullptr;
std::shared_ptr<ChassisModel> model = nullptr;
std::shared_ptr<CustomOdometry> odom = nullptr;

}
