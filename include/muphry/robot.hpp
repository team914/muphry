/**
 * @author     Acetousk
 * @date       2020
 */
#pragma once

#include "lib7842/api.hpp"
#include "main.h"
#include "muphry/subsystems/chassis.hpp"
#include "okapi/api.hpp"
#include "ros_lib/geometry_msgs/Pose2D.h"
#include "ros_lib/ros.h"
#include "ros_lib/ros/time.h"
#include "ros_lib/std_msgs/Float32.h"
#include "ros_lib/std_msgs/Time.h"
#include "ros_lib/tf/tf.h"
#include <functional>
#include <map>
#include <memory>

using namespace lib7842;
using namespace okapi;
using namespace okapi::literals;

// controller
extern std::shared_ptr<Controller> master;

// controller buttons
extern std::shared_ptr<ControllerButton> intakeUpBtn;
extern std::shared_ptr<ControllerButton> intakeDownBtn;
extern std::shared_ptr<ControllerButton> tilterUpBtn;
extern std::shared_ptr<ControllerButton> tilterDownBtn;
extern std::shared_ptr<ControllerButton> liftUpBtn;
extern std::shared_ptr<ControllerButton> liftMidBtn;

// screen
extern std::shared_ptr<GUI::Screen> screen;
extern GUI::Selector *selector;
extern GUI::Actions *intakeActions;
extern GUI::Actions *liftActions;
extern GUI::Actions *tilterActions;

// intake
const int leftIntakePort{4};
const int rightIntakePort{-7};
const AbstractMotor::gearset intakeGearset{AbstractMotor::gearset::green};
const QLength intakeDiameter{3_in};
const double intakekP{.001};
const double intakekI{0};
const double intakekD{0};

// lift
const int liftPort{15};
const AbstractMotor::gearset liftGearset{AbstractMotor::gearset::red};
const double liftkP{.005};
const double liftkI{0};
const double liftkD{.0001};
const double midTower{680};
const double lowTower{500};
const double a2CubeStack{150};
const double a3CubeStack{350};
const double a4CubeStack{550};
const double liftDown{-20};
extern bool liftToggle; // true means the lift is up false means the lift is down

// tilter
const int tilterPort{9};
const AbstractMotor::gearset tilterGearset{AbstractMotor::gearset::red};
const double tilterkP{.01};
const double tilterkI{0};
const double tilterkD{.0};
const double tilterUp{1100};
const double tilterLiftUp{100};
const double tilterDown{0};

// chassis
extern std::shared_ptr<Chassis> chassis;

// ROS
extern std::shared_ptr<ros::NodeHandle> nh;

extern std_msgs::Float32 posL_msg;
extern std_msgs::Float32 velL_msg;
extern std_msgs::Float32 accL_msg;
extern std_msgs::Float32 jrkL_msg;
extern std_msgs::Float32 snpL_msg;

extern ros::Publisher posL;
extern ros::Publisher velL;
extern ros::Publisher accL;
extern ros::Publisher jrkL;
extern ros::Publisher snpL;

extern std_msgs::Float32 posR_msg;
extern std_msgs::Float32 velR_msg;
extern std_msgs::Float32 accR_msg;
extern std_msgs::Float32 jrkR_msg;
extern std_msgs::Float32 snpR_msg;

extern ros::Publisher posR;
extern ros::Publisher velR;
extern ros::Publisher accR;
extern ros::Publisher jrkR;
extern ros::Publisher snpR;

extern geometry_msgs::Pose pose_msg;
extern ros::Publisher pose;
