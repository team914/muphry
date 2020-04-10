#include "muphry/robot.hpp"

//controller
std::shared_ptr<Controller> master;

//controller buttons
std::shared_ptr<ControllerButton> intakeUpBtn;
std::shared_ptr<ControllerButton> intakeDownBtn;
std::shared_ptr<ControllerButton> tilterUpBtn;
std::shared_ptr<ControllerButton> tilterDownBtn;
std::shared_ptr<ControllerButton> liftUpBtn;
std::shared_ptr<ControllerButton> liftMidBtn;

//screen
std::shared_ptr<GUI::Screen> screen;
GUI::Selector* selector;
GUI::Actions* intakeActions;
GUI::Actions* liftActions;
GUI::Actions* tilterActions;

//lift
bool liftToggle = true;

//ROS
std::shared_ptr<ros::NodeHandle>  nh = std::make_shared<ros::NodeHandle>();
std_msgs::Float64 ros_msg;
ros::Publisher chatter = ros::Publisher("chatter\0", &ros_msg);

//chassis
std::shared_ptr<Chassis> chassis = std::make_shared<Chassis>();
