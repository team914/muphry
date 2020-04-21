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

std_msgs::Float32 posL_msg;
std_msgs::Float32 velL_msg;
std_msgs::Float32 accL_msg;
std_msgs::Float32 jrkL_msg;
std_msgs::Float32 snpL_msg;

ros::Publisher posL = ros::Publisher("posL\0", &posL_msg);
ros::Publisher velL = ros::Publisher("velL\0", &velL_msg);
ros::Publisher accL = ros::Publisher("accL\0", &accL_msg);
ros::Publisher jrkL = ros::Publisher("jrkL\0", &jrkL_msg);
ros::Publisher snpL = ros::Publisher("snpL\0", &snpL_msg);

std_msgs::Float32 posR_msg;
std_msgs::Float32 velR_msg;
std_msgs::Float32 accR_msg;
std_msgs::Float32 jrkR_msg;
std_msgs::Float32 snpR_msg;


ros::Publisher posR = ros::Publisher("posR\0", &posR_msg);
ros::Publisher velR = ros::Publisher("velR\0", &velR_msg);
ros::Publisher accR = ros::Publisher("accR\0", &accR_msg);
ros::Publisher jrkR = ros::Publisher("jrkR\0", &jrkR_msg);
ros::Publisher snpR = ros::Publisher("snpR\0", &snpR_msg);

geometry_msgs::Pose pose_msg;
ros::Publisher pose = ros::Publisher("pose\0", &pose_msg);

//chassis
std::shared_ptr<Chassis> chassis = std::make_shared<Chassis>();
