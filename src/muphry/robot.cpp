#include "muphry/robot.hpp"

//controller
std::shared_ptr<Controller> master;

//controller buttons
std::shared_ptr<ControllerButton> intakeUp;
std::shared_ptr<ControllerButton> intakeDown;
std::shared_ptr<ControllerButton> tilterUpBtn;
std::shared_ptr<ControllerButton> tilterDownBtn;
std::shared_ptr<ControllerButton> liftUp;
std::shared_ptr<ControllerButton> liftMid;

//screen
std::shared_ptr<GUI::Screen> screen;
GUI::Selector* selector;
GUI::Actions* intakeActions;
GUI::Actions* liftActions;
GUI::Actions* tilterActions;
