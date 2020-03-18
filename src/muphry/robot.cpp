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