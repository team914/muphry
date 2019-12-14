#include "robot/modes/drive.hpp"

namespace Drive{

Controller master = Controller();

chassis_t chassisType = cheesy;

ControllerButton trayPositive  {ControllerDigital::L1};
ControllerButton trayNegative  {ControllerDigital::L2};
ControllerButton intakePositive{ControllerDigital::R1};
ControllerButton intakeNegative{ControllerDigital::R2};
ControllerButton otherPositive {ControllerDigital::up};
ControllerButton otherNegative {ControllerDigital::down};

double deadzonePct = .1;
double cheesyPct = .75;

void runTray(double pct){
    if( Robot::tray != nullptr ){
        if( (trayPositive.isPressed() && trayNegative.isPressed()) || (!trayPositive.isPressed() && !trayNegative.isPressed()) ){
            //set tray velocity to 0
            Robot::tray->moveVelocity(0);
        }else if( trayPositive.isPressed() ){
            //set tray velocity to maxVelocity * pct
            Robot::tray->moveVelocity(pct);
        }else if( trayNegative.isPressed() ){
            //set tray velocity to -1 * maxVelocity * pct
            Robot::tray->moveVelocity(-pct);
        }
    }
}

void runIntake(double pct){
    if( Robot::intake != nullptr ){
        if( (intakePositive.isPressed() && intakeNegative.isPressed()) || (!intakePositive.isPressed() && !intakeNegative.isPressed()) ){
            //set intake velocity to 0
            Robot::intake->moveVelocity(0);
        }else if( intakePositive.isPressed() ){
            //set intake velocity to maxVelocity * pct
            Robot::intake->moveVelocity(pct);
        }else if( intakeNegative.isPressed() ){
            //set trintakeay velocity to -1 * maxVelocity * pct
            Robot::intake->moveVelocity(-pct);
        }
    }
}

void runOther(double pct){
    if( Robot::other != nullptr ){
        if( (otherPositive.isPressed() && otherNegative.isPressed()) || (!otherPositive.isPressed() && !otherNegative.isPressed()) ){
            //set other velocity to 0
            Robot::other->moveVelocity(0);
        }else if( otherPositive.isPressed() ){
            //set other velocity to maxVelocity * pct
            Robot::other->moveVelocity(pct);
        }else if( otherNegative.isPressed() ){
            //set other velocity to -1 * maxVelocity * pct
            Robot::other->moveVelocity(-pct);
        }
    }
}

void runChassis(double pct, bool rightHanded ){
    if( Robot::model != nullptr ){
        double left = 0;
        double right = 0;
        switch (chassisType){
            case tank:
            left = master.getAnalog(ControllerAnalog::leftY);
            right = master.getAnalog(ControllerAnalog::rightY);
            break;
            case arcade:
            if( rightHanded ){
                double forward = master.getAnalog(ControllerAnalog::rightY);
                double turn = master.getAnalog(ControllerAnalog::rightX);
                left = forward + turn;
                right = forward - turn;
            }else if( !rightHanded ){
                double forward = master.getAnalog(ControllerAnalog::leftY);
                double turn = master.getAnalog(ControllerAnalog::leftX);
                left = forward + turn;
                right = forward - turn;
            }
            break;
            case split_arcade:
            if( rightHanded ){
                double forward = master.getAnalog(ControllerAnalog::rightY);
                double turn = master.getAnalog(ControllerAnalog::leftX);
                left = forward + turn;
                right = forward - turn;
            }else if( !rightHanded ){
                double forward = master.getAnalog(ControllerAnalog::leftY);
                double turn = master.getAnalog(ControllerAnalog::rightX);
                left = forward + turn;
                right = forward - turn;
            }
            break;
            case cheesy:
            if( rightHanded ){
                double forward = master.getAnalog(ControllerAnalog::rightY);
                double turn = master.getAnalog(ControllerAnalog::leftX);
                if( std::abs(forward) <= deadzonePct){
                    left = turn;
                    right = -turn;
                }else{
                    left = forward + (turn * cheesyPct);
                    right = forward - (turn * cheesyPct);
                }
            }else if( !rightHanded ){
                double forward = master.getAnalog(ControllerAnalog::leftY);
                double turn = master.getAnalog(ControllerAnalog::rightX);
                if( std::abs(forward) <= deadzonePct){
                    left = turn;
                    right = -turn;
                }else{
                    left = forward + (turn * cheesyPct);
                    right = forward - (turn * cheesyPct);
                }
            }
            break;
        }
        Robot::model->tank(left,right);
    }
}

void trayMacro(){
    
}

}//Drive
