#include "muphry/subsystems/intake.hpp"

Intake::Intake(        
        int leftPort,
        int rightPort,
        QLength idiameter,
        double ikP,
        double ikI,
        double ikD){

    //declare diameter
    circumference = idiameter.convert(meter) * PI;
    
    //declare intake motors
    leftIntake = std::make_shared<Motor>(leftPort);
    leftIntake->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    rightIntake = std::make_shared<Motor>(rightPort);
    rightIntake->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    //declare intake controllers
    leftController = std::make_shared<AsyncPosPIDController>(
        leftIntake->getEncoder(),
        leftIntake,
        TimeUtilFactory().create(),
        ikP,
        ikI,
        ikD
    );
    leftController->startThread();

    rightController = std::make_shared<AsyncPosPIDController>(
        rightIntake->getEncoder(),
        rightIntake,
        TimeUtilFactory().create(),
        ikP,
        ikI,
        ikD
    );
    rightController->startThread();    

}

void Intake::initialize(){}

void Intake::loop(){
    while(true){

        double target = (target + rightController->getTarget()) / 2;

        switch(state){
            case IntakeState::inFull:
                leftIntake->setVoltageLimit(12000);
                rightIntake->setVoltageLimit(12000);

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                leftController->setTarget( target + 1000 );
                rightController->setTarget( target + 1000 );

                setDone();
            break;
            case IntakeState::outFull:
                leftIntake->setVoltageLimit(12000);
                rightIntake->setVoltageLimit(12000);

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                leftController->setTarget( target - 1000 );
                rightController->setTarget( target - 1000 );

                setDone();
            break;
            case IntakeState::inHalf:
                leftIntake->setVoltageLimit(6000);
                rightIntake->setVoltageLimit(6000);

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                leftController->setTarget( target + 1000 );
                rightController->setTarget( target + 1000 );

                setDone();
            break;
            case IntakeState::outHalf:
                leftIntake->setVoltageLimit(6000);
                rightIntake->setVoltageLimit(6000);

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                leftController->setTarget( target - 1000 );
                rightController->setTarget( target - 1000 );

                setDone();
            break;
            case IntakeState::moveDistance:
                leftIntake->setVoltageLimit(12000);
                rightIntake->setVoltageLimit(12000);

                leftController->setTarget( leftController->getTarget() + distance / circumference / 360 );
                rightController->setTarget(  rightController->getTarget() + distance / circumference / 360 );

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                auto time = TimeUtilFactory().create();

                bool exit = false;
                while( !exit ){
                    if( !leftController->isSettled() || !rightController->isSettled() ){
                        if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000 ){
                            exit = true;
                        }
                    }else{
                        exit = true;
                    }
                    pros::delay(20);
                }
            break;
            case IntakeState::hold:
                leftIntake->setVoltageLimit(12000);
                rightIntake->setVoltageLimit(12000);

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                leftController->setTarget( target );
                rightController->setTarget( target );

                setDone();            
            break;
            default:
                leftIntake->tarePosition();
                rightIntake->tarePosition();

                leftController->flipDisable(true);
                rightController->flipDisable(true);

                setDone();
            break;
        }

        lastState = state;

        pros::delay(20); 
    }
}