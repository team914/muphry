#include "muphry/subsystems/intake.hpp"

Intake::Intake(){

    //declare diameter
    circumference = intakeDiameter.convert(meter) * PI;
    
    //declare intake motors
    leftIntake = std::make_shared<Motor>(leftIntakePort);
    leftIntake->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    rightIntake = std::make_shared<Motor>(rightIntakePort);
    rightIntake->setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    //declare intake controllers
    leftController = std::make_shared<AsyncPosPIDController>(
        leftIntake->getEncoder(),
        leftIntake,
        TimeUtilFactory().create(),
        intakekP,
        intakekI,
        intakekD
    );
    leftController->startThread();

    rightController = std::make_shared<AsyncPosPIDController>(
        rightIntake->getEncoder(),
        rightIntake,
        TimeUtilFactory().create(),
        intakekP,
        intakekI,
        intakekD
    );
    rightController->startThread();    

}

void Intake::initialize(){}

void Intake::loop(){
    while(true){

        double target = (leftIntake->getEncoder()->get() + rightIntake->getEncoder()->get()) / 2;

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
            case IntakeState::hold:
                leftIntake->setVoltageLimit(12000);
                rightIntake->setVoltageLimit(12000);

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                leftController->setTarget( leftIntake->getEncoder()->get() );
                rightController->setTarget( rightIntake->getEncoder()->get() );

                setDone();            
            break;
            case IntakeState::off:
                leftIntake->tarePosition();
                rightIntake->tarePosition();

                leftController->flipDisable(true);
                rightController->flipDisable(true);

                setDone();
            break;
            case IntakeState::moveDistance:
                leftIntake->setVoltageLimit(12000);
                rightIntake->setVoltageLimit(12000);

                leftController->setTarget( leftIntake->getEncoder()->get() + distance / circumference / 360 );
                rightController->setTarget(  rightIntake->getEncoder()->get() + distance / circumference / 360 );

                leftController->flipDisable(false);
                rightController->flipDisable(false);

                auto time = TimeUtilFactory().create();

                bool exit = false;
                while( !exit ){
                    if( !leftController->isSettled() || !rightController->isSettled() ){
                        if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000 || Intake::getIntake()->getState() != IntakeState::moveDistance ){
                            exit = true;
                        }
                    }else{
                        exit = true;
                    }
                    pros::delay(20);
                }
            break;            
        }

        pros::delay(20); 
    }
}

Intake* Intake::intake = nullptr;

Intake* Intake::getIntake(){
    if(!intake){
        intake = new Intake();
    }
    return intake;
}

void Intake::setDistance( QLength idistance ){
    distance = idistance.convert(meter);
}