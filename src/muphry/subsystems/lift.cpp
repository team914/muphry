#include "muphry/subsystems/lift.hpp"

Lift::Lift(){

    //declare lift motors
    liftMotor = std::make_shared<Motor>(liftPort);
    liftMotor->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    liftMotor->setVoltageLimit(12000);

    //declare lift controller
    liftController = std::make_shared<AsyncPosPIDController>(
        liftMotor->getEncoder(),
        liftMotor,
        TimeUtilFactory().create(),
        liftkP,
        liftkI,
        liftkD
    );
    liftController->startThread();    

}

void Lift::initialize(){}

void Lift::loop(){
    while(true){

        auto time = TimeUtilFactory().create();

        switch(state){
            case LiftState::midTower:
                liftController->flipDisable(false);

                liftController->setTarget(midTower);

                if( !liftController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }
            break;
            case LiftState::lowTower:
                liftController->flipDisable(false);

                liftController->setTarget(lowTower);
                
                if( !liftController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }
            break;
            case LiftState::a2CubeStack:
                liftController->flipDisable(false);

                liftController->setTarget(a2CubeStack);
                
                if( !liftController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }
            break;
            case LiftState::a3CubeStack:
                liftController->flipDisable(false);

                liftController->setTarget(a3CubeStack);

                if( !liftController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }

            break;
            case LiftState::a4CubeStack:
                liftController->flipDisable(false);

                liftController->setTarget(a4CubeStack);

                if( !liftController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }          
            break;
            case LiftState::hold:
                liftController->flipDisable(false);

                liftController->setTarget( liftMotor->getEncoder()->get() );

                if( !liftController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }
            break;
            case LiftState::down:
                liftController->flipDisable(false);

                liftController->setTarget(down);

                if( !liftController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }

            break;
            case LiftState::off:
                liftMotor->tarePosition();
                
                liftController->reset();
                liftController->flipDisable(true);

                setDone();
            break;
        }

        pros::delay(20); 
    }
}

Lift* Lift::lift = nullptr;

Lift* Lift::getLift(){
    if(!lift){
        lift = new Lift();
    }
    return lift;
}
