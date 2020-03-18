#include "muphry/subsystems/tilter.hpp"

Tilter::Tilter(){
    //declare Tilter motors
    tilterMotor = std::make_shared<Motor>(tilterPort);
    tilterMotor->setGearing(AbstractMotor::gearset::red);
    tilterMotor->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    tilterMotor->setVoltageLimit(12000);

    //declare Tilter controller
    tilterController = std::make_shared<AsyncPosPIDController>(
        tilterMotor->getEncoder(),
        tilterMotor,
        TimeUtilFactory().create(),
        tilterkP,
        tilterkI,
        tilterkD
    );
    tilterController->startThread();    

}

void Tilter::initialize(){}

void Tilter::loop(){
    while(true){

        auto time = TimeUtilFactory().create();

        switch(state){
            case TilterState::up:
                tilterController->flipDisable(false);

                tilterController->setTarget(tilterUp);

                if( !tilterController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }
            break;
            case TilterState::liftUp:
                tilterController->flipDisable(false);

                tilterController->setTarget(tilterLiftUp);
                
                if( !tilterController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }
            break;
            case TilterState::down:
                tilterController->flipDisable(false);

                tilterController->setTarget(tilterDown);

                if( !tilterController->isSettled() ){
                    if( time.getTimer()->getDtFromStart().convert(millisecond) > 2000){
                        setDone();
                    }
                }else{
                    setDone();
                }

            break;
            case TilterState::off:
                tilterMotor->tarePosition();
                
                tilterController->reset();
                tilterController->flipDisable(true);

                setDone();
            break;
        }

        pros::delay(20); 
    }
}

Tilter* Tilter::tilter = nullptr;

Tilter* Tilter::getTilter(){
    if(!tilter){
        tilter = new Tilter();
    }
    return tilter;
}
