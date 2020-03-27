#include "muphry/robot.hpp"
#include "muphry/subsystems/intake.hpp"
#include "muphry/subsystems/lift.hpp"
#include "muphry/subsystems/tilter.hpp"
#include "muphry/subsystems/chassis.hpp"
#include "muphry/autons.hpp"

#include "pros/misc.hpp"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;

void initialize() {

	pros::delay(100);

	Logger::setDefaultLogger(
	    std::make_shared<Logger>(
	        TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
	        "/ser/sout", // Output to the PROS terminal
	        Logger::LogLevel::info // Show errors and warnings
	    )
	);

	printf("init\n");

	Intake::getIntake()->startTask();
	Tilter::getTilter()->startTask();
	Lift::getLift()->startTask();

	Intake::getIntake()->setNewState(IntakeState::hold);
	Tilter::getTilter()->setNewState(TilterState::down);
	Lift::getLift()->setNewState(LiftState::down);

	master = std::make_shared<Controller>();

	intakeUpBtn = std::make_shared<ControllerButton>(ControllerDigital::R1);
	intakeDownBtn = std::make_shared<ControllerButton>(ControllerDigital::R2);
	tilterUpBtn = std::make_shared<ControllerButton>(ControllerDigital::L1);
	tilterDownBtn = std::make_shared<ControllerButton>(ControllerDigital::L2);
	liftUpBtn = std::make_shared<ControllerButton>(ControllerDigital::right);
	liftMidBtn = std::make_shared<ControllerButton>(ControllerDigital::Y);

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Skid Steer Selector")
			.button("Test Profile", [&]() {
				printf("running test profile\n");

				Intake::getIntake()->setNewState(IntakeState::inFull);

				chassis->skidSteerModel->setMaxVoltage(12000);

				chassis->leftProfileController->flipDisable(false);
				chassis->rightProfileController->flipDisable(false);

				chassis->leftProfileController->setTarget( 360 );
				chassis->rightProfileController->setTarget( 360 );

//				while( !chassis->leftProfileController->isSettled() && !chassis->rightProfileController->isSettled() ){
				while(true){
					double left = chassis->leftProfileController->step( chassis->skidSteerModel->getSensorVals()[0] );
					double right = chassis->rightProfileController->step( chassis->skidSteerModel->getSensorVals()[1] );

					printf("left is %d with sensors %d\nright is %d with sensors %d\n", left, chassis->skidSteerModel->getSensorVals()[0], right, chassis->skidSteerModel->getSensorVals()[1]);
					printf("target %d\noutput %d\nprocessValue %d\n", 
						chassis->leftProfileController->getTarget(),
						chassis->leftProfileController->getOutput(),
						chassis->leftProfileController->getProcessValue());
					printf("sampleTime %d\nvel %d\n accel%d\n",
						chassis->leftProfileController->getSampleTime().convert(millisecond),
						chassis->leftProfileController->getVel().convert(rpm),
						chassis->leftProfileController->getAccel().convert(rpm / second));

					chassis->skidSteerModel->left( left );
					chassis->skidSteerModel->right( right );

					pros::delay(5000);
				}
				Intake::getIntake()->setNewState(IntakeState::hold);

			})
			.button("Test PID", [&]() {

				Intake::getIntake()->setNewState(IntakeState::inFull);

				chassis->skidSteerModel->setMaxVoltage(12000);
				chassis->pidController->moveDistance(38_in);

				chassis->skidSteerModel->setMaxVoltage(11000);
				chassis->pidController->moveDistance(-14_in);

				chassis->skidSteerModel->setMaxVoltage(12000);
				chassis->pidController->turnAngle(135_deg);

				Intake::getIntake()->setNewState(IntakeState::hold);
			})
			.button("Test Straight PID", [&]() {

				Intake::getIntake()->setNewState(IntakeState::inFull);

				chassis->skidSteerModel->setMaxVoltage(9000);

				chassis->pidController->moveDistance(24_in);
				chassis->pidController->moveDistance(-24_in);

				Intake::getIntake()->setNewState(IntakeState::hold);

			})			
			.button("Test Turn PID", [&]() {

				Intake::getIntake()->setNewState(IntakeState::inFull);

				chassis->skidSteerModel->setMaxVoltage(10000);

				chassis->pidController->turnAngle(90_deg);
				chassis->pidController->turnAngle(-90_deg);

				Intake::getIntake()->setNewState(IntakeState::hold);

			})			

			.build()
		);

	intakeActions = dynamic_cast<GUI::Actions*>(
    	&screen->makePage<GUI::Actions>("Intake")
			.button("In Full", [&]() {
				Intake::getIntake()->setNewState(IntakeState::inFull);
			})
			.button("Out Full", [&]() { 
				Intake::getIntake()->setNewState(IntakeState::outFull);
			 })
			.button("In Half", [&]() { 
				Intake::getIntake()->setNewState(IntakeState::inHalf);
			 })
			.button("Out Half", [&]() { 
				Intake::getIntake()->setNewState(IntakeState::outHalf);
			 })
			.newRow()
			.button("Move Distance", [&](){
				Intake::getIntake()->setDistance(-5.5_in);
				Intake::getIntake()->setNewState(IntakeState::moveDistance);
			 })
			.button("Hold", [&]() { 
				Intake::getIntake()->setNewState(IntakeState::hold);
			})
			.button("Off", [&]() { 
				Intake::getIntake()->setNewState(IntakeState::off);
			})
			.build()
		);

	liftActions = dynamic_cast<GUI::Actions*>(
    	&screen->makePage<GUI::Actions>("Lift")

			.button("Mid Tower", [&]() {
				Lift::getLift()->setState(LiftState::midTower);
			})
			.button("Low Tower", [&]() { 
				Lift::getLift()->setState(LiftState::lowTower);
			 })
			.button("2 Cube ", [&]() { 
				Lift::getLift()->setState(LiftState::a2CubeStack);
			 })
			.button("3 Cube", [&]() { 
				Lift::getLift()->setState(LiftState::a3CubeStack);
			 })
			.newRow()
			.button("4 Cube", [&](){
				Lift::getLift()->setState(LiftState::a4CubeStack);
			 })
			.button("Down", [&]() { 
				Lift::getLift()->setState(LiftState::down);
			})			
			.button("Off", [&]() { 
				Lift::getLift()->setState(LiftState::off);
			})
			.build()
		);

	tilterActions = dynamic_cast<GUI::Actions*>(
    	&screen->makePage<GUI::Actions>("Tilter")
			.button("Up", [&](){
				Tilter::getTilter()->setState(TilterState::up);
			})
			.button("liftUpBtn", [&](){
				Tilter::getTilter()->setState(TilterState::liftUp);
			})
			.newRow()
			.button("Down", [&](){
				Tilter::getTilter()->setState(TilterState::down);
			})
			.button("Off", [&](){
				Tilter::getTilter()->setState(TilterState::off);
			})
			.build()
		);
//*
	screen->makePage<GUI::Odom>("Odom")
		.attachOdom(chassis->odom)
		.attachResetter([&](){
			chassis->skidSteerModel->resetSensors();
		});//*/

	pros::delay(100);

	printf("init end\n");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	printf("autonomous\n");

	auto time = pros::millis();

	printf("battery voltage = %d\n", pros::battery::get_voltage());

	chassis->skidSteerModel->resetSensors();
	chassis->stopControllers();

	if(pros::battery::get_voltage() >= 12000){
		selector->run();
	}
	
	printf("end autonomous\n");
}

void opcontrol() {

	printf("opcontrol\n");

	std::string out("line");
	master->setText(0,0,"opcontrol");
	master->setText(1,1,out);
	master->setText(2,2,"hi");

	chassis->stopControllers();
	chassis->skidSteerModel->setMaxVoltage(12000);

	while (true) {

		chassis->skidSteerModel->arcade(
			master->getAnalog(ControllerAnalog::rightY), 
			master->getAnalog(ControllerAnalog::leftX));

		if(intakeUpBtn->isPressed() && intakeDownBtn->isPressed()){
			printf("Intake Up and Intake Down Button Pressed\n");
			Intake::getIntake()->setNewState(IntakeState::off);
		}else if(intakeUpBtn->isPressed()){
			printf("Intake Up Button Pressed\n");
			Intake::getIntake()->setNewState(IntakeState::inFull);
		}else if(intakeDownBtn->isPressed()){
			printf("Intake Down Button Pressed\n");
			Intake::getIntake()->setNewState(IntakeState::outHalf);			
		}else{
			Intake::getIntake()->setNewState(IntakeState::hold);
		}

		if(tilterUpBtn->isPressed() && tilterDownBtn->isPressed()){
			printf("Tilter Up and Tilter Down Button Pressed\n");
			Tilter::getTilter()->setNewState(TilterState::off);
		}else if(tilterUpBtn->isPressed()){
			printf("Tilter Up Button Pressed\n");
			if( Lift::getLift()->getState() != LiftState::down ){
				Lift::getLift()->setStateBlocking(LiftState::down);
			}
			Tilter::getTilter()->setNewState(TilterState::up);
		}else if(tilterDownBtn->isPressed()){
			printf("Tilter Down Button Pressed\n");
			Tilter::getTilter()->setNewState(TilterState::down);
		}

		if(liftUpBtn->changedToPressed() || liftMidBtn->changedToPressed()){
			printf("lift Up Button or Lift Mid Button Pressed\n");
			liftToggle = !liftToggle;
		}

		if(liftUpBtn->isPressed() && liftMidBtn->isPressed()){
			printf("Lift Up and Lift Mid Button Pressed\n");
			Lift::getLift()->setNewState(LiftState::off);
		}else if(liftUpBtn->isPressed() && liftToggle ){
			printf("Lift Up Button Pressed and liftToggle\n");
			if( Tilter::getTilter()->getState() == TilterState::up ){
				Tilter::getTilter()->setStateBlocking(TilterState::liftUp);
			}else{
				Tilter::getTilter()->setNewState(TilterState::liftUp);
			}
			Lift::getLift()->setNewState(LiftState::midTower);
		}else if(liftMidBtn->isPressed() && liftToggle ){
			printf("Lift Mid Button Pressed and liftToggle\n");
			if( Tilter::getTilter()->getState() == TilterState::up ){
				Tilter::getTilter()->setStateBlocking(TilterState::liftUp);
			}else{
				Tilter::getTilter()->setNewState(TilterState::liftUp);
			}
			Lift::getLift()->setNewState(LiftState::lowTower);
		}else if( liftUpBtn->isPressed() && !liftToggle ){
			printf("Lift Up Button Pressed and not liftToggle\n");
			Lift::getLift()->setNewState(LiftState::down);
			Tilter::getTilter()->setNewState(TilterState::down);
		}else if( liftMidBtn->isPressed() && !liftToggle ){
			printf("Lift Mid Button Pressed and not liftToggle\n");
			Lift::getLift()->setNewState(LiftState::down);
			Tilter::getTilter()->setNewState(TilterState::down);
		}

		pros::delay(20);
	}
}

