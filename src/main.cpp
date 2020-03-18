#include "muphry/robot.hpp"
#include "muphry/subsystems/intake.hpp"
#include "muphry/subsystems/lift.hpp"
#include "muphry/subsystems/tilter.hpp"
#include "muphry/autons.hpp"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;


void initialize() {
	printf("init\n");

	Intake::getIntake()->startTask();
	Lift::getLift()->startTask();
	Tilter::getTilter()->startTask();

	master = std::make_shared<Controller>();

	intakeUp = std::make_shared<ControllerButton>(ControllerDigital::R1);
	intakeDown = std::make_shared<ControllerButton>(ControllerDigital::R2);
	tilterUpBtn = std::make_shared<ControllerButton>(ControllerDigital::L1);
	tilterDownBtn = std::make_shared<ControllerButton>(ControllerDigital::L1);
	liftUp = std::make_shared<ControllerButton>(ControllerDigital::right);
	liftMid = std::make_shared<ControllerButton>(ControllerDigital::Y);

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );
	screen->startTask("screenTask");

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
			.button("LiftUp", [&](){
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

	screen->makePage<GUI::Graph>("Temp")
		.withRange(0,100)
		.withGrid(2,4)
		;

	printf("init end\n");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	auto time = pros::millis();

	selector->run();
	
	master->setText(0,0,std::to_string(pros::millis()-time));
}

void opcontrol() {

	printf("opcontrol\n");

	bool intakeToggle = false;

	std::string out("line");
	master->setText(0,0,"opcontrol");
	master->setText(1,1,out);
	master->setText(2,2,"hi");

	Intake::getIntake()->setNewState(IntakeState::off);

	while (true) {
		//cheesy x arcade
		double forward = master->getAnalog(ControllerAnalog::rightY);
		double right = master->getAnalog(ControllerAnalog::rightX);
		double yaw = master->getAnalog(ControllerAnalog::leftX);

		if(intakeUp->changedToPressed() && intakeDown->changedToPressed()){
			Intake::getIntake()->setNewState(IntakeState::hold);
		}else if(intakeUp->changedToPressed()){
			Intake::getIntake()->setNewState(IntakeState::inFull);
		}else if(intakeDown->changedToPressed()){
			Intake::getIntake()->setNewState(IntakeState::outHalf);			
		}else{
			Intake::getIntake()->setNewState(IntakeState::hold);
		}

		pros::delay(20);
	}
}

