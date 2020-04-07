#include "muphry/robot.hpp"
#include "muphry/subsystems/intake.hpp"
#include "muphry/subsystems/lift.hpp"
#include "muphry/subsystems/tilter.hpp"
#include "muphry/subsystems/chassis.hpp"
#include "muphry/autons.hpp"

#include "api.h"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;

//this is going to make it so that I can stop logging the kinematics
bool logKinematics = false;
bool resetKinematics = false;

//this will get run as a task seperate from the main task
void task_fnc(void* param){
	pros::delay(100);

    //print helpful things
    printf("Angular Kinematics of Left Motor with time difference of 10ms\n");

	std::cout << "Iteration,\tPos,\tSpeed,\tAccel,\tJerk,\tSnap,\tdT\n";

    //all of these variables are need to calculate jerk and snap
	double pos = 0;
	double lastPos = 0;
	double speed = 0;
	double lastSpeed = 0;
	double accel = 0;
    double lastAccel = 0;
    double jerk = 0;
    double lastJerk = 0;
    double snap = 0;
	double lastSnap = 0;

	std::shared_ptr<Timer> time = std::make_shared<Timer>();
	time->placeMark();
	double dt = 0;

	int loop = 1;

	const double loopDt = 5;

	ADIEncoder encoder(1,2);
	AverageFilter<30> posFilter;
	AverageFilter<30> speedFilter;
	AverageFilter<30> accelFilter;
	AverageFilter<30> jerkFilter;
	AverageFilter<30> snapFilter;

    //this will keep the task running
    while(true){
		dt = time->getDtFromMark().convert(second);

		pos = posFilter.filter(encoder.get()) / 360 * 0.07140956 * PI;

		if(lastPos != 0)
			speed = speedFilter.filter((pos - lastPos) / dt);

		if(lastSpeed != 0)
			accel = accelFilter.filter((speed - lastSpeed) / dt);

		if(lastAccel != 0)
	        jerk = jerkFilter.filter((accel - lastAccel) / dt);

		if(lastJerk != 0)
			snap = snapFilter.filter((jerk - lastJerk) / dt);

		if(logKinematics){
			if(resetKinematics){
				std::cout << "INFO: Kinematics: Cannot reset Kinematics while logKinematics is enabled\n";
				resetKinematics = false;
			}

			//this will print all the data to the terminal
			std::string msg = std::to_string(loop) + ", " + 
				std::to_string(pos) + "\t,\t" + 
				std::to_string(speed) + "\t,\t" + std::to_string(accel) +
				"\t,\t" + std::to_string(jerk) + "\t,\t" + 
				std::to_string(snap) + "\t,\t" + std::to_string(time->getDtFromStart().convert(second)) +"\n";
			if(pos!=0){
				loop++;

				std::cout << msg;
			}
		}else if(resetKinematics){
			std::cout << "INFO: Kinematics: Resetting Kinematics\n";

			//reset all the kinematis variables
			pos = 0;
			lastPos = 0;
			speed = 0;
			lastSpeed = 0;
			accel = 0;
			lastAccel = 0;
			jerk = 0;
			lastJerk = 0;
			snap = 0;
			lastSnap = 0;

			resetKinematics = false;
		}

        //update last each loop
		lastPos = pos;
		lastSpeed = speed;
		lastAccel = accel;
        lastJerk = jerk;
		lastSnap = snap;

		time->placeMark();

        //delay (same value as change in time)
        pros::delay(loopDt);
    }
}
//this will initalize the task called "Kinematics Log" and it will run task_fnc
pros::task_t task = pros::c::task_create( task_fnc, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Kinematics Log");

void initialize() {

	pros::delay(100);

	Logger::setDefaultLogger(
	    std::make_shared<Logger>(
	        TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
	        "/ser/sout", // Output to the PROS terminal
	        Logger::LogLevel::warn // Show errors and warnings
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

				chassis->leftProfileController->generatePath ( {0_in,48_in}, "straight" );
				chassis->rightProfileController->generatePath( {0_in,48_in}, "straight" );

				chassis->leftProfileController->setTarget("straight");
				chassis->rightProfileController->setTarget("straight");

				chassis->leftProfileController->waitUntilSettled();
				chassis->rightProfileController->waitUntilSettled();

				chassis->leftProfileController->setTarget("straight", true);
				chassis->rightProfileController->setTarget("straight", true);

				chassis->leftProfileController->waitUntilSettled();
				chassis->rightProfileController->waitUntilSettled();

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

	resetKinematics = true;
	while(resetKinematics){
		pros::delay(10);
	}
	logKinematics = true;

	if(pros::battery::get_voltage() >= 12000){
		selector->run();
	}

	logKinematics = false;
	
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

	Timer timer;
	logKinematics = true;

	while (true) {

		if( 7 > timer.getDtFromStart().convert(second) > 6){
			logKinematics = false;
		}

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

