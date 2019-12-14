#include "main.h"

#include "lib7842/api.hpp"
#include "okapi/api.hpp"

#include <memory>
#include <map>
#include <functional>

using namespace lib7842;
using namespace okapi;

std::shared_ptr<ChassisController> chassis;
std::shared_ptr<ChassisModel> model;
std::shared_ptr<CustomOdometry> odom;
std::shared_ptr<MotorGroup> intake;
std::shared_ptr<MotorGroup> tray;

std::shared_ptr<GUI::Screen> screen;

std::map<std::string, std::function<void()>&> routines;

std::string routine;

void initialize() {

	pros::delay(500);//wait for ADIEncoders to catch up

	chassis = ChassisControllerBuilder()
		.withMotors({2,3},{-9,-10})
		.withSensors( ADIEncoder(1,2), ADIEncoder(7,8) )
		.withDimensions( AbstractMotor::GearsetRatioPair{AbstractMotor::gearset::green}, {10.2101761242_in, 3_in} )
		.build();

	model = chassis->getModel();

	odom = std::make_shared<CustomOdometry>(model, chassis->getChassisScales());

	intake = std::make_shared<MotorGroup>(MotorGroup({5,-6}));
	tray = std::make_shared<MotorGroup>(MotorGroup({1}));

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_GREEN );
	screen->startTask("screenTask");

/*
	routines["simpleForward"] = [&](){
		;
	};
	routines["simpleBackward"] = [&](){
		;
	};
	routines["redSmall"] = [&](){
		;
	};
	routines["redBig"] = [&](){
		;
	};
	routines["blueSmall"] = [&](){
		;
	};
	routines["blueBig"] = [&](){
		;
	};

	screen->makePage<GUI::Selector>("Selector")
		.button("simpleForward", [&](){;})
		.button("simpleBackward", [&](){;})
		.newRow()
		.button("redSmall", [&](){;})
		.button("redBig", [&](){;})
		.newRow()
		.button("blueSmall", [&](){;})
		.button("blueBlue", [&](){;});

	screen->makePage<GUI::Selector>("Runner")
		.button("simpleForward", [&](){;})
		.button("simpleBackward", [&](){;})
		.newRow()
		.button("redSmall", [&](){;})
		.button("redBig", [&](){;})
		.newRow()
		.button("blueSmall", [&](){;})
		.button("blueBlue", [&](){;});

	//*/
	//*

	pros::delay(10);
	screen->makePage<GUI::Odom>("Odom")
		.attachOdom(odom)
		.attachResetter([&](){
			model->resetSensors();
		});
	//*/
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	bool intakeToggle = false;

	while (true) {
		Controller master(ControllerId::master);

		double left;
		double right;
		if( std::abs(master.getAnalog(ControllerAnalog::rightY)) <= .1){
			left = master.getAnalog(ControllerAnalog::leftX);
			right = -master.getAnalog(ControllerAnalog::leftX);			
		}else{
			left = master.getAnalog(ControllerAnalog::rightY) + (.75 * master.getAnalog(ControllerAnalog::leftX));
			right = master.getAnalog(ControllerAnalog::rightY) + (-.75 * master.getAnalog(ControllerAnalog::leftX));
		}
		if(true){
			model->tank(left, right, .1);
		}

		if( master.getDigital(ControllerDigital::Y) ){
			while(master.getDigital(ControllerDigital::Y)){
				pros::delay(20);
			}
			intakeToggle=!intakeToggle;
		}

		if(!intakeToggle){
			if(master.getDigital(ControllerDigital::R1)){
				intake->moveVelocity(100);
			}else if(master.getDigital(ControllerDigital::R2)){
				intake->moveVelocity(-100);
			}else{
				intake->moveVelocity(0);
			}
		}else if(intakeToggle){
			intake->moveVelocity(100);
		}

		if(true ){
			if(master.getDigital(ControllerDigital::L1)){
				tray->moveVelocity(100);
			}else if(master.getDigital(ControllerDigital::L2)){
				tray->moveVelocity(-100);
			}else{
				tray->moveVelocity(0);
			}
		}else{
			
		}

		pros::delay(20);
	}
}
