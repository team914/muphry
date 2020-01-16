#include "main.h"

#include "lib7842/api.hpp"
#include "okapi/api.hpp"

#include <memory>
#include <map>
#include <functional>

using namespace lib7842;
using namespace okapi;

std::shared_ptr<ChassisController> chassis;
std::shared_ptr<ChassisController> backwardChassis;

std::shared_ptr<SkidSteerModel> model;
std::shared_ptr<SkidSteerModel> backwardModel;


std::shared_ptr<TwoEncoderOdometry> odom;
std::shared_ptr<TwoEncoderOdometry> backwardOdom;

std::shared_ptr<OdomController> controller;

std::shared_ptr<MotorGroup> intake;

std::shared_ptr<MotorGroup> tray;
std::shared_ptr<Potentiometer> trayPotent;
std::shared_ptr<AsyncPosPIDController> trayController;
std::shared_ptr<AsyncPosPIDController> viciousTrayController;

std::shared_ptr<Controller> master;

std::shared_ptr<GUI::Screen> screen;

GUI::Selector* selector;

void small(bool red = true){
	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	//flipout
	intake->moveVelocity(-100);
	viciousTrayController->setTarget(.41*4095);
	pros::delay(1000);
	backwardChassis->waitUntilSettled();
	intake->moveVelocity(0);
	viciousTrayController->setTarget(.0001*4095);
	pros::delay(500);

	//grab 4 cubes
	intake->moveVelocity(100);
	backwardModel->setMaxVelocity(150);
	backwardChassis->moveDistance(40_in);

	//move back
	backwardChassis->setMaxVelocity(200);
	backwardChassis->moveDistanceAsync(-20_in);
	pros::delay(1000);
	intake->moveVelocity(0);
	backwardChassis->waitUntilSettled();

	//turn to zone
	backwardModel->setMaxVelocity(150);
	if(red){
		backwardChassis->turnAngle(120_deg);
	}else{
		backwardChassis->turnAngle(-120_deg);
	}
	pros::delay(10);

	//move to zone
	backwardChassis->moveDistanceAsync(17_in);
	intake->moveVelocity(-100);
	pros::delay(500);
	intake->moveVelocity(100);
	pros::delay(500);
	intake->moveVelocity(0);
	backwardChassis->waitUntilSettled();

	//stack
	viciousTrayController->setTarget(.41*4095);
	pros::delay(500);
	intake->moveVelocity(100);
	pros::delay(500);
	intake->moveVelocity(0);
	viciousTrayController->waitUntilSettled();

	//move away
	viciousTrayController->setTarget(.0001*4095);
	backwardChassis->moveDistanceAsync(-18_in);
	pros::delay(250);
	intake->moveVelocity(-100);
	backwardChassis->waitUntilSettled();
	intake->moveVelocity(0);
}

void initialize() {
	pros::delay(500);//wait for ADIEncoders to catch up
	printf("init\n");

	master = std::make_shared<Controller>();
	master->setText(0,0,"initialize");
	
	//*
	chassis = ChassisControllerBuilder()
		.withMotors({1,2},{-9,-20})
		.withSensors( ADIEncoder(7,8,true),ADIEncoder(1,2) )
		.withDimensions( AbstractMotor::gearset::green, ChassisScales({7.919_in, 10.45_in, 2.75_in, .0001_in}, imev5GreenTPR) )
		.withGains(
			IterativePosPIDController::Gains{.002,.0015,.00003,.00},
			IterativePosPIDController::Gains{.0017,.0000,.00003,.00},
			IterativePosPIDController::Gains{.0015,.0000,.0000,.00}
		)
		.withClosedLoopControllerTimeUtil(10,5,500_ms)
		.build();

	model = std::dynamic_pointer_cast<SkidSteerModel>(chassis->getModel());

	odom = std::make_shared<TwoEncoderOdometry>(
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms),
		model,
		chassis->getChassisScales()
	);//*/

	backwardChassis = ChassisControllerBuilder()
		.withMotors({1,2},{-9,-20})
//		.withSensors( ADIEncoder(7,8,false),ADIEncoder(1,2,true) )
		.withDimensions( AbstractMotor::gearset::green, ChassisScales({(7.919/pi) * inch, 10.45_in}, imev5GreenTPR) )
		.build();

	backwardModel = std::dynamic_pointer_cast<SkidSteerModel>(backwardChassis->getModel());

	backwardOdom = std::make_shared<TwoEncoderOdometry>(
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms),
		backwardChassis->getModel(),
		backwardChassis->getChassisScales()
	);

	/*
	controller = std::make_shared<OdomController>(
		model,
		odom,
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.002,.0000,.00003,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.0035,.0000,.00015,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.00,.0000,.00003,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		2_in,
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)
	);//*/

	intake = std::make_shared<MotorGroup>(MotorGroup({5,-6}));
	intake->setGearing(AbstractMotor::gearset::red);

	tray = std::make_shared<MotorGroup>(MotorGroup({17}));
	tray->setGearing(AbstractMotor::gearset::red);

	trayPotent = std::make_shared<Potentiometer>(6);

	trayController = std::make_shared<AsyncPosPIDController>(
		trayPotent,
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.00065,
		.0000,
		.000,
		.0
	);
	trayController->startThread();
	trayController->flipDisable(false);

	viciousTrayController = std::make_shared<AsyncPosPIDController>(
		trayPotent,
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.001,
		.0000,
		.000,
		.0
	);
	viciousTrayController->startThread();
	viciousTrayController->flipDisable(true);

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );
	screen->startTask("screenTask");

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() {
				viciousTrayController->flipDisable(false);
				trayController->flipDisable(true);
				pros::delay(20);

				viciousTrayController->controllerSet(.41*4095);
				viciousTrayController->waitUntilSettled();
				pros::delay(20);

				viciousTrayController->controllerSet(.0001*4095);
				viciousTrayController->waitUntilSettled();
			})
			.button("Test", [&]() { 
				#define TURN
				#ifdef TURN
				auto angle = 360_deg;
				chassis->turnAngle(angle);
				pros::delay(500);
				chassis->turnAngle(-angle);
				pros::delay(500);
				#endif
				#ifndef TURN
				auto distance = 24_in;
				backwardChassis->moveDistance(distance);
				pros::delay(500);
				backwardChassis->moveDistance(-distance);
				pros::delay(500);
				#endif
			 })
			.newRow()
			.button("Red Big",   [&]() { 
				printf("redBig");
				backwardChassis->moveDistance(-5_in);
				backwardChassis->moveDistance(10_in);
				backwardChassis->turnAngle(67_deg);
			 })
			.button("Red Small", [&]() { 
				printf("redSmall");
				small();
			 })
			.newRow()
			.button("Blue Big", [&]()   { 
				backwardChassis->moveDistance(12_in);
				backwardChassis->moveDistance(-14_in);

				intake->moveVelocity(-100);
				trayController->setTarget(.41*4095);
				backwardChassis->moveDistanceAsync(3_in);
				trayController->waitUntilSettled();
				backwardChassis->waitUntilSettled();
				intake->moveVelocity(0);
				trayController->setTarget(.0001*4095);
				trayController->waitUntilSettled();
				backwardChassis->moveDistanceAsync(-3.5_in);

				printf("blueBig");
				auto angle = 110_deg;
				auto distance = 32_in;
				backwardChassis->setMaxVelocity(200);

				//*
				backwardChassis->moveDistanceAsync(24_in);
				intake->moveVelocity(100);
				backwardChassis->waitUntilSettled();
				backwardChassis->moveDistance(6_in);

				backwardChassis->setMaxVelocity(100);
				backwardChassis->turnAngle(angle);
				backwardChassis->moveDistance(distance);
				//*/

				intake->moveVelocity(-100);
				pros::delay(1000);

				tray->moveVelocity(100);
				trayController->setTarget(.41 * 4095);
				trayController->waitUntilSettled();
				trayController->setTarget(.0001 * 4095);
				intake->moveVelocity(-100);

				backwardChassis->moveDistance(-distance);
				trayController->waitUntilSettled();
				backwardChassis->turnAngle(-angle);
				intake->moveVelocity(0);				
			 })
			.button("Blue Small", [&]() { 
				printf("blueSmall");
				small(false);
			})
			 .newRow()
			 .button("Forward Stack", [&](){
/*				
				trayController->flipDisable(true);
				tray->moveVelocity(100);
				intake->moveVelocity(-100);
				pros::delay(1000);
				backwardChassis->waitUntilSettled();				
				tray->moveVelocity(0);//*/
				backwardChassis->moveDistanceAsync(36_in);
				intake->moveVelocity(100);
				backwardChassis->waitUntilSettled();
//				backwardChassis->moveDistanceAsync(24_ft);
				intake->moveVelocity(-100);
				pros::delay(500);
				intake->moveVelocity(100);
				pros::delay(500);
				intake->moveVelocity(0);
				trayController->flipDisable(false);
				trayController->setTarget(.41 * 4095);
				trayController->waitUntilSettled();
				trayController->setTarget(.0001 * 4095);
				backwardChassis->moveDistanceAsync(-12_in);
				intake->moveVelocity(-100);
				backwardChassis->waitUntilSettled();
				intake->moveVelocity(0);				 
			 })
			 .button("Grab Stack", [&](){
				intake->moveVelocity(100);
				backwardChassis->moveDistance(4_in);
				backwardChassis->setMaxVelocity(200);
				backwardChassis->moveDistance(12_in);				 
			 })
			.build()
		);

	pros::delay(10);
	screen->makePage<GUI::Odom>("Odom")
		.attachOdom(odom)
		.attachResetter([&](){
			model->resetSensors();
		});

	pros::delay(10);
	screen->makePage<GUI::Odom>("Backward Odom")
		.attachOdom(backwardOdom)
		.attachResetter([&](){
			model->resetSensors();
		});	

//*
	screen->makePage<GUI::Graph>("Temp")
		.withRange(0,100)
		.withGrid(2,4)
		.withSeries("Intake", LV_COLOR_MAKE(6,214,160), []() { return intake->getTemperature(); pros::delay(100); })
		.withSeries("Tray", LV_COLOR_MAKE(239,71,111), []() { return tray->getTemperature(); pros::delay(100); })
		.withSeries("Drive", LV_COLOR_MAKE(255,209,102), []() { return model->getLeftSideMotor()->getTemperature(); pros::delay(100); });

	screen->makePage<GUI::Graph>("Sensors")
		.withResolution(100)
		.withRange(0,4096)
		.withGrid(16,1)
		.withSeries("TrayPotent", LV_COLOR_MAKE(6,214,160), [](){ return trayPotent->controllerGet(); })
		.withSeries("TrayController7", LV_COLOR_MAKE(239,71,111), [](){ return trayPotent->controllerGet(); })
		;

//*/
	master->setText(0,11,"end");
	printf("init end\n");
}

void disabled() {
	chassis->stop();
}

void competition_initialize() {}

void autonomous() {
	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	backwardModel->resetSensors();
	backwardChassis->setMaxVelocity(100);

	backwardChassis->waitUntilSettled();
	//*

	//*/

	selector->run();

	viciousTrayController->flipDisable(true);
	trayController->flipDisable(false);
}

void taskFnc(void*){
	while(true){
		std::string out;
		//Tray Optimal || Tray Over Temp || Tray Over Crnt || Tray WARNING
		//Intk Optimal || Intk Over Temp || Intk Over Crnt || Intk WARNING
		//Drve Optimal || Drve Over Temp || Drve Over Crnt || Drve WARNING	

		const double maxTemp = 60;
		//TRAY
		if(tray->getTemperature()<maxTemp && !tray->isOverCurrent()){
			master->clearLine(0);
			out = "TrayTmp" + std::to_string(tray->getTemperature());
			master->setText(0,0,out.c_str());
		}else if(tray->getTemperature()>=maxTemp && !tray->isOverCurrent()){
			master->clearLine(0);
			master->setText(0,0,"Tray Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(!tray->getTemperature()>=maxTemp && tray->isOverTemp()){
			master->clearLine(0);
			master->setText(0,0,"Tray Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}
		//INTAKE
		if(intake->getTemperature()<maxTemp && !intake->isOverCurrent()){
			master->clearLine(1);
			out = "IntkTmp" + std::to_string(intake->getTemperature());
			master->setText(1,0,out.c_str());
		}else if(intake->getTemperature()>=maxTemp && !intake->isOverCurrent()){
			master->clearLine(1);
			master->setText(1,0,"Intk Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(!intake->getTemperature()>=maxTemp && intake->isOverTemp()){
			master->clearLine(1);
			master->setText(1,0,"Intk Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}
		//DRIVE
		if(model->getLeftSideMotor()->getTemperature()<maxTemp && !model->getLeftSideMotor()->isOverCurrent()){
			master->clearLine(1);
			out = "DrveTmp" + std::to_string(model->getLeftSideMotor()->getTemperature());
			master->setText(1,0,out.c_str());
		}else if(model->getLeftSideMotor()->getTemperature()>=maxTemp && !model->getLeftSideMotor()->isOverCurrent()){
			master->clearLine(1);
			master->setText(1,0,"Drve Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(model->getLeftSideMotor()->getTemperature()<maxTemp && !model->getLeftSideMotor()->isOverTemp()){
			master->clearLine(1);
			master->setText(1,0,"Drve Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}
		pros::delay(20);
	}
}

void opcontrol() {
	backwardChassis->stop();

	master->setText(0,1,"opcontrol");	
	pros::Task task( taskFnc, NULL, "taskFnc" );
	bool intakeToggle = false;

	while (true) {
		double left;
		double right;			
		if( std::abs(master->getAnalog(ControllerAnalog::rightY)) <= .1){
			left = master->getAnalog(ControllerAnalog::leftX);
			right = -master->getAnalog(ControllerAnalog::leftX);
//			model->setMaxVoltage(800);
		}else{
			left = master->getAnalog(ControllerAnalog::rightY) + (.75 * master->getAnalog(ControllerAnalog::leftX));
			right = master->getAnalog(ControllerAnalog::rightY) + (-.75 * master->getAnalog(ControllerAnalog::leftX));
//			model->setMaxVoltage(1400);
		}

		model->tank(left, right, .1);
		
		//INTAKE TOGGLE
		if(master->getDigital(ControllerDigital::Y)){
			intakeToggle=!intakeToggle;		
			while(master->getDigital(ControllerDigital::Y)){
				pros::delay(20);
			}	
		}

		//INTAKE
		if(master->getDigital(ControllerDigital::R1)){
			if(!intakeToggle){
				intake->moveVelocity(100);
			}
		}else if(master->getDigital(ControllerDigital::R2)){
			intake->moveVelocity(-100);
			intakeToggle = false;
		}else if(intakeToggle){
			intake->moveVelocity(100);
		}else{
			intake->moveVelocity(0);
		}

		//TRAY
		const double trayUp = .41 * 4095;   //old potentiometer 2625 //tray internal encoder 2000
		const double trayDown = .0001 * 4095; //old potentiometer 3650 //tray internal encoder -50
		if(master->getDigital(ControllerDigital::L1)){
			//trayController up
			trayController->flipDisable(false);
			trayController->setTarget(trayUp);
		}else if(master->getDigital(ControllerDigital::L2)){
			//go back down
			trayController->flipDisable(true);
			while(master->getDigital(ControllerDigital::L2)){
				tray->moveVelocity(-100);
				pros::delay(20);
			}
			tray->moveVelocity(0);
			trayController->flipDisable(false);
			trayController->setTarget(trayDown);
		}else if(master->getDigital(ControllerDigital::right)){
			//nothing
		}else if(master->getDigital(ControllerDigital::down)){
			//nothing
		}else if(master->getDigital(ControllerDigital::Y)){
			trayController->setTarget(trayDown);
			while(master->getDigital(ControllerDigital::Y)){
				intake->moveVelocity(-100);
			}
			intake->moveVelocity(0);
		}

		pros::delay(20);
	}
}
