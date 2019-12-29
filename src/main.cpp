#include "main.h"

#include "lib7842/api.hpp"
#include "okapi/api.hpp"

#include <memory>
#include <map>
#include <functional>

using namespace lib7842;
using namespace okapi;

std::shared_ptr<ChassisController> chassis;
std::shared_ptr<SkidSteerModel> model;
std::shared_ptr<TwoEncoderOdometry> odom;
std::shared_ptr<MotorGroup> intake;
std::shared_ptr<MotorGroup> tray;
std::shared_ptr<Potentiometer> trayPotent;
std::shared_ptr<AsyncPosPIDController> trayController7;
std::shared_ptr<AsyncPosPIDController> trayController10;
std::shared_ptr<Controller> master;

std::shared_ptr<GUI::Screen> screen;

GUI::Selector* selector;

void initialize() {
	pros::delay(500);//wait for ADIEncoders to catch up
	printf("init\n");

	master = std::make_shared<Controller>();
	master->setText(0,0,"initialize");

	chassis = ChassisControllerBuilder()
		.withMotors({2,3},{-9,-10})
		.withSensors( ADIEncoder(1,2),ADIEncoder(7,8, true) )
		.withGains(
			IterativePosPIDController::Gains{.0030,.0000,.0000,.00},
			IterativePosPIDController::Gains{.0035,.0000,.00015,.00},
			IterativePosPIDController::Gains{.0030,.0000,.0000,.00}
		)
		.withDimensions( AbstractMotor::gearset::green, ChassisScales({7.919_in, 10.45_in, 2.75_in, .1_in}, imev5GreenTPR) )
		.build();

	model = std::dynamic_pointer_cast<SkidSteerModel>(chassis->getModel());

	/*odom = std::make_shared<CustomOdometry>(
		model,
		chassis->getChassisScales(),
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)
	);//*/

	odom = std::make_shared<TwoEncoderOdometry>(
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms),
		model,
		chassis->getChassisScales()
	);//*/

	intake = std::make_shared<MotorGroup>(MotorGroup({5,-6}));
	intake->setGearing(AbstractMotor::gearset::red);

	tray = std::make_shared<MotorGroup>(MotorGroup({-1}));
	tray->setGearing(AbstractMotor::gearset::red);

	trayPotent = std::make_shared<Potentiometer>(3);

	trayController7 = std::make_shared<AsyncPosPIDController>(
		trayPotent,
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.001,
		.0000,
		.000,
		.0
	);
	trayController7->startThread();
	trayController7->flipDisable(true);

	trayController10 = std::make_shared<AsyncPosPIDController>(
		trayPotent,
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.0004,
		.0000,
		.0005,
		.0
	);
	trayController10->startThread();
	trayController10->flipDisable(true);

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );
	screen->startTask("screenTask");

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() { 
				#define TUR
				odom->setState(OdomState{7_ft,2_ft,90_deg});
				model->setMaxVelocity(200);
				#ifdef TURN
				auto angle = 90_deg;
				chassis->turnAngle(angle);
				pros::delay(500);
				chassis->turnAngle(-angle);
				pros::delay(500);
				#endif
				#ifndef TURN
				auto distance = 24_in;
				chassis->moveDistance(distance);
				pros::delay(500);
				chassis->moveDistance(-distance);
				pros::delay(500);
				#endif
			})
			.button("Test", [&]() { 
				chassis->moveDistance(24_in);
				pros::delay(500);
				chassis->moveDistance(-24_in);
				pros::delay(500);
			 })
			.newRow()
			.button("Red Big",   [&]() { 
				printf("redBig");
			 })
			.button("Red Small", [&]() { 
				printf("redSmall");
			 })
			.newRow()
			.button("Blue Big", [&]()   { 
				printf("blueBig");
			 })
			.button("Blue Small", [&]() { 
				printf("blueSmall");
			 })
			.build()
		);

	pros::delay(10);
	screen->makePage<GUI::Odom>("Odom")
		.attachOdom(odom)
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

void disabled() {}

void competition_initialize() {}

void autonomous() {
	model->resetSensors();
	selector->run();
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
	master->setText(0,1,"opcontrol");	
	pros::Task task( taskFnc, NULL, "taskFnc" );
	bool intakeToggle = false;

	while (true) {
		double left;
		double right;			
		if( std::abs(master->getAnalog(ControllerAnalog::rightY)) <= .1){
			left = master->getAnalog(ControllerAnalog::leftX);
			right = -master->getAnalog(ControllerAnalog::leftX);			
		}else{
			left = master->getAnalog(ControllerAnalog::rightY) + (.75 * master->getAnalog(ControllerAnalog::leftX));
			right = master->getAnalog(ControllerAnalog::rightY) + (-.75 * master->getAnalog(ControllerAnalog::leftX));
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
		const double trayUp = 2625;  //tray internal encoder 2000
		const double trayDown = 3650; //tray internal encoder -50
		if(master->getDigital(ControllerDigital::L1)){
			//trayController7
			trayController10->flipDisable(true);
			trayController7->flipDisable(false);
			trayController7->setTarget(trayUp);
		}else if(master->getDigital(ControllerDigital::L2)){
			//go back down
			trayController10->flipDisable(true);
			trayController7->flipDisable(true);
			tray->moveVelocity(100);
		}else if(master->getDigital(ControllerDigital::right)){
			//trayController10
			trayController7->flipDisable(true);
			trayController10->flipDisable(false);
			trayController10->setTarget(trayUp);
		}else if(master->getDigital(ControllerDigital::down)){
			//nothing
		}else if(master->getDigital(ControllerDigital::Y)){
			trayController7->setTarget(trayDown);
			trayController10->setTarget(trayDown);
			while(master->getDigital(ControllerDigital::Y)){
				intake->moveVelocity(-100);
			}
			intake->moveVelocity(0);
		}

		pros::delay(20);
	}
}
