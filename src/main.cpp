#include "main.h"

#include "lib7842/api.hpp"
#include "okapi/api.hpp"

#include <memory>
#include <map>
#include <functional>

using namespace lib7842;
using namespace okapi;

std::shared_ptr<ChassisController> chassis;
std::shared_ptr<XDriveModel> model;
std::shared_ptr<TwoEncoderOdometry> odom;
std::shared_ptr<MotorGroup> intake;
std::shared_ptr<MotorGroup> lift;
std::shared_ptr<Controller> master;

std::shared_ptr<GUI::Screen> screen;

GUI::Selector* selector;

void initialize() {
	pros::delay(500);//wait for ADIEncoders to catch up
	printf("init\n");

	master = std::make_shared<Controller>();
	master->setText(0,0,"initialize");

	chassis = ChassisControllerBuilder()
		.withMotors(2,-7,-4,3)
		.withSensors( ADIEncoder(1,2), ADIEncoder(7,8, true), ADIEncoder(4,5) )
		.withGains(
			IterativePosPIDController::Gains{.0030,.0000,.0000,.00},
			IterativePosPIDController::Gains{.0035,.0000,.00015,.00},
			IterativePosPIDController::Gains{.0030,.0000,.0000,.00}
		)
		.withDimensions( AbstractMotor::gearset::green, ChassisScales({7.919_in, 10.45_in, 2.75_in, .1_in}, imev5GreenTPR) )
		.build();

	model = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());

	odom = std::make_shared<TwoEncoderOdometry>(
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms),
		model,
		chassis->getChassisScales()
	);//*/

	intake = std::make_shared<MotorGroup>(MotorGroup({20}));
	intake->setGearing(AbstractMotor::gearset::red);//*/

	lift = std::make_shared<MotorGroup>(MotorGroup({6}));
	lift->setGearing(AbstractMotor::gearset::red);//*/

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );
	screen->startTask("screenTask");

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() { 
				printf("default\n");
			})
			.button("Test", [&]() { 
				printf("test\n");
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
		.withSeries("Drive", LV_COLOR_MAKE(255,209,102), []() { return model->getTopLeftMotor()->getTemperature(); pros::delay(100); });


//*/
	master->setText(0,11,"end");
	printf("init end\n");
}

void disabled() {
	printf("disabled\n");

}

void competition_initialize() {
	printf("comp init\n");

}

void autonomous() {
	model->resetSensors();
	selector->run();
}

void taskFnc(void*){
	while(true){
		std::string out;
		//lift Optimal || lift Over Temp || lift Over Crnt || lift WARNING
		//Intk Optimal || Intk Over Temp || Intk Over Crnt || Intk WARNING
		//Drve Optimal || Drve Over Temp || Drve Over Crnt || Drve WARNING	

		const double maxTemp = 60;
		//LIFT
		//*
		if(lift->getTemperature()<maxTemp && !lift->isOverCurrent()){
			master->clearLine(0);
			out = "liftTmp" + std::to_string(lift->getTemperature());
			master->setText(0,0,out.c_str());
		}else if(lift->getTemperature()>=maxTemp && !lift->isOverCurrent()){
			master->clearLine(0);
			master->setText(0,0,"lift Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(!lift->getTemperature()>=maxTemp && lift->isOverTemp()){
			master->clearLine(0);
			master->setText(0,0,"lift Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}//*/
		//INTAKE
		//*
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
		}//*/
		//DRIVE
		if(model->getTopLeftMotor()->getTemperature()<maxTemp && !model->getTopLeftMotor()->isOverCurrent()){
			master->clearLine(1);
			out = "DrveTmp" + std::to_string(model->getTopLeftMotor()->getTemperature());
			master->setText(1,0,out.c_str());
		}else if(model->getTopLeftMotor()->getTemperature()>=maxTemp && !model->getTopLeftMotor()->isOverCurrent()){
			master->clearLine(1);
			master->setText(1,0,"Drve Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(model->getTopLeftMotor()->getTemperature()<maxTemp && !model->getTopLeftMotor()->isOverTemp()){
			master->clearLine(1);
			master->setText(1,0,"Drve Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}
		pros::delay(20);
	}
}

void opcontrol() {
	printf("opcontrol\n");
	pros::Task task( taskFnc, NULL, "taskFnc" );
	bool intakeToggle = false;

	while (true) {

		model->xArcade(master->getAnalog(ControllerAnalog::rightX), master->getAnalog(ControllerAnalog::rightY), master->getAnalog(ControllerAnalog::leftX), .1);

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

		pros::delay(20);
	}
}
