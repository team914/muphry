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
std::shared_ptr<CustomOdometry> odom;
std::shared_ptr<MotorGroup> intake;
std::shared_ptr<MotorGroup> lift;
//std::shared_ptr<Potentiometer> liftPotent;
//std::shared_ptr<AsyncPosPIDController> liftController7;
//std::shared_ptr<AsyncPosPIDController> liftController10;
std::shared_ptr<Controller> master;

std::shared_ptr<GUI::Screen> scr;
GUI::Selector* selector;

std::map<std::string, std::function<void()>&> routines;

std::string routine;

void initialize() {
	pros::delay(500);//wait for ADIEncoders to catch up

	chassis = ChassisControllerBuilder()
		.withMotors(2,3,-9,-10)
		.withSensors( ADIEncoder(7,8), ADIEncoder(1,2,true) )
		.withDimensions( AbstractMotor::gearset::green, ChassisScales({12.5_in, 3.25_in, 0_in, 2.75_in}, imev5GreenTPR) )
		.build();

	model = std::dynamic_pointer_cast<XDriveModel>(chassis->getModel());

	odom = std::make_shared<CustomOdometry>(
		model, 
		chassis->getChassisScales(),
		TimeUtilFactory::withSettledUtilParams()
	);

	intake = std::make_shared<MotorGroup>(MotorGroup({5,-6}));
	intake->setGearing(AbstractMotor::gearset::red);

	lift = std::make_shared<MotorGroup>(MotorGroup({1}));
	lift->setGearing(AbstractMotor::gearset::red);

/*
	liftPotent = std::make_shared<Potentiometer>(3);

	liftController7 = std::make_shared<AsyncPosPIDController>(
		lift->getEncoder(),
		lift,
		TimeUtilFactory::withSettledUtilParams(),
		.0004,
		.0000,
		.00007,
		.0
	);
	liftController7->startThread();
	liftController7->flipDisable(true);

	liftController10 = std::make_shared<AsyncPosPIDController>(
		lift->getEncoder(),
		lift,
		TimeUtilFactory::withSettledUtilParams(),
		.0004,
		.0000,
		.00005,
		.0
	);
	liftController10->startThread();
	liftController10->flipDisable(true);
//*/

	master = std::make_shared<Controller>();

	routines.at("redBig") =[&](){
		printf("redBig");
	};
	routines.at("redSmall") = [&](){
		printf("redSmall");
	};
	routines.at("blueBig") = [&](){
		printf("blueBig");
	};
	routines.at("blueSmall") = [&](){
		printf("blueSmall");
	};

	scr = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_GREEN );
	scr->startTask("screenTask");

	GUI::Selector* iselector = dynamic_cast<GUI::Selector*>(
    	&scr->makePage<GUI::Selector>("Selector")
			.button("Red Big",   [&]() { routines.at("redBig")(); })
			.button("Red Small", [&]() { routines.at("redSmall")(); })
			.newRow()
			.button("Blue Big", [&]()   { routines.at("blueBig")(); })
			.button("Blue Small", [&]() { routines.at("blueSmall")(); })
			.build()
		);
	
	pros::delay(10);
	scr->makePage<GUI::Odom>().attachOdom(odom).attachResetter([&]() { model->resetSensors(); });

	scr->makePage<GUI::Graph>("Temp")
		.withRange(0,100)
		.withGrid(2,4)
		.withResolution(100)
		.withSeries("Intake", LV_COLOR_MAKE(6,214,160), []() { return intake->getTemperature(); })
		.withSeries("lift", LV_COLOR_MAKE(239,71,111), []() { return lift->getTemperature(); })
		.withSeries("Drive", LV_COLOR_MAKE(255,209,102), []() { return model->getTopLeftMotor()->getTemperature(); });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	selector->run();
}

void taskFnc(void*){
	while(true){
		std::string out;
		//lift Optimal || lift Over Temp || lift Over Crnt || lift WARNING
		//Intk Optimal || Intk Over Temp || Intk Over Crnt || Intk WARNING
		//Drve Optimal || Drve Over Temp || Drve Over Crnt || Drve WARNING	

		const double maxTemp = 60;

		//lift
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
	}
}

void opcontrol() {
	pros::Task task( taskFnc, NULL, "taskFnc" );
	bool intakeToggle = false;

	while (true) {
		double forward = 0;
		double right = 0;
		double turn = 0;
		//*
		if( std::abs(master->getAnalog(ControllerAnalog::rightY)) <= .1 && std::abs(master->getAnalog(ControllerAnalog::rightX)) <= .1){
			turn = master->getAnalog(ControllerAnalog::leftX);
		}/*else if(std::abs(master->getAnalog(ControllerAnalog::rightX))<=.1){
			forward = master->getAnalog(ControllerAnalog::rightY);
			turn = master->getAnalog(ControllerAnalog::leftX);
		}*/else{
			forward = master->getAnalog(ControllerAnalog::rightY);
			right = master->getAnalog(ControllerAnalog::rightX);
			turn = master->getAnalog(ControllerAnalog::leftX);
		}
		//*/

		model->xArcade(right, forward, turn, .1);

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

		//lift
		const double liftUp = 2000;
		const double liftDown = -50;
		if(master->getDigital(ControllerDigital::L1)){
			lift->moveVelocity(100);
		}else if(master->getDigital(ControllerDigital::L2)){
			//go back down
			lift->moveVelocity(-100);
		}else if(master->getDigital(ControllerDigital::right)){
			//nothing
		}else if(master->getDigital(ControllerDigital::down)){
			//nothing
		}else if(master->getDigital(ControllerDigital::Y)){
			//nothing
		}

		pros::delay(20);
	}
}
