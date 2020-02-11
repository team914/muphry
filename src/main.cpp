#include "muphry/robot.hpp"
#include "muphry/autons.hpp"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;

//*
//chassis
std::shared_ptr<XDriveModel> model;
std::shared_ptr<CustomOdometry> odom;

//controllers
std::shared_ptr<AsyncMotionProfileController> controller;

//sensors
std::shared_ptr<ADIEncoder> left;
std::shared_ptr<ADIEncoder> right;
std::shared_ptr<ADIEncoder> middle;

//intake
std::shared_ptr<MotorGroup> intake;

//tray
std::shared_ptr<MotorGroup> tray;
std::shared_ptr<AsyncPosPIDController> trayController;

//tray vals
double trayUp = 3000;
double trayMiddleUp = 1000;
double trayMiddleDown = 1000;
double trayDown = 10;
bool trayMiddleUpToggle = false;
bool trayMiddleDownToggle = false;

//lift
std::shared_ptr<Motor> lift;
std::shared_ptr<AsyncPosPIDController> liftController;

double liftUp = 2300;
double liftMiddle = 2000;
double liftDown = -50;

//controller
std::shared_ptr<Controller> master;

//screen
std::shared_ptr<GUI::Screen> screen;
GUI::Selector* selector;
//*/

void initialize() {
	printf("init\n");

	std::shared_ptr<Motor> topRight = std::make_shared<Motor>(-9);
	std::shared_ptr<Motor> topLeft = std::make_shared<Motor>(2);
	std::shared_ptr<Motor> bottomLeft = std::make_shared<Motor>(20);
	std::shared_ptr<Motor> bottomRight = std::make_shared<Motor>(-1);

	bottomLeft->setBrakeMode(AbstractMotor::brakeMode::coast);
	bottomLeft->setGearing(AbstractMotor::gearset::green);
	bottomRight->setBrakeMode(AbstractMotor::brakeMode::coast);
	bottomRight->setGearing(AbstractMotor::gearset::green);
	topLeft->setBrakeMode(AbstractMotor::brakeMode::coast);
	topLeft->setGearing(AbstractMotor::gearset::green);
	topRight->setBrakeMode(AbstractMotor::brakeMode::coast);
	topRight->setGearing(AbstractMotor::gearset::green);

	left = std::make_shared<ADIEncoder>(1,2,false);
	right = std::make_shared<ADIEncoder>(7,8,false);
	middle = std::make_shared<ADIEncoder>(3,4,false);

	pros::delay(500);//wait for ADIEncoders to catch up

	master = std::make_shared<Controller>();
	std::string out("line");
	master->setText(0,0,"init");
	master->setText(1,1,out);
	master->setText(2,2,"hi");
	
	model = std::make_shared<XDriveModel>(
		topLeft,
		topRight,
		bottomRight,
		bottomLeft,
		bottomLeft->getEncoder(),
		bottomRight->getEncoder(),
		200,
		12000
	);

	PathfinderLimits limits{1,1,1};

	//*
	controller = std::make_shared<AsyncMotionProfileController>(
		TimeUtilFactory().create(),
		limits,
		model,
		ChassisScales({4_in,10_in},imev5GreenTPR),
		AbstractMotor::GearsetRatioPair(AbstractMotor::gearset::green,1)
	);//*/
	controller->startThread();
	controller->generatePath(
		{
			PathfinderPoint{0_ft,0_in,0_deg},
			PathfinderPoint{0_ft,24_in,0_deg}
		},
		std::string("test")
	);

	intake = std::make_shared<MotorGroup>(MotorGroup({11,-18}));
	intake->setGearing(AbstractMotor::gearset::green);

	tray = std::make_shared<MotorGroup>(MotorGroup({17}));
	tray->setGearing(AbstractMotor::gearset::red);

	trayController = std::make_shared<AsyncPosPIDController>(
		tray->getEncoder(),
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.0002,
		.0000,
		.0000,
		.0
	);

	trayController->startThread();
	trayController->flipDisable(false);

	lift = std::make_shared<Motor>(15);
	lift->setGearing(AbstractMotor::gearset::red);
	liftController = std::make_shared<AsyncPosPIDController>(
		lift->getEncoder(),
		lift,
		TimeUtilFactory::withSettledUtilParams(),
		.0009,
		.0000,
		.000,
		.0
	);
	liftController->startThread();
	liftController->flipDisable(false);

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );
	screen->startTask("screenTask");

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() {
				auto time = pros::millis();
			})
			.button("Test", [&]() { 
				printf("test\n");
				Auton::test(true);
			 })
			.newRow()
			.button("Red Big", [&]() { 
				;
			 })
			.button("Red Small", [&]() { 
				printf("redSmall");
				Auton::small();
			 })
			.newRow()
			.button("Blue Big", [&](){
				;
			 })
			.button("Blue Small", [&]() { 
				printf("blueSmall");
				Auton::small(false);
			})
			.build()
		);



	screen->makePage<GUI::Graph>("Temp")
		.withRange(0,100)
		.withGrid(2,4)
		.withSeries("Intake", LV_COLOR_MAKE(6,214,160), []() { return intake->getTemperature(); pros::delay(100); })
		.withSeries("Tray", LV_COLOR_MAKE(239,71,111), []() { return tray->getTemperature(); pros::delay(100); })
		.withSeries("Drive", LV_COLOR_MAKE(255,209,102), []() { return model->getBottomLeftMotor()->getTemperature(); pros::delay(100); })
		.withSeries("Lift", LV_COLOR_MAKE(255,255,255), []() { return lift->getTemperature(); pros::delay(100); });

	printf("init end\n");
}

void disabled() {

}

void competition_initialize() {}

void autonomous() {
	trayController->flipDisable(true);

	selector->run();

	trayController->flipDisable(false);
}

void opcontrol() {

	printf("opcontrol\n");
	trayController->flipDisable(false);

	bool intakeToggle = false;

	std::string out("line");
	master->setText(0,0,"opcontrol");
	master->setText(1,1,out);
	master->setText(2,2,"hi");

	while (true) {
		//cheesy x arcade
		double forward = 0;
		double right = 0;
		double yaw = 0;

		forward = master->getAnalog(ControllerAnalog::rightY);
		right = master->getAnalog(ControllerAnalog::rightX);
		yaw = master->getAnalog(ControllerAnalog::leftX);

		if(tray->getEncoder()->get()<-2000){
			forward /= 3;
			right /= 3;
			yaw /= 3;
		}

		model->xArcade(right, forward, yaw, 0.1);

		//INTAKE
		if(master->getDigital(ControllerDigital::Y)){
			intakeToggle=!intakeToggle;		
			while(master->getDigital(ControllerDigital::Y)){
				pros::delay(20);
			}
		}else if(master->getDigital(ControllerDigital::R1)&&master->getDigital(ControllerDigital::R2)){
			intakeToggle=!intakeToggle;		
			while(master->getDigital(ControllerDigital::R1)&&master->getDigital(ControllerDigital::R2)){
				pros::delay(20);
			}
		}else if(master->getDigital(ControllerDigital::R1)){
			if(lift->getEncoder()->get()<1000){
				intake->moveVoltage(8000);
			}else{
				intake->moveVoltage(12000);
			}
			intakeToggle = false;
		}else if(master->getDigital(ControllerDigital::R2)){
			if(lift->getEncoder()->get()<1000){
				intake->moveVoltage(-8000);
			}else{
				intake->moveVoltage(-12000);
			}
			intakeToggle = false;
		}else if(intakeToggle){
			{
				intake->moveVoltage(8000);
			}
		}else{
			intake->moveVelocity(0);
		}

		//TRAY
		if(master->getDigital(ControllerDigital::L1)){
			//trayController up
			trayController->flipDisable(false);
			trayController->setTarget(-4950);
		}else if(master->getDigital(ControllerDigital::L2)){
			//go back down
			trayController->setTarget(trayDown);
			liftController->setTarget(liftDown);

			while(master->getDigital(ControllerDigital::L2)){
				pros::delay(20);
			}
		}

		//LIFT
		if(master->getDigital(ControllerDigital::down)){
			//*
			//liftController up to middle
			liftController->setTarget(liftMiddle);
			trayController->setTarget(trayMiddleDown);
			intake->moveVoltage(0);
			if(lift->getEncoder()->get()>1000){
				liftController->setTarget(liftDown);
				trayController->setTarget(trayDown);
			}//*/
			while(master->getDigital(ControllerDigital::down)){
				pros::delay(20);
			}
		}else if(master->getDigital(ControllerDigital::right)){
			//liftController up to top
			liftController->setTarget(liftUp);
			trayController->setTarget(trayMiddleUp);			
			if(lift->getEncoder()->get()>1000){
				liftController->setTarget(liftDown);
				trayController->setTarget(trayDown);
			}
			while(master->getDigital(ControllerDigital::right)){
				pros::delay(20);
			}
		}
		
		pros::delay(20);
	}
}
