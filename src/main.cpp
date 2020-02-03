#include "muphry/robot.hpp"
#include "muphry/autons.hpp"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;


//*
//chassis
std::shared_ptr<ChassisController> chassis;
std::shared_ptr<ThreeEncoderXDriveModel> model;
std::shared_ptr<CustomOdometry> odom;

//controllers
std::shared_ptr<OdomXController> controller;
std::shared_ptr<PathFollower> follower;

//encoders
std::shared_ptr<ADIEncoder> left;
std::shared_ptr<ADIEncoder> right;
std::shared_ptr<ADIEncoder> middle;

//paths
std::map<std::string,PursuitPath> paths;

//intake
std::shared_ptr<MotorGroup> intake;

//tray
std::shared_ptr<MotorGroup> tray;
std::shared_ptr<Potentiometer> trayPotent;
std::shared_ptr<AsyncPosPIDController> trayController;
std::shared_ptr<AsyncPosPIDController> viciousTrayController;

//tray vals
double trayUp = 2400;
double trayMiddleUp = 1200;
double trayMiddleDown = 1400;
double trayDown = 5;

//lift
std::shared_ptr<Motor> lift;
std::shared_ptr<AsyncPosPIDController> liftController;

double liftUp = 1500;
double liftMiddle = 1000;
double liftDown = -10;

//controller
std::shared_ptr<Controller> master;

//screen
std::shared_ptr<GUI::Screen> screen;
GUI::Selector* selector;
//*/

void initialize() {
	pros::delay(500);//wait for ADIEncoders to catch up
	printf("init\n");

	master = std::make_shared<Controller>();
	master->setText(0,0,"initialize");

	chassis = ChassisControllerBuilder()
		.withMotors(1,2,-9,-20)
		.withDimensions( AbstractMotor::gearset::green, ChassisScales({4_in, 9_in}, imev5GreenTPR) )
		.build();

	std::shared_ptr<Motor> topLeft = std::make_shared<Motor>(-20);
	topLeft->setBrakeMode(AbstractMotor::brakeMode::coast);
	topLeft->setGearing(AbstractMotor::gearset::green);
	std::shared_ptr<Motor> topRight = std::make_shared<Motor>(1);
	topRight->setBrakeMode(AbstractMotor::brakeMode::coast);
	topRight->setGearing(AbstractMotor::gearset::green);
	std::shared_ptr<Motor> bottomRight = std::make_shared<Motor>(2);
	bottomRight->setBrakeMode(AbstractMotor::brakeMode::coast);
	bottomRight->setGearing(AbstractMotor::gearset::green);
	std::shared_ptr<Motor> bottomLeft = std::make_shared<Motor>(-9);
	bottomLeft->setBrakeMode(AbstractMotor::brakeMode::coast);
	bottomLeft->setGearing(AbstractMotor::gearset::green);

	left = std::make_shared<ADIEncoder>(1,2,false);
	right = std::make_shared<ADIEncoder>(7,8,true);
	middle = std::make_shared<ADIEncoder>(3,4,false);

	model = std::make_shared<ThreeEncoderXDriveModel>(
		topLeft,
		topRight,
		bottomRight,
		bottomLeft,
		left,
		right,
		middle,
		200,
		12000
	);

	odom = std::make_shared<CustomOdometry>(
		model,
		ChassisScales({2.8114_in,9.883_in,.01_in,2.8114_in},360),
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)
	);
	odom->startTask();

	//*
	controller = std::make_shared<OdomXController>(
		model,
		odom,
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.0023,.0000,.00003,.00},
			TimeUtilFactory::withSettledUtilParams(15, 5, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.00,.0000,.00015,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.00,.0000,.00003,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)
	);//*/

	follower = std::make_shared<PathFollower>(
		model,
		odom,
		ChassisScales({4_in,9_in},imev5GreenTPR),
		6_in,
		TimeUtilFactory::withSettledUtilParams(50,5,250_ms)
	);

	PursuitLimits limits(
		.25_mps,
		.1_mps2,
		.5_mps,
		.1_mps2,
		.25_mps,
		1_mps/*?*/
	);

	paths.insert(
		{
			"test",
			PathGenerator::generate(
				SimplePath({
					{0_ft,0_ft},
					{0_ft,2_ft},
					{2_ft,2_ft},
					{2_ft, 4_ft}})
				.generate(1_cm)
				.smoothen(.001, 1e-10 * meter),
				limits
			)
		}
	);

	intake = std::make_shared<MotorGroup>(MotorGroup({11,-18}));
	intake->setGearing(AbstractMotor::gearset::green);

	tray = std::make_shared<MotorGroup>(MotorGroup({-19}));
	tray->setGearing(AbstractMotor::gearset::green);

	trayPotent = std::make_shared<Potentiometer>(6);

	trayController = std::make_shared<AsyncPosPIDController>(
		tray->getEncoder(),
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.0007,
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

	lift = std::make_shared<Motor>(15);
	lift->setGearing(AbstractMotor::gearset::red);
	liftController = std::make_shared<AsyncPosPIDController>(
		lift->getEncoder(),
		lift,
		TimeUtilFactory::withSettledUtilParams(),
		.0007,
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
				follower->followPath( paths.at(std::string("test")) );
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

	pros::delay(10);
	screen->makePage<GUI::Odom>("Odom")
		.attachOdom(odom)
		.attachResetter([&](){
			model->resetSensors();
		});	

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
	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	selector->run();

	viciousTrayController->flipDisable(true);
	trayController->flipDisable(false);
}

void opcontrol() {
	master->setText(0,1,"opcontrol");	

	trayController->flipDisable(false);

	bool intakeToggle = false;
	bool liftToggle = false;

	while (true) {

		//cheesy x arcade
		double forward = 0;
		double right = 0;
		double yaw = 0;

		forward = master->getAnalog(ControllerAnalog::rightY);
		right = master->getAnalog(ControllerAnalog::rightX);
		yaw = master->getAnalog(ControllerAnalog::leftX);

		model->xArcade(-right, forward, -yaw, 0.1);

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
			intake->moveVoltage(12000);
			intakeToggle = false;
		}else if(master->getDigital(ControllerDigital::R2)){
			intake->moveVoltage(-12000);
			intakeToggle = false;
		}else if(intakeToggle){
			intake->moveVoltage(8000);
		}else{
			intake->moveVelocity(0);
		}

		//TRAY
		if(master->getDigital(ControllerDigital::L1)){
			//trayController up
			trayController->setTarget(trayUp);
		}else if(master->getDigital(ControllerDigital::L2)){
			//go back down
			trayController->flipDisable(true);
			while(master->getDigital(ControllerDigital::L2)){
				tray->moveVoltage(-12000);
				pros::delay(20);
			}
			tray->moveVelocity(0);
			trayController->flipDisable(false);
			trayController->setTarget(trayDown);
			liftController->setTarget(liftDown);
		}

		//LIFT
		if(master->getDigital(ControllerDigital::right)&&!liftToggle){
			//liftController up to middle
			liftController->setTarget(liftMiddle);
			trayController->setTarget(trayMiddleDown);
			liftToggle = true;
			while(master->getDigital(ControllerDigital::right)){
				pros::delay(20);
			}
		}else if(master->getDigital(ControllerDigital::right)&&liftToggle){
			//liftController up to top
			liftController->setTarget(liftUp);
			trayController->setTarget(trayMiddleUp);
			liftToggle = false;
			while(master->getDigital(ControllerDigital::right)){
				pros::delay(20);
			}
		}else if(master->getDigital(ControllerDigital::down)){
			//liftController down
			liftController->setTarget(liftDown);
			trayController->setTarget(trayDown);
			liftToggle = false;
			while(master->getDigital(ControllerDigital::down)){
				pros::delay(20);
			}
		}
		
		pros::delay(20);
	}
}
