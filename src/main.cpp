#include "muphry/robot.hpp"
#include "muphry/autons.hpp"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;

//*
//chassis
std::shared_ptr<ThreeEncoderSkidSteerModel> model;
std::shared_ptr<CustomOdometry> odom;

//controllers
std::shared_ptr<OdomController> controller;
std::shared_ptr<PathFollower> follower;

//sensors
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

bool encoderCatchup = false;

void initialize() {
	printf("init\n");

	Motor topRight   (-9);
	Motor topLeft    (2);
	Motor bottomLeft (20);
	Motor bottomRight(-1);

	MotorGroup leftGroup(
		{
			topLeft,bottomLeft
		}
	);

	MotorGroup rightGroup(
		{
			topRight,bottomRight
		}
	);

	left = std::make_shared<ADIEncoder>(1,2,false);
	right = std::make_shared<ADIEncoder>(7,8,false);
	middle = std::make_shared<ADIEncoder>(3,4,false);

	master = std::make_shared<Controller>();
	std::string out("line");
	master->setText(0,0,"init");
	master->setText(1,1,out);
	master->setText(2,2,"hi");
	
	model = std::make_shared<ThreeEncoderSkidSteerModel>(
		std::make_shared<MotorGroup>(leftGroup),
		std::make_shared<MotorGroup>(rightGroup),
		left,
		right,
		middle,
		200,
		12000
	);

	odom = std::make_shared<CustomOdometry>(
		model,
		ChassisScales({2.8114_in,9.883_in,.01_in,2.8114_in},360),
		TimeUtilFactory().create()
	);
	odom->setState(State(0_in, 0_in, 0_deg));
	odom->startTask();

	controller = std::make_shared<OdomController>(
		model,
		odom,
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.006,.0000,.00015,.00},
			TimeUtilFactory::withSettledUtilParams(35, 10, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.02,.0000,.0002,.00},
			TimeUtilFactory::withSettledUtilParams(5, 5, 100_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.017,.0000,.0000,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		6_in,
		TimeUtilFactory().create()
	);

	follower = std::make_shared<PathFollower>(
		model,
		odom,
		ChassisScales({4_in,9_in},imev5GreenTPR),
		4_in,
		TimeUtilFactory().create()
	);

	PursuitLimits limits(
		0.2_mps,  1.1_mps2, .1_mps,
        0.4_mps2, 0_mps,    40_mps
	);

	PursuitLimits matchLimits(
		0.2_mps,  1.1_mps2, 1_mps,
        0.4_mps2, 0.2_mps,   5_mps
	);
	
	paths.insert(
		{
			"skills9Cube",
			PathGenerator::generate(
				SimplePath({
					//start pos
					{7.0_in,26.9_in},
					//first cube
					{22.95_in,26.97_in},
					//4th cube
					{41.51_in,26.08_in},
					//avoid tower
					{52.25_in,14.84_in},
					//grab cube
					{60.96_in,14.04_in},
					//grab cube
					{66.96_in,14.04_in},
					//grab sixth cube
					{82.47_in,22.49_in},
					//ninth cube
					{103.21_in,22.66_in},
				})
				.generate(.5_cm)
				.smoothen(.001, 1e-10 * meter),
				limits
			)
		}
	);
	paths.insert(
		{
			"redSide",
			PathGenerator::generate(
				SimplePath({

					//start pos
					{7.0_in,26.9_in},
					//first cube
					{22.95_in,26.97_in},
					//4th cube
					{41.51_in,26.08_in},
					//turn 1
					{50.94_in,26.87_in},
					//grab cube
					{55.29_in,39.14_in},
					{51.96_in,44.14_in},
					{46.19_in,47.86_in},
					{31.72_in,48.10_in},
					//{_in,_in},//*/
				})
				.generate(.5_cm)
				.smoothen(.001, 1e-10 * meter),
				matchLimits
			)
		}
	);

	intake = std::make_shared<MotorGroup>(MotorGroup({14,-18}));
	intake->setGearing(AbstractMotor::gearset::green);
 
	tray = std::make_shared<MotorGroup>(MotorGroup({-17}));
	tray->setGearing(AbstractMotor::gearset::red);

	trayPotent = std::make_shared<Potentiometer>(6);

	trayController = std::make_shared<AsyncPosPIDController>(
		tray->getEncoder(),
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.0003,
		.0000,
		.000,
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
				Auton::skills();
//				Auton::test(true);
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
		.withSeries("Drive", LV_COLOR_MAKE(255,209,102), []() { return model->getLeftSideMotor()->getTemperature(); pros::delay(100); })
		.withSeries("Lift", LV_COLOR_MAKE(255,255,255), []() { return lift->getTemperature(); pros::delay(100); });

	printf("init end\n");
}

void disabled() {
	pros::delay(500);
	odom->startTask();
	encoderCatchup = true;
}

void competition_initialize() {}

void autonomous() {
	if(!encoderCatchup){
		pros::delay(500);
		odom->startTask();
		encoderCatchup = true;
	}
	auto time = pros::millis();

	trayController->flipDisable(true);

	selector->run();

	trayController->flipDisable(false);
	
	master->setText(0,0,std::to_string(pros::millis()-time));
}

void opcontrol() {

	printf("opcontrol\n");

	trayController->setTarget(trayDown);
	liftController->setTarget(liftDown);

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

		if(tray->getEncoder()->get()>2000){
			forward /= 2;
			right /= 2;
			yaw /= 2;
		}

		model->tank(forward + yaw,forward - yaw, 0.1);

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
			if(tray->getEncoder()->get()>1000 || lift->getEncoder()->get()>1000){
				intake->moveVoltage(-6000);
			}else{
				intake->moveVoltage(-12000);
			}
			intakeToggle = false;
		}else if(intakeToggle){
			intake->moveVoltage(12000);
		}else{
			intake->moveVelocity(0);
		}

		//TRAY
		if(master->getDigital(ControllerDigital::L1)){
			//trayController up		
			if(lift->getEncoder()->get() > 1000){
				liftController->setTarget(liftDown);				
			}else{
				trayController->setTarget(4950);
			}
			pros::delay(20);
		}else if(master->getDigital(ControllerDigital::L2)){
			//trayController down
			trayController->setTarget(trayDown);
			liftController->setTarget(liftDown);
			pros::delay(20);
		}

		//LIFT
		if(master->getDigital(ControllerDigital::down)){
			//liftController up
			if(lift->getEncoder()->get() > 1000){
//				liftController->setTarget(liftDown);
//				trayController->setTarget(trayDown);
			}else if(tray->getEncoder()->get() < 3000){
				liftController->setTarget(liftUp);
				trayController->setTarget(trayMiddleUp);
			}
			while(master->getDigital(ControllerDigital::down)){
				pros::delay(20);
			}
		}else if(master->getDigital(ControllerDigital::right)){
			//liftController middle
			if(lift->getEncoder()->get() > 1000 ){
				liftController->setTarget(liftDown);
				trayController->setTarget(trayDown);
			}else if(tray->getEncoder()->get() < 3000){
				liftController->setTarget(liftMiddle);
				trayController->setTarget(trayMiddleUp);
			}
			while(master->getDigital(ControllerDigital::down)){
				pros::delay(20);
			}			
		}else if(master->getDigital(ControllerDigital::B)){
			//move backward
			controller->moveDistance(-42_in);
		}

		pros::delay(20);
	}
}
