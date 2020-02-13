#include "muphry/robot.hpp"
#include "muphry/autons.hpp"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;

//*
//chassis
std::shared_ptr<ThreeEncoderXDriveModel> model;
std::shared_ptr<CustomOdometry> odom;

//controllers
std::shared_ptr<OdomXController> controller;
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

void initialize() {
	printf("init\n");

	std::shared_ptr<Motor> topRight = std::make_shared<Motor>(-9);
	std::shared_ptr<Motor> topLeft = std::make_shared<Motor>(2);
	std::shared_ptr<Motor> bottomLeft = std::make_shared<Motor>(20);
	std::shared_ptr<Motor> bottomRight = std::make_shared<Motor>(-1);

	bottomLeft->setBrakeMode(AbstractMotor::brakeMode::coast);
	bottomLeft->setGearing(AbstractMotor::gearset::red);
	bottomRight->setBrakeMode(AbstractMotor::brakeMode::coast);
	bottomRight->setGearing(AbstractMotor::gearset::red);
	topLeft->setBrakeMode(AbstractMotor::brakeMode::coast);
	topLeft->setGearing(AbstractMotor::gearset::red);
	topRight->setBrakeMode(AbstractMotor::brakeMode::coast);
	topRight->setGearing(AbstractMotor::gearset::red);

	left = std::make_shared<ADIEncoder>(1,2,false);
	right = std::make_shared<ADIEncoder>(7,8,false);
	middle = std::make_shared<ADIEncoder>(3,4,false);

	master = std::make_shared<Controller>();
	std::string out("line");
	master->setText(0,0,"init");
	master->setText(1,1,out);
	master->setText(2,2,"hi");
	
	model = std::make_shared<ThreeEncoderXDriveModel>(
		topLeft,
		topRight,
		bottomRight,
		bottomLeft,
		left,
		right,
		middle,
		100,
		12000
	);

	odom = std::make_shared<CustomOdometry>(
		model,
		ChassisScales({2.8114_in,9.883_in,.01_in,2.8114_in},360),
		TimeUtilFactory().create()
	);
	odom->startTask();
	odom->setState(State(0_in, 0_in, 0_deg));

	//*
	controller = std::make_shared<OdomXController>(
		model,
		odom,
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.0045,.0000,.0001,.00},
			TimeUtilFactory::withSettledUtilParams(75, 10, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.02,.0000,.001,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.05,.0000,.0000,.00},
			TimeUtilFactory::withSettledUtilParams(25, 5, 250_ms)),
		TimeUtilFactory().create()
	);//*/

	follower = std::make_shared<PathFollower>(
		model,
		odom,
		ChassisScales({4_in,9_in},imev5GreenTPR),
		4_in,
		TimeUtilFactory().create()
	);

	PursuitLimits limits(
		0.2_mps,  1.1_mps2, 1_mps,
        0.4_mps2, 0_mps,    40_mps
	);

	paths.insert(
		{
			"test",
			PathGenerator::generate(
				SimplePath({
					{20.0_in,27.0_in},
					{40.0_in,26.4_in},
					{47.2_in,26.9_in},
					{60.8_in,28.6_in}/*,
					{51.3_in,41.0_in},
					{50.1_in,50.1_in},
					{33.2_in,53.2_in}//*/
				})
				.generate(1_cm)
				.smoothen(.001, 1e-10 * meter),
				limits
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

	pros::delay(500);//wait for ADIEncoders to catch up

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() {

				auto time = pros::millis();

				odom->setState(State(7_in, 27_in, 90_deg));

				intake->moveVoltage(12000);
				follower->followPath(paths.at("test"));
/*
				controller->moveDistance(-30_in);

				controller->turnToPoint(Vector{-18_in,0_in});
				controller->strafeToPoint(Vector{13.2_in,12.0_in});
				intake->moveVoltage(-12000);
				pros::delay(350);
				intake->moveVoltage(12000);
				pros::delay(100);
				intake->moveVoltage(0);

				trayController->flipDisable(false);
				trayController->setTarget(4950);
				pros::delay(500);
				trayController->waitUntilSettled();

				intake->moveVoltage(-8000);
				trayController->setTarget(0);
				controller->strafeToPoint(Vector{21_in,21_in});
				intake->moveVoltage(0);
//*/

				master->setText(0,0,std::to_string(pros::millis()-time));
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
	trayController->flipDisable(true);

	selector->run();

	trayController->flipDisable(false);
}

void opcontrol() {

	odom->setState(State(0_in,0_in, 0_deg));

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

		if(tray->getEncoder()->get()>2000){
			forward /= 2;
			right /= 2;
			yaw /= 2;
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
			{
				intake->moveVoltage(12000);
			}
			intakeToggle = false;
		}else if(master->getDigital(ControllerDigital::R2)){
			{
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
			trayController->setTarget(4950);
			pros::delay(20);
		}else if(master->getDigital(ControllerDigital::L2)){
			//go back down
			trayController->setTarget(trayDown);
			liftController->setTarget(liftDown);
			pros::delay(250);

			while(master->getDigital(ControllerDigital::L2)){
				pros::delay(20);
				intake->moveVelocity(-200);
			}
		}

		//LIFT
		if(master->getDigital(ControllerDigital::down)){
			/*
			//liftController up to middle
			liftController->setTarget(liftMiddle);
			trayController->setTarget(trayMiddleDown);
			intake->moveVoltage(-8000);
			pros::delay(250);
			intake->moveVoltage(0);
			if(lift->getEncoder()->get()>1000){
				liftController->setTarget(liftDown);
				trayController->setTarget(trayDown);
				controller->moveDistance(-6_ft);
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
				intake->moveVoltage(-8000);
				pros::delay(250);
				intake->moveVoltage(0);
//				controller->moveDistance(-6_in);
			}
			while(master->getDigital(ControllerDigital::right)){
				pros::delay(20);
			}
		}else if(master->getDigital(ControllerDigital::B)){
			/*
			//stack grab
			intake->moveVelocity(200);
			trayController->setTarget(trayDown);
			pros::delay(500);
			liftController->setTarget(liftDown);
			while(master->getDigital(ControllerDigital::down)){
				pros::delay(20);
			}//*/
		}
		
		pros::delay(20);
	}
}
