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
std::shared_ptr<pros::ADIAnalogIn> line;
std::shared_ptr<pros::ADIDigitalIn> button;

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

double liftUp = 2000;
double liftMiddle = 1400;
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
	std::shared_ptr<Motor> bottomLeft = std::make_shared<Motor>(1);
	std::shared_ptr<Motor> bottomRight = std::make_shared<Motor>(-20);

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
	line = std::make_shared<pros::ADIAnalogIn>(5);
	line->calibrate();
	button = std::make_shared<pros::ADIDigitalIn>(6);

	pros::delay(500);//wait for ADIEncoders to catch up

	master = std::make_shared<Controller>();
	std::string out("line");
	out.append( std::to_string( line->get_value()) );
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
		200,
		12000
	);

	odom = std::make_shared<CustomOdometry>(
		model,
		ChassisScales({2.8114_in,9.883_in,.01_in,2.8114_in},360),
		TimeUtilFactory().create()
	);
	odom->startTask();

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
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		TimeUtilFactory().create()
	);//*/

	follower = std::make_shared<PathFollower>(
		model,
		odom,
		ChassisScales({4_in,9_in},imev5GreenTPR),
		6_in,
		TimeUtilFactory::withSettledUtilParams(50,5,250_ms)
	);

	PursuitLimits limits(
		0.2_mps,  1.1_mps2, 0.75_mps,
        0.4_mps2, 0_mps,    40_mps
	);

	paths.insert(
		{
			"test",
			PathGenerator::generate(
				SimplePath({
					{0_ft,0_ft},
					{3_ft,3_ft},
					{3_ft,6_ft},
//					{1_ft,1_ft},
					{6_ft, 6_ft}})
				.generate(1_cm)
				.smoothen(.001, 1e-10 * meter),
				limits
			)
		}
	);

	intake = std::make_shared<MotorGroup>(MotorGroup({11,-18}));
	intake->setGearing(AbstractMotor::gearset::green);

	tray = std::make_shared<MotorGroup>(MotorGroup({-19}));
	tray->setGearing(AbstractMotor::gearset::red);

	trayPotent = std::make_shared<Potentiometer>(6);

	trayController = std::make_shared<AsyncPosPIDController>(
		tray->getEncoder(),
		tray,
		TimeUtilFactory::withSettledUtilParams(),
		.001,
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

				odom->setState(State(7_in, 27_in, 90_deg));
//*
				intake->moveVelocity(200);

				//move forward
				controller->driveToPoint2(Vector{2.3_ft,27_in});

				//too fast
				pros::delay(250);

				//grab 2 cubes
				controller->driveToPoint2(Vector{4.0_ft,27_in});

				

				/*
				//reposition
				controller->driveToPoint2(Vector{3.0_ft,3.5_ft});
				controller->turnToPoint(Vector{6_ft,3.5_ft});

				//grab orange cube
				controller->strafeToPoint(Vector{4.4_ft,3.5_ft});

				//move to scoring arcade
				controller->strafeToPoint(Vector{2_ft,2_ft});

				controller->turnToPoint(Vector{-1.4_ft,0_ft});
				controller->strafeToPoint(Vector{10_in,10_in});

				trayController->setTarget(4950);
				intake->moveVelocity(-200);
				pros::delay(500);
				intake->moveVelocity(0);
				trayController->waitUntilSettled();

				controller->strafeToPoint(Vector{2_ft,2_ft});
				intake->moveVelocity(100);
				//*/
				
/*
				//grab right purple
				controller->strafeToPoint(Vector{5.2_ft,2.4_ft});

				//grab left purple
				controller->strafeToPoint(Vector{5.2_ft,3.6_ft});
//*/

//				controller->strafeToPoint(State(67_in,87_in, 90_deg));
				
				intake->moveVelocity(0);

				master->setText(0,0,std::to_string(pros::millis()-time));
				pros::delay(5000);
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

	screen->makePage<GUI::Graph>("Line")
		.withRange(-4095,4095)
		.withGrid(8,8)
		.withSeries("Line", LV_COLOR_MAKE(6,214,160), []() { return line->get_value(); pros::delay(100); });

	screen->makePage<GUI::Graph>("Button")
		.withRange(0,127)
		.withGrid(8,8)
		.withSeries("Button", LV_COLOR_MAKE(6,214,160), []() { return button->get_value(); pros::delay(100); });


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

	trayController->flipDisable(false);

	bool intakeToggle = false;

	std::string out("line");
	out.append( std::to_string( line->get_value()) );
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
			forward /= 4;
			right /= 4;
			yaw /= 4;
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
			if(lift->getEncoder()->get()>1000){
				intake->moveVoltage(8000);
			}else{
				intake->moveVoltage(12000);
			}
			intakeToggle = false;
		}else if(master->getDigital(ControllerDigital::R2)){
			if(lift->getEncoder()->get()>1000){
				intake->moveVoltage(-8000);
			}else{
				intake->moveVoltage(-12000);
			}
			intakeToggle = false;
		}else if(intakeToggle){
			if(2500 <= line->get_value() <= 3500){
				intake->moveVoltage(12000);
			}else{
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
			trayController->waitUntilSettled();
			pros::delay(20);
		}else if(master->getDigital(ControllerDigital::L2)){
			//go back down
			trayController->setTarget(trayDown);
			liftController->setTarget(liftDown);
			pros::delay(250);
			intake->moveVelocity(-200);
			controller->moveDistance(-12_in);
			intake->moveVelocity(0);//*/

			while(master->getDigital(ControllerDigital::L2)){
				pros::delay(20);
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
				intake->moveVoltage(-12000);
				pros::delay(250);
				intake->moveVoltage(0);
				controller->moveDistance(-6_in);
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
