#include "muphry/robot.hpp"
#include "muphry/autons.hpp"

using namespace lib7842;
using namespace okapi;
using namespace pros::c;

//controller
std::shared_ptr<ChassisControllerIntegrated> chassis;
std::shared_ptr<SkidSteerModel> model;
std::shared_ptr<SkidSteerModel> imodel;
std::shared_ptr<TwoEncoderOdometry> odom;

std::shared_ptr<OdomController> controller;

//gains
/*std::unique_ptr<IterativePosPIDController> distance;
std::unique_ptr<IterativePosPIDController> turn;
std::unique_ptr<IterativePosPIDController> angle;
QLength settleRadius(1_ft);
TimeUtil timeUtil(TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms));//*/

//intake
std::shared_ptr<MotorGroup> intake;

//tray
std::shared_ptr<MotorGroup> tray;
std::shared_ptr<Potentiometer> trayPotent;
std::shared_ptr<AsyncPosPIDController> trayController;
std::shared_ptr<AsyncPosPIDController> viciousTrayController;

//controller
std::shared_ptr<Controller> master;

//screen
std::shared_ptr<GUI::Screen> screen;
GUI::Selector* selector;

//task functions
void my_task_fn(void* param){
	while(true){
		odom->step();
		pros::delay(10);
	}
}

//Task
std::shared_ptr<pros::Task> odomTask;

void initialize() {
	pros::delay(500);//wait for ADIEncoders to catch up
	printf("init\n");

	master = std::make_shared<Controller>();

	master->clear();
	pros::delay(10);
	master->setText(0,0,"initialize");	
	pros::delay(10);

	MotorGroup leftSide({1,2});
	MotorGroup rightSide({-9,-20});

	model = std::make_shared<SkidSteerModel>(
		std::make_shared<MotorGroup>(leftSide),
		std::make_shared<MotorGroup>(rightSide),
		leftSide.getEncoder(),
		rightSide.getEncoder(),
		200,
		12000
	);
	model->setEncoderUnits(AbstractMotor::encoderUnits(0));
	model->setGearing(AbstractMotor::gearset::green);

/*
	imodel = std::make_shared<SkidSteerModel>(
		std::make_shared<MotorGroup>(leftSide),
		std::make_shared<MotorGroup>(rightSide),
		std::make_shared<ADIEncoder>(1,2,true),
		std::make_shared<ADIEncoder>(7,8,false),
		200,
		12000
	);
	imodel->setEncoderUnits(AbstractMotor::encoderUnits(0));
	imodel->setGearing(AbstractMotor::gearset::green);	//*/

	chassis = std::make_shared<ChassisControllerIntegrated>(
		TimeUtilFactory::withSettledUtilParams(50,5,250_ms),
		model,
		std::make_unique<AsyncPosIntegratedController>(std::make_shared<MotorGroup>(leftSide),AbstractMotor::GearsetRatioPair{AbstractMotor::gearset::green,1}, 200,TimeUtilFactory::withSettledUtilParams(50,5,250_ms)),
		std::make_unique<AsyncPosIntegratedController>(std::make_shared<MotorGroup>(rightSide),AbstractMotor::GearsetRatioPair{AbstractMotor::gearset::green,1}, 200,TimeUtilFactory::withSettledUtilParams(50,5,250_ms)),
		AbstractMotor::gearset::green,
		ChassisScales({3.25_in, 11_in},imev5GreenTPR)
	);
	chassis->setMaxVelocity(100);

	odom = std::make_shared<TwoEncoderOdometry>(
		TimeUtilFactory::withSettledUtilParams(50,5,250_ms),
		model,
		ChassisScales({3.5_in, 10_in}, 360)
	);
    std::string text("PROS");
	odomTask = std::make_shared<pros::Task>(my_task_fn, &text, "");

	odom->setState(OdomState{0_ft,0_ft,0_deg}, StateMode::CARTESIAN);
/*
	distance = std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
		{.002,.00001,.00003,.00},
		TimeUtilFactory::withSettledUtilParams(75, 5, 250_ms));
	turn = std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
		{.01,.0000,.000,.00},
		TimeUtilFactory::withSettledUtilParams(10, 5, 250_ms));
	angle = std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
		{.0005,.0000,.0000,.00},
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms));//*/

	//*
	controller = std::make_shared<OdomController>(
		model,
		odom,
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.002,.00001,.00003,.00},
			TimeUtilFactory::withSettledUtilParams(75, 5, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
		{.01,.0000,.000,.00},
		TimeUtilFactory::withSettledUtilParams(10, 5, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.0005,.0000,.0000,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		1_ft,
		TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)
	);//*/

	intake = std::make_shared<MotorGroup>(MotorGroup({11,-18}));
	intake->setGearing(AbstractMotor::gearset::red);

	pros::c::adi_pin_mode(3,INPUT_ANALOG);

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
		.005,
		.00003,
		.00005,
		.0
	);
	viciousTrayController->startThread();
	viciousTrayController->flipDisable(true);

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );
	screen->startTask("screenTask");

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() {
				Auton::simple();
			})
			.button("Skills", [&]() { 
				printf("skills\n");
				Auton::flipout();
				Auton::skills();
			 })
			.newRow()
			.button("Red Big", [&]() { 
				
			 })
			.button("Red Small", [&]() { 
				printf("redSmall");
				Auton::flipout();
				Auton::small(true);
			 })
			.newRow()
			.button("Blue Big", [&](){
				controller->moveDistance(12_in);
				controller->moveDistance(-14_in);

				intake->moveVelocity(-100);
				trayController->setTarget(.45*4095);
//				controller->moveDistanceAsync(3_in);
				trayController->waitUntilSettled();
//				controller->waitUntilSettled();
				intake->moveVelocity(0);
				trayController->setTarget(.0001*4095);
				trayController->waitUntilSettled();
//				controller->moveDistanceAsync(-3.5_in);

				printf("blueBig");
				auto angle = 110_deg;
				auto distance = 32_in;
//				controller->setMaxVelocity(200);

				//*
//				controller->moveDistanceAsync(24_in);
				intake->moveVelocity(100);
//				controller->waitUntilSettled();
//				controller->moveDistance(6_in);

//				controller->setMaxVelocity(100);
				controller->turnAngle(angle);
				controller->moveDistance(distance);
				//*/

				intake->moveVelocity(-100);
				pros::delay(1000);

				tray->moveVelocity(100);
				trayController->setTarget(.45 * 4095);
				trayController->waitUntilSettled();
				trayController->setTarget(.0001 * 4095);
				intake->moveVelocity(-100);

//				controller->moveDistance(-distance);
				trayController->waitUntilSettled();
				controller->turnAngle(-angle);
				intake->moveVelocity(0);				
			 })
			.button("Blue Small", [&]() { 
				printf("blueSmall");
				Auton::flipout();
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
	pros::delay(10);
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
		.withSeries("TrayController7", LV_COLOR_MAKE(239,71,111), [](){ return trayPotent->controllerGet(); });


}

void disabled() {
	master->clear();
	pros::delay(10);
	master->setText(0,0,"disabled");	
	pros::delay(10);
}

void competition_initialize() {

	master->clear();
	pros::delay(10);
	master->setText(0,0,"comp init");	
	pros::delay(10);	
}

void autonomous() {
	master->clear();
	pros::delay(10);
	master->setText(0,0,"autonomous");
	pros::delay(10);

	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	model->resetSensors();
	model->setMaxVelocity(100);

//	model->waitUntilSettled();

	selector->run();

	viciousTrayController->flipDisable(true);
	trayController->flipDisable(false);

}

void opcontrol() {

	master->clear();
	pros::delay(10);

//	pros::Task task( taskFnc, NULL, "taskFnc" );
	bool intakeToggle = false;

	std::string out0( std::to_string(odom->getState().x.convert(inch)) );
	out0.append(" in");
	std::string lastOut0( std::to_string(odom->getState().x.convert(inch)) );
	lastOut0.append(" in");

	std::string out1( std::to_string(adi_analog_read(3)) );
	out1.append(" val");
	std::string lastOut1( std::to_string(adi_analog_read(3)) );
	lastOut1.append(" val");

	while (true) {

		
		if( std::to_string(odom->getState().x.convert(inch)) + " in" == lastOut0 ){
			out0 = std::to_string(odom->getState().x.convert(inch)) + " in";
			master->setText(0,0,out0);
		}

		if( std::to_string(adi_analog_read(3)) + " val" == lastOut1 ){
			out1 = std::to_string(adi_analog_read(3)) + " val";
			master->setText(1,0,out1);
		}

		double left;
		double right;			
		if( std::abs(master->getAnalog(ControllerAnalog::rightY)) <= .1){
			left = master->getAnalog(ControllerAnalog::leftX);
			right = -master->getAnalog(ControllerAnalog::leftX);
//			model->setMaxVoltage(800);
		}else{
			left = master->getAnalog(ControllerAnalog::rightY) + (.75 * master->getAnalog(ControllerAnalog::leftX));
			right = master->getAnalog(ControllerAnalog::rightY) + (-.75 * master->getAnalog(ControllerAnalog::leftX));
//			model->setMaxVoltage(1200);
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
			intake->moveVelocity(50);
		}else{
			intake->moveVelocity(0);
		}

		//TRAY
		const double trayUp = .45 * 4095;   //old potentiometer 2625 //tray internal encoder 2000
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

		if(master->getDigital(ControllerDigital::down)){
			autonomous();
		}

		if(master->getDigital(ControllerDigital::B)){
			Auton::stackInTower();

		}		

		overheatWarn();

	}
}
