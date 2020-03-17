#include "muphry/robot.hpp"
#include "muphry/autons.hpp"


using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;


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
		200,
		12000
	);

	odom = std::make_shared<CustomOdometry>(
		model,
		ChassisScales({2.8114_in,9.883_in,.01_in,2.8114_in},360),
		TimeUtilFactory().create()
	);
	odom->setState(State(0_in, 0_in, 0_deg));

	controller = std::make_shared<OdomXController>(
		model,
		odom,
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.0045,.0000,.0001,.00},
			TimeUtilFactory::withSettledUtilParams(75, 10, 250_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.02,.0000,.001,.00},
			TimeUtilFactory::withSettledUtilParams(15, 5, 100_ms)),
		std::make_unique<IterativePosPIDController>(IterativePosPIDController::Gains
			{.017,.0000,.0000,.00},
			TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms)),
		TimeUtilFactory().create()
	);

	follower = std::make_shared<PathFollower>(
		model,
		odom,
		ChassisScales({4_in,9_in},imev5GreenTPR),
		4_in,
		TimeUtilFactory().create()
	);

	screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(38,84,124) );
	screen->startTask("screenTask");

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() {
				Auton::skills();
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
		});

	screen->makePage<GUI::Graph>("Temp")
		.withRange(0,100)
		.withGrid(2,4)
		;

	printf("init end\n");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	auto time = pros::millis();

	selector->run();
	
	master->setText(0,0,std::to_string(pros::millis()-time));
}

void opcontrol() {

	printf("opcontrol\n");

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



		pros::delay(20);
	}
}

