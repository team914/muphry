#include "muphry/autons.hpp"

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;

bool Auton::alignCubes(){
				intake->moveVoltage(-12000);
				pros::delay(550);
				intake->moveVoltage(12000);
				pros::delay(100);
				intake->moveVoltage(0);
				return true;
}

bool Auton::stackForward(const Vector& vector){
				trayController->flipDisable(false);
				trayController->setTarget(4950);
				pros::delay(2300);
				//controller->strafeToPoint(vector);
				return true;
}

bool Auton::stackBackward(const Vector& vector){
				intake->moveVoltage(-8000);
				trayController->setTarget(0);
				//controller->strafeToPoint(vector,OdomController::makeAngleCalculator(odom->getState().theta));
				return true;
}

bool Auton::small(bool red){

				odom->setState(State(7_in, 27_in, 90_deg));

				intake->moveVoltage(12000);

				follower->followPath(paths.at("redSide"));
//*
				controller->turnToPoint(Vector{20_in,23_in});
				//controller->strafeToPoint(Vector{20_in,23_in});

				controller->turnToPoint(Vector{10.27_in,14.75_in});
				//controller->strafeToPoint(Vector{10.27_in,14.75_in},OdomController::makeAngleCalculator(-131_deg));
				Auton::alignCubes();
				stackForward(Vector(State(odom->getState())));
				pros::delay(500);
				Auton::stackBackward(Vector{18.61_in,30.50_in});//*/

				intake->moveVoltage(0);
}

bool Auton::skills(){
/*
	odom->setState(State(7_in, 27_in, 90_deg));

	intake->moveVelocity(12000);

//*
	//controller->strafeToPoint(Vector{21.9_in,27.7_in}, OdomController::makeAngleCalculator(91.5_deg));
	pros::delay(250);
	//controller->strafeToPoint(Vector{31.9_in,27.7_in}, OdomController::makeAngleCalculator(91.5_deg));
	pros::delay(250);
	//controller->strafeToPoint(Vector{44.4_in,28.0_in}, OdomController::makeAngleCalculator(87.2_deg));
	pros::delay(250);
	//controller->strafeToPoint(Vector{53.4_in,28.0_in}, OdomController::makeAngleCalculator(87.2_deg));
	pros::delay(250);
	//controller->strafeToPoint(Vector{44.4_in,28.0_in}, OdomController::makeAngleCalculator(87.2_deg));
	pros::delay(250);
	//controller->strafeToPoint(Vector{57.3_in,38.5_in}, OdomController::makeAngleCalculator(-2.0_deg));
	pros::delay(250);
//*/

	model->setMaxVoltage(10000);
	odom->setState(State(0_in, 0_in, 90_deg));

	intake->moveVoltage(12000);

	controller->moveDistanceAtAngle(23_in,OdomController::makeAngleCalculator(90_deg),0);
	controller->moveDistanceAtAngle(6_in,OdomController::makeAngleCalculator(90_deg),0);

	liftController->setTarget(liftUp/3);
	trayController->setTarget(trayMiddleUp);
	pros::delay(500);
	controller->moveDistanceAtAngle(6_in,OdomController::makeAngleCalculator(90_deg),0);
	liftController->setTarget(liftDown);
	trayController->setTarget(trayDown);
	pros::delay(1000);

	model->setMaxVoltage(12000);
//	controller->turnToAngle(63.5_deg);//this worked at one point
	controller->moveDistanceAtAngle(-25_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	controller->turnToAngle(0_deg);

	controller->moveDistanceAtAngle(-26_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	controller->turnToAngle(85_deg);

//	controller->turnToAngle(72_deg);//this worked at one point

	model->setMaxVoltage(4000);
//	controller->moveDistanceAtAngle(20_in,OdomController::makeAngleCalculator(odom->getState().theta),0);

	intake->moveVoltage(-12000);
	pros::delay(250);
	intake->moveVoltage(12000);

//	controller->moveDistanceAtAngle(20_in,OdomController::makeAngleCalculator(odom->getState().theta),0);

	return true;
}

bool Auton::test(bool turn){
/*	odom->setState(State(7_in, 27_in, 90_deg));
	intake->moveVoltage(12000);
	//controller->strafeToPoint(Vector{31_in,27_in},OdomController::makeAngleCalculator(odom->getState().theta) );
	controller->turnToPoint(Vector{14_in,51_in});	
	//controller->strafeToPoint(Vector{14_in,51_in},OdomController::makeAngleCalculator(odom->getState().theta));	
	//controller->strafeToPoint(Vector{10_in,27_in},OdomController::makeAngleCalculator(90_deg));
	//controller->strafeToPoint(Vector{7_in,27_in},OdomController::makeAngleCalculator(90_deg));
	intake->moveVoltage(-12000);
	pros::delay(3000);
	intake->moveVoltage(0);
	//*/
	odom->setState(State(0_in, 0_in, 90_deg));
	controller->turnAngle(60_deg);
	controller->turnAngle(-60_deg);
}

bool Auton::moveToPoint(Vector vector){
	controller->turnToPoint(vector);
	pros::delay(10);
	//controller->strafeToPoint(vector);
}

bool Auton::random(){

    //one cube
	printf("redBig");
//	backwardChassis->moveDistance(-5_in);
//	backwardChassis->moveDistance(10_in);
//	backwardChassis->turnAngle(67_deg);    

    //grab stack
//	backwardChassis->moveDistanceAsync(36_in);
	intake->moveVelocity(100);
//	backwardChassis->waitUntilSettled();
	intake->moveVelocity(-100);
	pros::delay(500);
	intake->moveVelocity(100);
	pros::delay(500);
	intake->moveVelocity(0);
	trayController->flipDisable(false);
	trayController->setTarget(.41 * 4095);
	trayController->waitUntilSettled();
	trayController->setTarget(.0001 * 4095);
//	backwardChassis->moveDistanceAsync(-12_in);
	intake->moveVelocity(-100);
//	backwardChassis->waitUntilSettled();
	intake->moveVelocity(0);				 

}