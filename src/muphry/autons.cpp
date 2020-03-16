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
	odom->setState(State(0_in, 0_in, 90_deg));

	liftController->setTarget(liftMiddle);
	trayController->setTarget(trayMiddleUp);
	intake->moveVelocity(-12000);

	pros::delay(1000);

	liftController->setTarget(liftDown);
	trayController->setTarget(trayDown);
	intake->moveVelocity(0);

	pros::delay(1000);

	model->setMaxVoltage(6000);
	pros::delay(10);

	intake->moveVoltage(12000);
	pros::delay(10);

	controller->moveDistanceAtAngle(12_in,OdomController::makeAngleCalculator(90_deg),0);

/*
	controller->moveDistanceAtAngle(55_in,OdomController::makeAngleCalculator(90_deg),0);
	pros::delay(10);
	intake->moveVoltage(0);
	
	controller->moveDistanceAtAngle(-12_in,OdomController::makeAngleCalculator(90_deg),0);
	pros::delay(10);

	controller->turnToAngle(220_deg);//this worked at one point

	pros::delay(10);

	controller->moveDistanceAtAngle(36_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	trayController->flipDisable(false);
	trayController->setTarget(trayUp);
	pros::delay(10);

	intake->moveVoltage(-10000);
	pros::delay(200);
	intake->moveVoltage(0);
	pros::delay(2000);

	intake->moveVoltage(-12000);
	pros::delay(300);
	intake->moveVoltage(0);

	controller->moveDistanceAtAngle(-18_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	trayController->setTarget(trayDown);
	
	lift

	intake->moveVoltage(0);

	return true;//*/
}

bool Auton::skills(){
/*
	odom->setState(State(7_in, 27_in, 90_deg));

	intake->moveVelocity(12000);

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
//*
	odom->setState(State(0_in, 0_in, 90_deg));

	liftController->setTarget(liftMiddle);
	trayController->setTarget(trayMiddleUp);
	intake->moveVelocity(-12000);

//	controller->moveDistanceAtAngle(12_in,OdomController::makeAngleCalculator(90_deg),0);
	controller->turnToAngle(90_deg);//this worked at one point

	pros::delay(1000);

	liftController->setTarget(liftDown);
	trayController->setTarget(trayDown);
	intake->moveVelocity(0);

	pros::delay(1000);
	controller->turnToAngle(90_deg);//this worked at one point

	model->setMaxVoltage(6000);
	pros::delay(10);

	intake->moveVoltage(12000);
	pros::delay(10);

	controller->moveDistanceAtAngle(50_in,OdomController::makeAngleCalculator(90_deg),0);
	pros::delay(10);
	intake->moveVoltage(0);
	
	controller->moveDistanceAtAngle(-37_in,OdomController::makeAngleCalculator(90_deg),0);
	pros::delay(10);

	controller->turnToAngle(220_deg);//this worked at one point

	pros::delay(10);
//*
	controller->moveDistanceAtAngle(16_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	trayController->flipDisable(false);
	trayController->setTarget(trayUp);
	pros::delay(10);

	intake->moveVoltage(-10000);
	pros::delay(200);
	intake->moveVoltage(0);
	pros::delay(2000);

	intake->moveVoltage(-12000);
	pros::delay(300);
	intake->moveVoltage(0);

	controller->moveDistanceAtAngle(-33_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	pros::delay(10);
	trayController->setTarget(trayDown);
	pros::delay(10);

	controller->turnToAngle(90_deg);//this worked at one point

	intake->moveVoltage(12000);

	controller->moveDistanceAtAngle(30_in,OdomController::makeAngleCalculator(odom->getState().theta),0);

	pros::delay(250);
	intake->moveVoltage(0);

	liftController->setTarget(liftMiddle);
	
	controller->moveDistanceAtAngle(-12_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	intake->moveVoltage(-6000);
	controller->moveDistanceAtAngle(12_in,OdomController::makeAngleCalculator(odom->getState().theta),0);
	pros::delay(250);
	intake->moveVoltage(0);
	//*/
	
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
	odom->setState(State(0_in, 0_in, 0_deg));
	controller->moveDistance(-24_in);
	controller->moveDistance(24_in);

	liftController->setTarget(liftMiddle);
	trayController->setTarget(trayMiddleUp);
	intake->moveVelocity(-12000);

	pros::delay(1000);

	liftController->setTarget(liftDown);
	trayController->setTarget(trayDown);
	intake->moveVelocity(0);

	pros::delay(1000);	
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
