#include "muphry/autons.hpp"

bool Auton::small(bool red){
	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	//flipout
	intake->moveVelocity(-100);
	viciousTrayController->setTarget(.41*4095);
	pros::delay(1000);
//	backwardChassis->waitUntilSettled();
	intake->moveVelocity(0);
	viciousTrayController->setTarget(.0001*4095);
	pros::delay(500);

	//grab 4 cubes
	intake->moveVelocity(100);
//	backwardModel->setMaxVelocity(150);
//	backwardChassis->moveDistance(40_in);

	//move back
//	backwardChassis->setMaxVelocity(200);
//	backwardChassis->moveDistanceAsync(-20_in);
	pros::delay(1000);
	intake->moveVelocity(0);
//	backwardChassis->waitUntilSettled();

	//turn to zone
//	backwardModel->setMaxVelocity(150);
	if(red){
//		backwardChassis->turnAngle(120_deg);
	}else{
//		backwardChassis->turnAngle(-120_deg);
	}
	pros::delay(10);

	//move to zone
//	backwardChassis->moveDistanceAsync(17_in);
	intake->moveVelocity(-100);
	pros::delay(500);
	intake->moveVelocity(100);
	pros::delay(500);
	intake->moveVelocity(0);
//	backwardChassis->waitUntilSettled();

	//stack
	viciousTrayController->setTarget(.41*4095);
	pros::delay(500);
	intake->moveVelocity(100);
	pros::delay(500);
	intake->moveVelocity(0);
	viciousTrayController->waitUntilSettled();

	//move away
	viciousTrayController->setTarget(.0001*4095);
//	backwardChassis->moveDistanceAsync(-18_in);
	pros::delay(250);
	intake->moveVelocity(-100);
//	backwardChassis->waitUntilSettled();
	intake->moveVelocity(0);

    //end
    return true;
}

bool Auton::test(bool turn){
	if(turn){
        auto angle = 360_deg;
//        chassis->turnAngle(angle);
        pros::delay(500);
//        chassis->turnAngle(-angle);
        pros::delay(500);
    }else if(turn){
        auto distance = 24_in;
//        backwardChassis->moveDistance(distance);
        pros::delay(500);
//        backwardChassis->moveDistance(-distance);
        pros::delay(500);
    }

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