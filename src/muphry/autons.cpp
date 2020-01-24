#include "muphry/autons.hpp"


bool Auton::flipout(){
	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	//flipout
	intake->moveVelocity(-100);
	pros::delay(750);
	intake->moveVelocity(0);
	pros::delay(500);

	return true;	
}

bool Auton::stack(){
	viciousTrayController->setTarget(.41*4095);
	intake->moveVelocity(0);
	pros::delay(500);				
	viciousTrayController->waitUntilSettled();
}

bool Auton::stackInTower(){
			viciousTrayController->flipDisable(false);
			trayController->flipDisable(true);

			chassis->moveDistance(-12_in);
			viciousTrayController->setTarget(.41*4095);
			pros::delay(500);
			chassis->moveDistance(9_in);
			viciousTrayController->waitUntilSettled();
			viciousTrayController->setTarget(.0001*4095);

			pros::delay(500);
			intake->moveVelocity(100);
			chassis->moveDistance(7_in);
			intake->moveVelocity(0);

			viciousTrayController->flipDisable(true);
			trayController->flipDisable(false);	
}

bool Auton::simple(){
	chassis->moveDistance(-12_in);
	chassis->moveDistance(12_in);
	flipout();
	stack();
}

bool Auton::small(bool red){

	chassis->stop();
	pros::delay(500);
	model->resetSensors();
	if(red){
		odom->setState(OdomState{1.75_in,25.5_in,90_deg}, StateMode::CARTESIAN);
	}else{
		odom->setState(OdomState{58.25_in,25.5_in,90_deg}, StateMode::CARTESIAN);
	}
	pros::delay(10);
//*
	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	//grab 4 cubes
	intake->moveVelocity(100);
	model->setMaxVelocity(100);
	chassis->moveDistance(45_in);

	//move back
	model->setMaxVelocity(200);
	chassis->moveDistance(-28_in);
//	pros::delay(1000);
	intake->moveVelocity(0);
//	controller->waitUntilSettled();

	//turn to zone
	model->setMaxVelocity(150);
	auto angle = 127_deg;
	if(red){
		chassis->turnAngle(angle);
	}else{
		chassis->turnAngle(-angle);
	}
	pros::delay(10);
//*

	//move to zone
	chassis->moveDistanceAsync(12_in);
	intake->moveVelocity(-100);
	pros::delay(350);
	intake->moveVelocity(100);
	pros::delay(250);
	intake->moveVelocity(0);
	pros::delay(100);
	chassis->waitUntilSettled();		

	//stack
	Auton::stack();

	//move away
	intake->moveVelocity(-100);
	chassis->moveDistance(-18_in);
	viciousTrayController->setTarget(.0001*4095);				//*/

	intake->moveVelocity(0);

	//*/
    //end
    return true;
}

bool Auton::skills(){
	//*
	chassis->stop();
	pros::delay(500);
	model->resetSensors();

	pros::delay(10);
//*
	viciousTrayController->flipDisable(false);
	trayController->flipDisable(true);

	//grab 4 cubes
	intake->moveVelocity(100);
	model->setMaxVelocity(100);
	chassis->moveDistance(45_in);
	//*/

	//manauver to cube
	chassis->turnAngle(45_deg);
	chassis->moveDistance(-18_in);
	chassis->turnAngle(-45_deg);

	//go to cube
	intake->moveVelocity(100);
	chassis->moveDistance(17_in);
	pros::delay(500);
	intake->moveVelocity(0);

	pros::delay(500);
	intake->moveVelocity(0);


				chassis->moveDistance(-40_in);
				chassis->moveDistanceAsync(40_in);
				pros::delay(500);
				intake->moveVelocity(-100);
				pros::delay(350);
				intake->moveVelocity(100);
				pros::delay(350);
				intake->moveVelocity(0);
				pros::delay(100);
				chassis->waitUntilSettled();

				//stackInTower
				Auton::stackInTower();
				viciousTrayController->flipDisable(false);
				trayController->flipDisable(true);

				chassis->moveDistance(-16_in);
				chassis->turnAngle(135_deg);
				chassis->moveDistance(21_in);
				pros::delay(500);

				viciousTrayController->setTarget(.41*4095);
				intake->moveVelocity(0);
				pros::delay(500);				
				viciousTrayController->waitUntilSettled();

				viciousTrayController->setTarget(.0001*4095);
				intake->moveVelocity(-100);
				chassis->moveDistance(-18_in);
				intake->moveVelocity(0);	
}

bool Auton::test(bool turn){
	if(turn){
        auto angle = 360_deg;
        controller->turnAngle(angle);
        pros::delay(500);
        controller->turnAngle(-angle);
        pros::delay(500);
    }else if(turn){
        auto distance = 24_in;
        controller->moveDistance(distance);
        pros::delay(500);
        controller->moveDistance(-distance);
        pros::delay(500);
    }
	return true;
}

bool Auton::random(){
/*
    //one cube
	printf("redBig");
	controller->moveDistance(-5_in);
	controller->moveDistance(10_in);
	controller->turnAngle(67_deg);    

    //grab stack
	controller->moveDistanceAsync(36_in);
	intake->moveVelocity(100);
	controller->waitUntilSettled();
	intake->moveVelocity(-100);
	pros::delay(500);
	intake->moveVelocity(100);
	pros::delay(500);
	intake->moveVelocity(0);
	trayController->flipDisable(false);
	trayController->setTarget(.41 * 4095);
	trayController->waitUntilSettled();
	trayController->setTarget(.0001 * 4095);
	controller->moveDistanceAsync(-12_in);
	intake->moveVelocity(-100);
	controller->waitUntilSettled();
	intake->moveVelocity(0);				 
//*/
	return true;
}