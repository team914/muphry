#include "muphry/robot.hpp"
/*
void overheatWarn(){
//	while(true){
		std::string out;
		//Tray Optimal || Tray Over Temp || Tray Over Crnt || Tray WARNING
		//Intk Optimal || Intk Over Temp || Intk Over Crnt || Intk WARNING
		//Drve Optimal || Drve Over Temp || Drve Over Crnt || Drve WARNING	

		const double maxTemp = 60;
		//TRAY
		if(tray->getTemperature()<maxTemp && !tray->isOverCurrent()){
			master->clearLine(0);
			out = "TrayTmp" + std::to_string(tray->getTemperature());
			master->setText(0,0,out.c_str());
		}else if(tray->getTemperature()>=maxTemp && !tray->isOverCurrent()){
			master->clearLine(0);
			master->setText(0,0,"Tray Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(!tray->getTemperature()>=maxTemp && tray->isOverTemp()){
			master->clearLine(0);
			master->setText(0,0,"Tray Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}
		//INTAKE
		if(intake->getTemperature()<maxTemp && !intake->isOverCurrent()){
			master->clearLine(1);
			out = "IntkTmp" + std::to_string(intake->getTemperature());
			master->setText(1,0,out.c_str());
		}else if(intake->getTemperature()>=maxTemp && !intake->isOverCurrent()){
			master->clearLine(1);
			master->setText(1,0,"Intk Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(!intake->getTemperature()>=maxTemp && intake->isOverTemp()){
			master->clearLine(1);
			master->setText(1,0,"Intk Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}
		//DRIVE
		if(model->getBottomLeftMotor()->getTemperature()<maxTemp && !model->getBottomLeftMotor()->isOverCurrent()){
			master->clearLine(1);
			out = "DrveTmp" + std::to_string(model->getBottomLeftMotor()->getTemperature());
			master->setText(1,0,out.c_str());
		}else if(model->getBottomLeftMotor()->getTemperature()>=maxTemp && !model->getBottomLeftMotor()->isOverCurrent()){
			master->clearLine(1);
			master->setText(1,0,"Drve Over Temp");
			master->rumble("- -");
			pros::delay(500);
		}else if(model->getBottomLeftMotor()->getTemperature()<maxTemp && !model->getBottomLeftMotor()->isOverTemp()){
			master->clearLine(1);
			master->setText(1,0,"Drve Over Crnt");
			master->rumble("- -");
			pros::delay(500);
		}
		pros::delay(20);
//	}
}//*/
