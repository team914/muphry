/**
 * @author     Acetousk
 * @date       2020
 */
#include "api.h"
#include "lib7842/api.hpp"
#include "muphry/robot.hpp"
#include "muphry/subsystems/chassis.hpp"
#include "muphry/subsystems/intake.hpp"
#include "muphry/subsystems/lift.hpp"
#include "muphry/subsystems/tilter.hpp"
#include <iostream>
#include <stdio.h>

using namespace lib7842;
using namespace lib7842::units;
using namespace okapi;
using namespace okapi::literals;

#define ROS 0
#define TERMINAL 1

pros::task_t graphing;
pros::task_t odomTask;

/**
 * @brief      loop for a task
 *
 * @param      params  The parameters
 */
void loop(void *params) {
  auto lposFilter = std::make_shared<AverageFilter<10>>();
  auto lspeedFilter = std::make_shared<AverageFilter<10>>();
  auto laccelFilter = std::make_shared<AverageFilter<10>>();
  auto ljerkFilter = std::make_shared<AverageFilter<10>>();
  auto lsnapFilter = std::make_shared<AverageFilter<10>>();

  auto rposFilter = std::make_shared<AverageFilter<10>>();
  auto rspeedFilter = std::make_shared<AverageFilter<10>>();
  auto raccelFilter = std::make_shared<AverageFilter<10>>();
  auto rjerkFilter = std::make_shared<AverageFilter<10>>();
  auto rsnapFilter = std::make_shared<AverageFilter<10>>();

  int cnt = 1;

  double dt{0};
  std::unique_ptr<AbstractTimer> loopDtTimer = std::make_unique<Timer>();

  double lpos{0}, llastPos{0};
  double lspeed{0}, llastSpeed{0};
  double laccel{0}, llastAccel{0};
  double ljerk{0}, llastJerk{0};
  double lsnap{0}, llastSnap{0};

  double rpos{0}, rlastPos{0};
  double rspeed{0}, rlastSpeed{0};
  double raccel{0}, rlastAccel{0};
  double rjerk{0}, rlastJerk{0};
  double rsnap{0}, rlastSnap{0};

  double tpr{imev5GreenTPR};
  double ratio{1};
  QLength wheelDiameter{4.036_in};

  while (true) {
    // get delta time
    dt = loopDtTimer->getDtFromMark().convert(second);

    // get position from encoder feed back and convert to meters
    lpos = lposFilter->filter(chassis->skidSteerModel->getSensorVals()[0]) / (tpr * ratio) *
           wheelDiameter.convert(meter) * PI;
    rpos = rposFilter->filter(chassis->skidSteerModel->getSensorVals()[1]) / (tpr * ratio) *
           wheelDiameter.convert(meter) * PI;

    // this will make it so that the sensors don't send crazy feedback at program beginning
    if (std::abs(lpos) >= 1000) lpos = 0;
    if (std::abs(rpos) >= 1000) rpos = 0;

    // calculate derivatives of speed to 4th degree
    if (llastPos != 0) lspeed = lspeedFilter->filter((lpos - llastPos) / dt);
    if (llastSpeed != 0) laccel = laccelFilter->filter((lspeed - llastSpeed) / dt);
    if (llastAccel != 0) ljerk = ljerkFilter->filter((laccel - llastAccel) / dt);
    if (llastJerk != 0) lsnap = lsnapFilter->filter((ljerk - llastJerk) / dt);

    if (rlastPos != 0) rspeed = rspeedFilter->filter((rpos - rlastPos) / dt);
    if (rlastSpeed != 0) raccel = raccelFilter->filter((rspeed - rlastSpeed) / dt);
    if (rlastAccel != 0) rjerk = rjerkFilter->filter((raccel - rlastAccel) / dt);
    if (rlastJerk != 0) rsnap = rsnapFilter->filter((rjerk - rlastJerk) / dt);

#if ROS == 1
    // publish values to ros node(s)
    posL_msg.data = lpos;
    velL_msg.data = lspeed;
    accL_msg.data = laccel;
    jrkL_msg.data = ljerk;
    snpL_msg.data = lsnap;
    posL.publish(&posL_msg);
    velL.publish(&velL_msg);
    accL.publish(&accL_msg);
    jrkL.publish(&jrkL_msg);
    snpL.publish(&snpL_msg);

    posR_msg.data = rpos;
    velR_msg.data = rspeed;
    accR_msg.data = raccel;
    jrkR_msg.data = rjerk;
    snpR_msg.data = rsnap;
    posR.publish(&posR_msg);
    velR.publish(&velR_msg);
    accR.publish(&accR_msg);
    jrkR.publish(&jrkR_msg);
    snpR.publish(&snpR_msg);
#endif

#if TERMINAL == 1
    std::string msg = "knmcs\t," + std::to_string((double)pros::millis() / 1000) + "\t," +
                      std::to_string(lpos) + "\t," + std::to_string(lspeed) + "\t," +
                      std::to_string(laccel) + "\t," + std::to_string(ljerk) + "\t," +
                      std::to_string(lsnap) + "\n";
    std::cout << msg;
#endif

    // update last derivative each loop
    llastPos = lpos;
    llastSpeed = lspeed;
    llastAccel = laccel;
    llastJerk = ljerk;
    llastSnap = lsnap;

    rlastPos = rpos;
    rlastSpeed = rspeed;
    rlastAccel = raccel;
    rlastJerk = rjerk;
    rlastSnap = rsnap;

    // place mark for timer
    loopDtTimer->placeMark();

    if (ROS) {
      // spin
      nh->spinOnce();
    }

    pros::delay(50);
  }
}

/**
 * @brief      loop for odom task
 *
 * @param      params  The parameters
 */
void odomFnc(void *params) {
  while (true) {
    chassis->odom->step();

    double x = chassis->odom->getState().x.convert(meter);
    double y = chassis->odom->getState().y.convert(meter);
    double theta = chassis->odom->getState().theta.convert(radian);

#if ROS == 1
    pose_msg.position.x = x;
    pose_msg.position.y = y;
    pose_msg.position.z = 0;
    pose_msg.orientation = tf::createQuaternionFromYaw(theta);
    pose.publish(&pose_msg);
#endif

#if TERMINAL == 1
    std::string msg = "odom\t," + std::to_string((double)pros::millis() / 1000) + "\t," +
                      std::to_string(x) + "\t," + std::to_string(y) + "\t," +
                      std::to_string(theta) + "\n";
    printf(msg.c_str());
#endif
    pros::delay(50);
  }
}

/**
 * @brief      Initialize.
 */
void initialize() {
  pros::delay(100); // wait for ADI Sensors to catch up

  Logger::setDefaultLogger(
    std::make_shared<Logger>(TimeUtilFactory::createDefault().getTimer(),
                             "/ser/sout",           // Output to the PROS terminal
                             Logger::LogLevel::warn // Show errors and warnings
                             ));

  printf("init\n");

  Intake::getIntake()->startTask();
  Tilter::getTilter()->startTask();
  Lift::getLift()->startTask();

  Intake::getIntake()->setNewState(IntakeState::hold);
  Tilter::getTilter()->setNewState(TilterState::down);
  Lift::getLift()->setNewState(LiftState::down);

  master = std::make_shared<Controller>();

  intakeUpBtn = std::make_shared<ControllerButton>(ControllerDigital::R1);
  intakeDownBtn = std::make_shared<ControllerButton>(ControllerDigital::R2);
  tilterUpBtn = std::make_shared<ControllerButton>(ControllerDigital::L1);
  tilterDownBtn = std::make_shared<ControllerButton>(ControllerDigital::L2);
  liftUpBtn = std::make_shared<ControllerButton>(ControllerDigital::right);
  liftMidBtn = std::make_shared<ControllerButton>(ControllerDigital::Y);

  screen = std::make_shared<GUI::Screen>(lv_scr_act(), LV_COLOR_MAKE(38, 84, 124));

  selector = dynamic_cast<GUI::Selector *>(
    &screen->makePage<GUI::Selector>("Skid Steer Selector")
       .button("Test PID",
               [&]() {
                 chassis->skidSteerModel->setMaxVoltage(11000);
                 chassis->pidController->startThread();

                 chassis->pidController->moveDistance(12_in);
                 Intake::getIntake()->setNewState(IntakeState::inFull);
                 chassis->pidController->moveDistance(15_in);
                 chassis->pidController->moveDistance(-5.5_in);

                 pros::delay(50);
                 chassis->pidController->turnAngle(-130_deg);
                 Intake::getIntake()->setNewState(IntakeState::outHalf);
                 pros::delay(500);
                 Intake::getIntake()->setNewState(IntakeState::hold);

                 chassis->pidController->moveDistance(17_in);

                 Tilter::getTilter()->setNewState(TilterState::up);

                 pros::delay(1900);

                 Intake::getIntake()->setNewState(IntakeState::outHalf);
                 Tilter::getTilter()->tilterController->waitUntilSettled();
                 Intake::getIntake()->setNewState(IntakeState::hold);

                 Intake::getIntake()->setNewState(IntakeState::outHalf);
                 Tilter::getTilter()->setNewState(TilterState::down);
                 Tilter::getTilter()->tilterController->waitUntilSettled();

                 chassis->pidController->waitUntilSettled();

                 chassis->pidController->stop();
                 chassis->pidController->startThread();

                 chassis->pidController->moveDistance(-17_in);

                 Intake::getIntake()->setNewState(IntakeState::hold);
                 chassis->pidController->stop();
               })
       .button("Straight PID",
               [&]() {
                 Intake::getIntake()->setNewState(IntakeState::hold);

                 chassis->skidSteerModel->setMaxVoltage(12000);
                 chassis->pidController->startThread();

                 chassis->pidController->moveDistance(36_in);
                 Intake::getIntake()->setNewState(IntakeState::inFull);
                 chassis->pidController->moveDistance(-18_in);
                 chassis->pidController->moveDistance(-12_in);
                 chassis->pidController->moveDistance(-6_in);

                 chassis->pidController->stop();

                 Intake::getIntake()->setNewState(IntakeState::hold);
               })
       .button("Turn PID",
               [&]() {
                 chassis->skidSteerModel->setMaxVoltage(10000);
                 chassis->pidController->startThread();
                 chassis->pidController->turnAngle(270_deg);
                 chassis->pidController->turnAngle(-270_deg);
                 chassis->pidController->stop();

                 Intake::getIntake()->setNewState(IntakeState::hold);
               })
       .newRow()
       .button("Test Profile",
               [&]() {
                 printf("running test profile\n");

                 chassis->leftProfileController->flipDisable(false);
                 chassis->rightProfileController->flipDisable(false);

                 chassis->linearProfileStraight(12_in);
                 Intake::getIntake()->setNewState(IntakeState::inFull);
                 chassis->linearProfileStraight(15_in);
                 chassis->linearProfileStraight(-5.5_in);

                 chassis->linearProfileTurn(-170_deg);
                 Intake::getIntake()->setNewState(IntakeState::outHalf);
                 pros::delay(500);
                 Intake::getIntake()->setNewState(IntakeState::hold);

                 chassis->linearProfileStraight(19_in);

                 Tilter::getTilter()->setNewState(TilterState::up);

                 pros::delay(1900);

                 Intake::getIntake()->setNewState(IntakeState::outHalf);
                 Tilter::getTilter()->tilterController->waitUntilSettled();

                 Tilter::getTilter()->setNewState(TilterState::down);
                 chassis->linearProfileStraight(-24_in);
                 Tilter::getTilter()->tilterController->waitUntilSettled();
                 Intake::getIntake()->setNewState(IntakeState::hold);
               })
       .button("Turn Profile",
               [&]() {
                 printf("running test profile\n");

                 chassis->leftProfileController->flipDisable(false);
                 chassis->rightProfileController->flipDisable(false);
                 chassis->linearProfileTurn(270_deg);
                 chassis->linearProfileTurn(-270_deg);
               })
       .button("Fwd Profile",
               [&]() {
                 printf("running test profile\n");

                 chassis->leftProfileController->flipDisable(false);
                 chassis->rightProfileController->flipDisable(false);
                 chassis->linearProfileStraight(48_in);
                 chassis->linearProfileStraight(-48_in);
               })
       .newRow()
       .build());

  intakeActions = dynamic_cast<GUI::Actions *>(
    &screen->makePage<GUI::Actions>("Intake")
       .button("In Full", [&]() { Intake::getIntake()->setNewState(IntakeState::inFull); })
       .button("Out Full", [&]() { Intake::getIntake()->setNewState(IntakeState::outFull); })
       .button("In Half", [&]() { Intake::getIntake()->setNewState(IntakeState::inHalf); })
       .button("Out Half", [&]() { Intake::getIntake()->setNewState(IntakeState::outHalf); })
       .newRow()
       .button("Move Distance",
               [&]() {
                 Intake::getIntake()->setDistance(-5.5_in);
                 Intake::getIntake()->setNewState(IntakeState::moveDistance);
               })
       .button("Hold", [&]() { Intake::getIntake()->setNewState(IntakeState::hold); })
       .button("Off", [&]() { Intake::getIntake()->setNewState(IntakeState::off); })
       .build());

  liftActions = dynamic_cast<GUI::Actions *>(
    &screen->makePage<GUI::Actions>("Lift")
       .button("Mid Tower", [&]() { Lift::getLift()->setState(LiftState::midTower); })
       .button("Low Tower", [&]() { Lift::getLift()->setState(LiftState::lowTower); })
       .button("2 Cube ", [&]() { Lift::getLift()->setState(LiftState::a2CubeStack); })
       .button("3 Cube", [&]() { Lift::getLift()->setState(LiftState::a3CubeStack); })
       .newRow()
       .button("4 Cube", [&]() { Lift::getLift()->setState(LiftState::a4CubeStack); })
       .button("Down", [&]() { Lift::getLift()->setState(LiftState::down); })
       .button("Off", [&]() { Lift::getLift()->setState(LiftState::off); })
       .build());

  tilterActions = dynamic_cast<GUI::Actions *>(
    &screen->makePage<GUI::Actions>("Tilter")
       .button("Up", [&]() { Tilter::getTilter()->setState(TilterState::up); })
       .button("liftUpBtn", [&]() { Tilter::getTilter()->setState(TilterState::liftUp); })
       .newRow()
       .button("Down", [&]() { Tilter::getTilter()->setState(TilterState::down); })
       .button("Off", [&]() { Tilter::getTilter()->setState(TilterState::off); })
       .build());
  screen->makePage<GUI::Odom>("Odom").attachOdom(chassis->odom).attachResetter([&]() {
    chassis->skidSteerModel->resetSensors();
  });

#if ROS == 1
  nh->initNode();

  nh->advertise(posL);
  nh->advertise(velL);
  nh->advertise(accL);
  nh->advertise(jrkL);
  nh->advertise(snpL);

  nh->advertise(posR);
  nh->advertise(velR);
  nh->advertise(accR);
  nh->advertise(jrkR);
  nh->advertise(snpR);

  nh->advertise(pose);
#endif

  graphing = task_create(loop, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "graphing");

  odomTask =
    task_create(odomFnc, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "odom task");

  printf("init end\n");
}

/**
 * @brief      disabled
 */
void disabled();

/**
 * @brief      competition initialize
 */
void competition_initialize();

/**
 * @brief      autonomous
 */
void autonomous() {
  printf("autonomous\n");

  auto time = pros::millis();

  printf("battery voltage = %d\n", pros::battery::get_voltage());

  chassis->skidSteerModel->resetSensors();
  chassis->stopControllers();

  if (pros::battery::get_voltage() >= 12000) {
    selector->run();
  }

  printf("end autonomous\n");
}

/**
 * @brief      opcontrol
 */
void opcontrol() {

  printf("opcontrol\n");

  std::string out("line");
  master->setText(0, 0, "opcontrol");
  master->setText(1, 1, out);
  master->setText(2, 2, "hi");

  chassis->stopControllers();
  chassis->skidSteerModel->setMaxVoltage(12000);

  while (true) {
    chassis->skidSteerModel->arcade(master->getAnalog(ControllerAnalog::rightY),
                                    master->getAnalog(ControllerAnalog::leftX));

    if (intakeUpBtn->isPressed() && intakeDownBtn->isPressed()) {
      printf("Intake Up and Intake Down Button Pressed\n");
      Intake::getIntake()->setNewState(IntakeState::off);
    } else if (intakeUpBtn->isPressed()) {
      printf("Intake Up Button Pressed\n");
      Intake::getIntake()->setNewState(IntakeState::inFull);
    } else if (intakeDownBtn->isPressed()) {
      printf("Intake Down Button Pressed\n");
      Intake::getIntake()->setNewState(IntakeState::outHalf);
    } else {
      Intake::getIntake()->setNewState(IntakeState::hold);
    }

    if (tilterUpBtn->isPressed() && tilterDownBtn->isPressed()) {
      printf("Tilter Up and Tilter Down Button Pressed\n");
      Tilter::getTilter()->setNewState(TilterState::off);
    } else if (tilterUpBtn->isPressed()) {
      printf("Tilter Up Button Pressed\n");
      if (Lift::getLift()->getState() != LiftState::down) {
        Lift::getLift()->setStateBlocking(LiftState::down);
      }
      Tilter::getTilter()->setNewState(TilterState::up);
    } else if (tilterDownBtn->isPressed()) {
      printf("Tilter Down Button Pressed\n");
      Tilter::getTilter()->setNewState(TilterState::down);
    }

    if (liftUpBtn->changedToPressed() || liftMidBtn->changedToPressed()) {
      printf("lift Up Button or Lift Mid Button Pressed\n");
      liftToggle = !liftToggle;
    }

    if (liftUpBtn->isPressed() && liftMidBtn->isPressed()) {
      printf("Lift Up and Lift Mid Button Pressed\n");
      Lift::getLift()->setNewState(LiftState::off);
    } else if (liftUpBtn->isPressed() && liftToggle) {
      printf("Lift Up Button Pressed and liftToggle\n");
      if (Tilter::getTilter()->getState() == TilterState::up) {
        Tilter::getTilter()->setStateBlocking(TilterState::liftUp);
      } else {
        Tilter::getTilter()->setNewState(TilterState::liftUp);
      }
      Lift::getLift()->setNewState(LiftState::midTower);
    } else if (liftMidBtn->isPressed() && liftToggle) {
      printf("Lift Mid Button Pressed and liftToggle\n");
      if (Tilter::getTilter()->getState() == TilterState::up) {
        Tilter::getTilter()->setStateBlocking(TilterState::liftUp);
      } else {
        Tilter::getTilter()->setNewState(TilterState::liftUp);
      }
      Lift::getLift()->setNewState(LiftState::lowTower);
    } else if (liftUpBtn->isPressed() && !liftToggle) {
      printf("Lift Up Button Pressed and not liftToggle\n");
      Lift::getLift()->setNewState(LiftState::down);
      Tilter::getTilter()->setNewState(TilterState::down);
    } else if (liftMidBtn->isPressed() && !liftToggle) {
      printf("Lift Mid Button Pressed and not liftToggle\n");
      Lift::getLift()->setNewState(LiftState::down);
      Tilter::getTilter()->setNewState(TilterState::down);
    }

    pros::delay(10);
  }
}
