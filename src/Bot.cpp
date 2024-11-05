#include "Bot.h"
#include "vex.h"

vex::brain Bot::Brain = vex::brain();
vex::controller Bot::Controller = vex::controller();

vex::motor Bot::LeftFront = vex::motor(vex::PORT1, vex::ratio18_1, false);
vex::motor Bot::LeftRear = vex::motor(vex::PORT2, vex::ratio18_1, false);
vex::motor Bot::RightFront = vex::motor(vex::PORT3, vex::ratio18_1, true);
vex::motor Bot::RightRear = vex::motor(vex::PORT4, vex::ratio18_1, true);

vex::motor_group Bot::LeftMotors = vex::motor_group(Bot::LeftFront, Bot::LeftRear);
vex::motor_group Bot::RightMotors = vex::motor_group(Bot::RightFront, Bot::RightRear);

vex::drivetrain Bot::Drivetrain = vex::drivetrain(LeftMotors, RightMotors, 4, 14.75, 12, vex::inches, 1.6);
//vex::motor Bot::ArmMotor = vex::motor(vex::PORT3, vex::ratio18_1, false);
vex::motor Bot::ClawMotor = vex::motor(vex::PORT8, vex::ratio18_1, false);
vex::motor Bot::LiftMotor0 = vex::motor(vex::PORT5, vex::ratio36_1, false);
vex::motor Bot::LiftMotor1 = vex::motor(vex::PORT6, vex::ratio36_1, true);
vex::motor Bot::Conveyor = vex::motor(vex::PORT7, vex::ratio6_1, false);
vex::motor Bot::Intake = vex::motor(vex::PORT8, vex::ratio18_1, false);
vex::motor_group Bot::Lift = vex::motor_group(Bot::LiftMotor0, Bot::LiftMotor1);

vex::inertial Bot::Inertial = vex::inertial(vex::PORT13);
vex::rotation Bot::Rotation10 = vex::rotation(vex::PORT10);
vex::rotation Bot::Rotation11 = vex::rotation(vex::PORT11);
vex::optical Bot::ColorSensor = vex::optical(vex::PORT9);

vex::digital_out Bot::GoalClamp = vex::digital_out(Bot::Brain.ThreeWirePort.A);

aliance Bot::Aliance = aliance::Nuetral;
