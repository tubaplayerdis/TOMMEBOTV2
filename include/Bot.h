#include "vex.h"

enum aliance {
    Nuetral = 0,
    Blue = 1,
    Red = 2
};

class Bot {
    public:
        //Brian + Controllers
        static vex::brain Brain;
        static vex::controller Controller;

        //Motors
        static vex::motor LeftFront;
        static vex::motor LeftRear;
        static vex::motor RightFront;
        static vex::motor RightRear;

        static vex::motor_group LeftMotors;
        static vex::motor_group RightMotors;

        //Drivetrian and etc
        static vex::drivetrain Drivetrain;
        //static vex::motor ArmMotor;
        static vex::motor ClawMotor;
        static vex::motor LiftMotor0;
        static vex::motor LiftMotor1;
        static vex::motor Conveyor;
        static vex::motor Intake;
        static vex::motor_group Lift;

        //Gyros and stuff
        static vex::inertial Inertial;
        static vex::rotation Rotation10; //Forward
        static vex::rotation Rotation11; //Lateral
        static vex::optical ColorSensor;

        //Pneumatics
        static vex::digital_out GoalClamp;

        //Info
        static aliance Aliance;
};