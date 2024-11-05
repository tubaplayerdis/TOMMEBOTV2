#include <cmath>
#include <algorithm>
#include "vex.h"
#include "Bot.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


competition Competition;

/*
    OVERVIEW
    The values are tolerances for the blue and red rings. Colors can vary significantly due to brightness so the ranges are broad. Dosent matter tho cause only red and blue rings
    THE RED HAS EXCLUSION VALUES. What this means is that the range given is excluded instead of included. This is because the red and violet look similar and the color sensor dosent know the difference.
    Calculated as such:
        double GV = Bot::ColorSensor.hue();
        if(GV < BLUE_HUE_HIGH && GV > BLUE_HEU_LOW) {
            //Blue Ring
        } else if( GV < RED_HUE_EXCLUSION_LOW && GV > RED_HUE_EXCLUSION_HIGH) {
            //Red Ring
        }
*/
#define BLUE_HEU_LOW 85
#define BLUE_HUE_HIGH 271

#define RED_HUE_EXCLUSION_LOW 51
#define RED_HUE_EXCLUSION_HIGH 341


//For Throwing:
#define REVERSESECONDS 0.25//Seconds to move the conveyor in reverse at given velocity below.
#define REVERSEVELOCITY 600//(rpm because there is no percent option and 100% is 600 rpm)
#define FORWARDSECONDS 0.5//Seconds to keep moving the conveyor forward after going in reverse is done.




// Constants for Tracking Wheels
constexpr double tracking_wheel_radius = 2.0;                  // Radius in inches (assuming 1:1 gear ratio)
constexpr double tracking_wheel_circumference = 2 * M_PI * tracking_wheel_radius;

// Odometry Variables
double x = 0.0;   // X position in inches (forward/backward)
double y = 0.0;   // Y position in inches (left/right)
double heading = 0.0;

// Variable for general use
float myVariable;

// Wall Stakes Positions (in mm)
const double tallWallStake1X = -30.0; // X position of the first wall stake
const double tallWallStake1Y = 0.0;     // Y position of the first wall stake
const double tallWallStake2X = 1750.0;  // X position of the second wall stake
const double tallWallStake2Y = 0.0;     // Y position of the second wall stake
constexpr double approach_distance = 152.4; // Distance to stop in front of wall stake, 6 inches converted to mm

// Calculate distance between robot and a wall stake
double calculateDistanceToStake(double stakeX, double stakeY) {
    return sqrt(pow(stakeX - x, 2) + pow(stakeY - y, 2));
}

#define MIN(x,y) ((x) < (y) ? (x) : (y)) 

// Drive to a specific position
void driveToPosition(double targetX, double targetY) {
    double distance_to_target;
    double angle_to_target;
    bool dostop = false;
    while (true) {
        if(!Bot::Controller.ButtonX.pressing()) {
            if(dostop) {
                Bot::LeftMotors.stop();
                Bot::RightMotors.stop();
                dostop = false;
            }
            continue;
        }
        distance_to_target = calculateDistanceToStake(targetX, targetY);
        angle_to_target = atan2(targetY - y, targetX - x) * (180 / M_PI) - heading;

        Bot::Brain.Screen.printAt(0, 50, "Distance to Target: %f", distance_to_target);

        if (distance_to_target < 25.4) { // Tolerance in mm (1 inch)
            Bot::LeftMotors.stop();
            Bot::RightMotors.stop();
            Bot::Brain.Screen.printAt(130, 130, "Drove To WallStake!");
            break;
        }

        double forwardSpeed = MIN(distance_to_target * 0.2, 50); // Scale speed, limit to 50
        double turnSpeed = angle_to_target * 0.5;

        Bot::LeftMotors.spin(forward, forwardSpeed + turnSpeed, pct);
        Bot::RightMotors.spin(forward, forwardSpeed - turnSpeed, pct);

        Bot::Brain.Screen.printAt(130, 130, "Driving To WallStake");
        dostop = true;
        vex::this_thread::sleep_for(20);
    }
}

// Rotate to specified angle using inertial sensor
void rotateToAngle(double targetAngle) {
    bool dostop = false;
    while (fabs(heading - targetAngle) > 1.0) {
        if(!Bot::Controller.ButtonX.pressing()) {
            if(dostop) {
                Bot::LeftMotors.stop();
                Bot::RightMotors.stop();
                dostop = false;
            }
            continue;
        }
        double error = targetAngle - heading;
        double turnSpeed = error * 0.5;

        Bot::LeftMotors.spin(forward, turnSpeed, pct);
        Bot::RightMotors.spin(reverse, turnSpeed, pct);

        dostop = true;
        vex::this_thread::sleep_for(20);
    }
    Bot::LeftMotors.stop();
    Bot::RightMotors.stop();
}

// Drive to the nearest wall stake and align perpendicular
int driveToNearestWallStake() {
    while (true)
    {
        double distanceToStake1 = calculateDistanceToStake(tallWallStake1X, tallWallStake1Y);
        double distanceToStake2 = calculateDistanceToStake(tallWallStake2X, tallWallStake2Y);

        Bot::Brain.Screen.printAt(0, 70, "Calculated Distance Of Stakes");

        double targetX, targetY, perpendicularAngle;

        if (distanceToStake1 < distanceToStake2) {
            targetX = tallWallStake1X + approach_distance;
            targetY = tallWallStake1Y;
            perpendicularAngle = 90.0;  // Align perpendicular to stake at (-1750, 0)
        } else {
            targetX = tallWallStake2X - approach_distance;
            targetY = tallWallStake2Y;
            perpendicularAngle = 270.0; // Align perpendicular to stake at (1750, 0)
        }

        // Drive to the target position
        driveToPosition(targetX, targetY);

        // Rotate to face perpendicular
        rotateToAngle(perpendicularAngle);   
    }
    return 0;
}


// Function to Update Odometry Using Encoders and Bot:: Sensor
void updateOdometry() {
    heading = Bot::Inertial.heading(degrees);

    double forward_distance = (Bot::Rotation10.position(degrees) / 360) * tracking_wheel_circumference; // Forward movement
    Bot::Brain.Screen.printAt(100,100,"Forward Dis %f", forward_distance);
    double lateral_distance = (Bot::Rotation11.position(degrees) / 360) * tracking_wheel_circumference; // Lateral movement

    Bot::Rotation10.resetPosition();
    Bot::Rotation11.resetPosition();

    double deltaX = forward_distance * cos(heading * M_PI / 180);
    double deltaY = lateral_distance * cos(heading * M_PI / 180);

    x += deltaX;
    y += deltaY;
}

// Odometry thread function
int odometry() {
    while (true) {
        updateOdometry();
        vex::this_thread::sleep_for(10);  // Update rate in milliseconds
    }
    return 0;
}

// Display thread function to continuously print X and Y on the Bot:: screen
int displayCoordinates() {
    while (true) {
        Bot::Brain.Screen.clearScreen(); // Clear the screen for updated information
        Bot::Brain.Screen.setCursor(1, 1);
        Bot::Brain.Screen.print("X = %.2f, Y = %.2f", x, y); // Print current coordinates
        vex::this_thread::sleep_for(20); // Update rate for display (in milliseconds)
    }
    return 0;
}

// "when started" hat block
int whenStarted1() {
    Bot::Drivetrain.setDriveVelocity(90.0, percent);
    Bot::Drivetrain.setTurnVelocity(100.0, percent);
    return 0;
}

// "when autonomous" hat block
int onauton_autonomous_0() {
    // Start the odometry thread
    vex::thread odometry_thread(odometry);

    Bot::GoalClamp.set(true);
    Bot::LeftMotors.setVelocity(60.0, percent);
    Bot::RightMotors.setVelocity(60.0, percent);
    Bot::Drivetrain.driveFor(forward, 1200.0, mm, true);
    Bot::GoalClamp.set(false);
    vex::this_thread::sleep_for(500); // Wait for half a second
    Bot::Drivetrain.turnFor(right, 45.0, degrees, true);
    Bot::Drivetrain.driveFor(forward, 700.0, mm, true);
    
    return 0;
}

// "when driver control" hat block
int ondriver_drivercontrol_0() {
    bool RemoteControlCodeEnabled = true;
    bool DrivetrainLNeedsToBeStopped = true;
    bool DrivetrainRNeedsToBeStopped = true;
    vex::task gotosus(driveToNearestWallStake);
    while(true) {

      if(RemoteControlCodeEnabled) {
        
        // calculate the drivetrain motor velocities from the controller joystick axies
        // left = Axis3 + Axis1
        // right = Axis3 - Axis1
        int drivetrainLeftSideSpeed = Bot::Controller.Axis3.position() + Bot::Controller.Axis1.position();
        int drivetrainRightSideSpeed = Bot::Controller.Axis3.position() - Bot::Controller.Axis1.position();
        
        // check if the value is inside of the deadband range
        if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
          // check if the left motor has already been stopped
          if (DrivetrainLNeedsToBeStopped) {
            // stop the left drive motor
            Bot::LeftMotors.stop();
            // tell the code that the left motor has been stopped
            DrivetrainLNeedsToBeStopped = false;
          }
        } else {
          // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
          DrivetrainLNeedsToBeStopped = true;
        }
        // check if the value is inside of the deadband range
        if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
          // check if the right motor has already been stopped
          if (DrivetrainRNeedsToBeStopped) {
            // stop the right drive motor
            Bot::RightMotors.stop();
            // tell the code that the right motor has been stopped
            DrivetrainRNeedsToBeStopped = false;
          }
        } else {
          // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
          DrivetrainRNeedsToBeStopped = true;
        }
        
        // only tell the left drive motor to spin if the values are not in the deadband range
        if (DrivetrainLNeedsToBeStopped) {
          Bot::LeftMotors.setVelocity(drivetrainLeftSideSpeed, vex::percent);
          Bot::LeftMotors.spin(vex::reverse);
        }
        // only tell the right drive motor to spin if the values are not in the deadband range
        if (DrivetrainRNeedsToBeStopped) {
          Bot::RightMotors.setVelocity(drivetrainRightSideSpeed, vex::percent);
          Bot::RightMotors.spin(vex::reverse);
        }
      }
      // wait before repeating the process
      vex::wait(20, vex::msec);
    }
    return 0;
}

// Driver Control function
void usercontrol() {
    // Start the odometry thread if it hasn't already been started
    vex::thread odometry_thread(odometry);
}

// Event handlers for buttons
void onevent_controllerButtonL1_pressed_0() {
    Bot::Lift.setVelocity(100.0, percent);
    Bot::Lift.setMaxTorque(100.0, percent);
    Bot::Lift.spin(forward);
    waitUntil((!Bot::Controller.ButtonL1.pressing()));
    Bot::Lift.setStopping(hold);
    Bot::Lift.stop();
}

void onevent_controllerButtonR1_pressed_0() {
    Bot::Lift.setVelocity(100.0, percent);
    Bot::Lift.setMaxTorque(100.0, percent);
    Bot::Lift.spin(reverse);
    waitUntil((!Bot::Controller.ButtonR1.pressing()));
    Bot::Lift.setStopping(hold);
    Bot::Lift.stop();
}

void onevent_controllerButtonR2_pressed_0() {
    Bot::Conveyor.setVelocity(80.0, percent);
    Bot::Conveyor.setMaxTorque(100.0, percent);
    Bot::Conveyor.spin(forward);
    waitUntil((!Bot::Controller.ButtonR2.pressing()));
    Bot::Conveyor.setStopping(brake);
    Bot::Conveyor.stop();
}

void onevent_controllerButtonL2_pressed_0() {
    Bot::Conveyor.setVelocity(80.0, percent);
    Bot::Conveyor.setMaxTorque(100.0, percent);
    Bot::Conveyor.spin(reverse);
    waitUntil((!Bot::Controller.ButtonL2.pressing()));
    Bot::Conveyor.setStopping(brake);
    Bot::Conveyor.stop();
}

void onevent_controllerButtonR2_pressed_1() {
    Bot::Intake.setVelocity(100.0, percent);
    Bot::Intake.setMaxTorque(100.0, percent);
    Bot::Intake.spin(forward);
    waitUntil((!Bot::Controller.ButtonR2.pressing()));
    Bot::Intake.setStopping(brake);
    Bot::Intake.stop();
}

void onevent_controllerButtonL2_pressed_1() {
    Bot::Intake.setVelocity(100.0, percent);
    Bot::Intake.setMaxTorque(100.0, percent);
    Bot::Intake.spin(reverse);
    waitUntil((!Bot::Controller.ButtonL2.pressing()));
    Bot::Intake.setStopping(brake);
    Bot::Intake.stop();
}

void onevent_controllerButtonDown_pressed_0() {
    Bot::GoalClamp.set(false);
}

void onevent_controllerButtonUp_pressed_0() {
    Bot::GoalClamp.set(true);
}

void VEXcode_driver_task() {
    vex::task drive0(ondriver_drivercontrol_0);
    while(Competition.isDriverControl() && Competition.isEnabled()) {this_thread::sleep_for(10);}
    drive0.stop();
    return;
}

void VEXcode_auton_task() {
    vex::task auto0(onauton_autonomous_0);
    while(Competition.isAutonomous() && Competition.isEnabled()) {this_thread::sleep_for(10);}
    auto0.stop();
    return;
}

void onevent_controllerButtonB_pressed_0() {
    switch (Bot::Aliance)
    {
        case aliance::Nuetral:
            Bot::Aliance = aliance::Blue;
            Bot::Controller.Screen.setCursor(3,1);
            Bot::Controller.Screen.print("BLUE ALIANCE");
            break;

        case aliance::Blue:
            Bot::Aliance = aliance::Red;
            Bot::Controller.Screen.setCursor(3,1);
            Bot::Controller.Screen.print("RED ALIANCE ");
            break;

        case aliance::Red:
            Bot::Aliance = aliance::Nuetral;
            Bot::Controller.Screen.setCursor(3,1);
            Bot::Controller.Screen.print("NO ALIANCE  ");
            break;

        default:
            Bot::Aliance = aliance::Nuetral;
            Bot::Controller.Screen.setCursor(3,1);
            Bot::Controller.Screen.print("NO ALIANCE ^!");
            break;
    }
}

int visionTask() {
    while(true) 
    {
        //optical::rgbc value = Bot::ColorSensor.getRgb(false);
        //Bot::Brain.Screen.printAt(0,150, "R: %f, G: %f, B: %f", value.red, value.green, value.blue);

        double brightness = Bot::ColorSensor.brightness(false);
        double hue = Bot::ColorSensor.hue();
        Bot::Brain.Screen.printAt(0,170, "hue: %f brightness: %f", hue, brightness);


        /*
        Known Values:
        66, 135, 245 Blue Ring
        240, 29, 36 Red Ring
        Should Incorperate tolerances for red/blue rings
        */

        double GV = Bot::ColorSensor.hue();
        if(GV < BLUE_HUE_HIGH && GV > BLUE_HEU_LOW) {
            //Blue Ring
            Bot::Brain.Screen.printAt(0,190, "Blue Ring Detected!");
            if(Bot::Aliance == aliance::Nuetral) continue;
            if(!Bot::Controller.ButtonA.pressing()) continue;
            //Blue Ring Detected
            if(Bot::Aliance != aliance::Blue) {
                //bool spinFor(vex::directionType dir, double time, vex::timeUnits units, double velocity, vex::velocityUnits units_v)
                Bot::Conveyor.spinFor(vex::directionType::rev, REVERSESECONDS, vex::timeUnits::sec, REVERSEVELOCITY, vex::velocityUnits::rpm);
                Bot::Conveyor.setVelocity(100, vex::percent);
                Bot::Conveyor.spinFor(FORWARDSECONDS, vex::seconds);

                if(Bot::Controller.ButtonL2.pressing()) {
                    Bot::Conveyor.setVelocity(80.0, percent);
                    Bot::Conveyor.setMaxTorque(100.0, percent);
                    Bot::Conveyor.spin(reverse);
                } else if(Bot::Controller.ButtonR2.pressing()) {
                    Bot::Conveyor.setVelocity(80.0, percent);
                    Bot::Conveyor.setMaxTorque(100.0, percent);
                    Bot::Conveyor.spin(vex::forward);
                }
                //Bot::Conveyor.stop();
            }
        } else if( GV < RED_HUE_EXCLUSION_LOW && GV > RED_HUE_EXCLUSION_HIGH) {
            //Red Ring
            Bot::Brain.Screen.printAt(0,190, "Red Ring Detected!");
            if(Bot::Aliance == aliance::Nuetral) continue;
            if(!Bot::Controller.ButtonA.pressing()) continue;
            //Red Ring Detected
            if(Bot::Aliance != aliance::Red) {
                Bot::Conveyor.spinFor(vex::directionType::rev, 0.25, vex::timeUnits::sec, 600, vex::velocityUnits::rpm);
                Bot::Conveyor.setVelocity(75, vex::percent);
                Bot::Conveyor.spinFor(FORWARDSECONDS, vex::seconds);

                if(Bot::Controller.ButtonL2.pressing()) {
                    Bot::Conveyor.setVelocity(80.0, percent);
                    Bot::Conveyor.setMaxTorque(100.0, percent);
                    Bot::Conveyor.spin(reverse);
                } else if(Bot::Controller.ButtonR2.pressing()) {
                    Bot::Conveyor.setVelocity(80.0, percent);
                    Bot::Conveyor.setMaxTorque(100.0, percent);
                    Bot::Conveyor.spin(vex::forward);
                }
                //Bot::Conveyor.stop();
            }
        }
        
    }
    return 0;
}


int main() {
    vex::competition::bStopTasksBetweenModes = false;

    vex::thread odometry_thread(odometry);
    vex::task drive0(ondriver_drivercontrol_0);
    vex::task thrower(visionTask);
    Bot::Controller.Screen.setCursor(3,1);
    Bot::Controller.Screen.print("NO ALIANCE  ");
    // Initializing Robot Configuration. DO NOT REMOVE!
    //vexcodeInit();

    // Start the display thread for coordinates
    vex::thread display_thread(displayCoordinates);

    // Register autonomous and driver control functions
    Competition.autonomous(VEXcode_auton_task);
    //Competition.drivercontrol(VEXcode_driver_task);

    // Register event handlers
    Bot::Controller.ButtonL1.pressed(onevent_controllerButtonL1_pressed_0);
    Bot::Controller.ButtonR1.pressed(onevent_controllerButtonR1_pressed_0);
    Bot::Controller.ButtonR2.pressed(onevent_controllerButtonR2_pressed_0);
    Bot::Controller.ButtonR2.pressed(onevent_controllerButtonR2_pressed_1);
    Bot::Controller.ButtonL2.pressed(onevent_controllerButtonL2_pressed_0);
    Bot::Controller.ButtonL2.pressed(onevent_controllerButtonL2_pressed_1);
    Bot::Controller.ButtonDown.pressed(onevent_controllerButtonDown_pressed_0);
    Bot::Controller.ButtonUp.pressed(onevent_controllerButtonUp_pressed_0);
    Bot::Controller.ButtonB.pressed(onevent_controllerButtonB_pressed_0);

    wait(15, msec); // Allow time for initialization

    // Set default print color to black
    printf("\033[30m");

    // Wait for rotation sensor to fully initialize
    wait(30, msec);

    whenStarted1();

    // Start the odometry thread independently for coordinate tracking during all modes
}
