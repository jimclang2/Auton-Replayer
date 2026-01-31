#include "autonomous.h"
#include "robot_config.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

void skills_auton() {
    chassis.setPose(0, 0, 270);
    chassis.moveToPoint(-31.75, 2, 2000);
    chassis.turnToHeading(180, 1500);
    pros::delay(300);
    Unloader.set_value(true);
    chassis.moveToPoint(-31.75, -15, 2000, {.maxSpeed = 100});
    Intake.move(-127);
    chassis.moveToPoint(-32, 5, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(500);
    Unloader.set_value(false);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-51, 5, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.turnToHeading(0, 2000);
    chassis.moveToPoint(-44, 85, 2000);
    
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(-50, 85, 1000, {.forwards = false, .minSpeed = 60});
    
    chassis.moveToPoint(-29.75, 90, 2000);
    chassis.turnToHeading(0, 2000);
    chassis.moveToPoint(-29.75, 70, 2000, {.forwards = false, .maxSpeed = 50});
    pros::delay(800);
    Outtake.move(127);
    Intake.move(-127);
    pros::delay(1500);
    Outtake.move(0);
    pros::delay(1000);
    chassis.turnToHeading(0, 1000);
    
    Unloader.set_value(true);
    chassis.moveToPoint(-30, 110, 2000, {.maxSpeed = 70, .minSpeed = 60});
    pros::delay(2500);
    chassis.moveToPoint(-30, 70, 2000, {.forwards = false, .maxSpeed = 75});
    pros::delay(1700);
    Outtake.move(127);
    Intake.move(-127);
    pros::delay(1000);
    Unloader.set_value(false);
    pros::delay(1500);
    Outtake.move(0);
    Intake.move(0);
    chassis.moveToPose(-20, 60, 180, 5000, {.lead = .3, .maxSpeed = 70});
    chassis.moveToPose(-10, 0, 0, 5000, {.forwards = false});
    chassis.moveToPoint(-10, -15, 1500, {.forwards = false});
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(-20, -15, 1500, {.forwards = false});
    Intake.move(-127);
    Outtake.move(127);
    chassis.moveToPoint(10, -15, 2000, {.minSpeed = 120});
}

void leftAuton() {
    Descore.set_value(true);
    chassis.setPose(0, 0, 270);
    
    chassis.moveToPoint(-32, 10, 2000);
    Intake.move(-127);
    chassis.turnToPoint(-32, -10, 1500);
    pros::delay(1000);
    Descore.set_value(false);
    Unloader.set_value(true);
    chassis.moveToPoint(-32, -15, 3000);
    chassis.moveToPoint(-32, 30, 2000, {.forwards = false, .maxSpeed = 80}, false);
    
    Outtake.move(127);
}

void rightAuton() {
    Descore.set_value(true);
    chassis.setPose(0, 0, 90);
    
    chassis.moveToPoint(32, 10, 2000);
    Intake.move(-127);
    chassis.turnToPoint(32, -10, 1500);
    pros::delay(1000);
    Unloader.set_value(true);
    Descore.set_value(false);
    chassis.moveToPoint(32, -15, 3000, {.maxSpeed = 127}, false);
    chassis.moveToPoint(32, 30, 3000, {.forwards = false, .maxSpeed = 80}, false);
    
    Outtake.move(127);
}

void rightAutonDescore() {
    Descore.set_value(true);
    chassis.setPose(0, 0, 90);
    
    chassis.moveToPoint(32, 10, 2000);
    Intake.move(-127);
    chassis.turnToPoint(32, -10, 1500);
    pros::delay(1000);
    Unloader.set_value(true);
    Descore.set_value(false);
    chassis.moveToPoint(32, -15, 3000, {.maxSpeed = 127}, false);
    chassis.moveToPoint(32, 30, 2000, {.forwards = false, .maxSpeed = 80}, false);
    Outtake.move(127);
    pros::delay(2000);
    chassis.moveToPoint(32, 5, 2000);
}

void yashasjerrytest() {
    using namespace lemlib;
    chassis.setPose(-130.167, 44.582, 18.434);
    chassis.turnToPoint(-105.646, 118.148, 1500);
    chassis.moveToPoint(-105.646, 118.148, 2000);
    Intake.move(-127);
    chassis.turnToPoint(-153.728, 118.147, 1500);
    Unloader.set_value(true);
    chassis.moveToPoint(-153.728, 118.147, 2000); //unload #1
    pros::delay(1000);
    chassis.moveToPoint(-122.474, 118.148, 2000,{.forwards = false});
    Unloader.set_value(false);
    Intake.move(0);
    chassis.turnToPoint(-122.474, -100, 1500);
    chassis.moveToPoint(-122.474, 159.017, 2000,{.forwards = false}); //allign
    chassis.moveToPoint(-122.474, 153.246, 2000); // forward a bit
    chassis.turnToPoint(101.107, 153.246, 1500);
    chassis.moveToPoint(101.107, 153.246, 2000);
    chassis.turnToPoint(101.107, 116.704, 1500);
    chassis.moveToPoint(101.107, 116.704, 2000); // move to goal plane
    chassis.turnToPoint(151.112, 116.704, 1500); // turn to face goal
    chassis.moveToPoint(81.874, 116.704, 2000,{.forwards = false, .maxSpeed = 80}); // back up to score
    Outtake.move(127);
    Intake.move(-127);
    Unloader.set_value(true);
    pros::delay(2000);
    Outtake.move(0);
    chassis.moveToPoint(151.112, 116.704, 1500); //unload #2
    pros::delay(1000);
    chassis.moveToPoint(81.874, 116.704, 2000,{.forwards = false}); // back up to score
    Outtake.move(127);
    Intake.move(-127);
    pros::delay(2000);
    Outtake.move(0);
    Intake.move(0);
    chassis.moveToPoint(106.396, 116.704, 2000); // move forward a bit
    chassis.turnToPoint(106.396, 130, 1500); // turn up
    chassis.moveToPoint(106.396, -155.439, 7000,{.forwards = false, .maxSpeed = 80}); // go backwards to allign next to goal 3
    chassis.moveToPoint(106.396, -121.301, 2000); // forward to goal plane
    chassis.turnToPoint(153.997, -121.301, 1500);
    Unloader.set_value(true);
    Intake.move(-127);
    chassis.moveToPoint(153.997, -121.301, 2000); // unload #3
    pros::delay(1000);
    chassis.moveToPoint(128.994, -121.301, 2000,{.forwards = false}); // back up a bit
    Intake.move(0);
    Unloader.set_value(false);
    chassis.turnToPoint(128.994, 100, 1500); // turn up
    chassis.moveToPoint(128.994, -155.439, 7000,{.forwards = false, .maxSpeed = 80}); // back up to allign
    chassis.moveToPoint(128.994, -151.593, 2000);// forward a bit
    chassis.turnToPoint(-106.126, -151.593, 1500);
    chassis.moveToPoint(-106.126, -151.593, 2000);
    chassis.turnToPoint(-106.126, -120.339, 1500);
    chassis.moveToPoint(-106.126, -120.339, 2000);
    chassis.turnToPoint(-100, -120.339, 1500); //turn to goal
    chassis.moveToPoint(-81.605, -120.339, 2000,{.forwards = false, .maxSpeed = 80}); //score
    Outtake.move(127);
    Intake.move(-127);
    Unloader.set_value(true);
    pros::delay(2000);
    Outtake.move(0);
    chassis.moveToPoint(-152.285, -120.339, 2000); // unload
    pros::delay(1000);
    chassis.moveToPoint(-81.605, -120.339, 2000,{.forwards = false, .maxSpeed = 80}); //score
    Outtake.move(127);
    Unloader.set_value(false);
    pros::delay(2000);
    chassis.moveToPoint(-117.185, -120.339, 2000);
    chassis.turnToPoint(-117.185, -80.912, 1500);
    chassis.moveToPoint(-117.185, -80.912, 2000);
    chassis.turnToPoint(-154.689, -80.912, 1500);
    chassis.moveToPoint(-154.689, -80.912, 2000);
    chassis.turnToPoint(-169.114, 1.789, 1500);
    chassis.moveToPoint(-169.114, 1.789, 2000); // park

}

void yashas_skills_auton() {
    chassis.setPose(0, 0, 270);
    chassis.moveToPoint(-31.75, 2,2000);
    chassis.turnToHeading(180, 1500,{},false);
    Unloader.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(-31.75, -15, 2000, {.maxSpeed = 100} );
    Intake.move(-127);
    chassis.moveToPoint( -32,  5, 2000, {.forwards = false, .maxSpeed = 80},false);
    Unloader.set_value(false);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint( -51,  5, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.turnToHeading(0, 2000);
    chassis.moveToPoint(-44, 85, 2000);
//_____
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(-50, 85, 1000, {.forwards = false, .minSpeed = 60});
    chassis.moveToPoint(-29.75, 90, 2000);
    chassis.turnToHeading(0, 2000);
    chassis.moveToPoint(-29.75, 70, 2000,{.forwards = false, .maxSpeed = 50},false);
    Unloader.set_value(true);
    Outtake.move(127);
    Intake.move(-127);
    pros::delay(1500);
    Outtake.move(0);
    pros::delay(1000);
    left_motors.move(30); // new
    right_motors.move(30);
    pros::delay(300);
    left_motors.move(0);
    right_motors.move(0); // 
    chassis.turnToHeading(0, 1000,{},false);
    chassis.moveToPoint(-30, 110, 2000, {.maxSpeed = 70, .minSpeed = 60},false);
    pros::delay(2500);
    chassis.moveToPoint(-30, 70, 2000,{.forwards = false, .maxSpeed = 75},false);
    pros::delay(1700);
    Outtake.move(127);
    Intake.move(-127);
    pros::delay(1000);
    Unloader.set_value(false);
    pros::delay(1500);
    Outtake.move(0);
    Intake.move(0);
    chassis.moveToPoint(-30, 90, 1000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-50, 90, 1000,{.forwards=false});
    chassis.moveToPoint(-30, 90, 1000);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(130,90,5000,{.forwards=false,.maxSpeed=80});
    // new
    chassis.moveToPoint(110,90,2000,{},false);
    chassis.turnToHeading(0, 1000);
    Unloader.set_value(true);
    Intake.move(-127);
    pros::delay(300);
    chassis.moveToPoint(110,100,2000,{.maxSpeed = 100});
    chassis.moveToPoint(100, 100, 2000,{.forwards=false},false);
    Unloader.set_value(false);
    chassis.turnToHeading(270,1000);
    chassis.moveToPoint(115, 110, 2000,{.forwards=false});
    chassis.turnToHeading(180,2000);
    chassis.moveToPoint(108,30,2000);
    // end new code


    /*
    chassis.moveToPose(-20, 60, 180, 5000, {.lead = .3, .maxSpeed = 70});
    chassis.moveToPose(-10, 0, 0, 5000, {.forwards = false});
    chassis.moveToPoint(-10, -15, 1500, {.forwards = false});
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(-20, -15, 1500, {.forwards = false});
    Intake.move(-127);
    Outtake.move(127);
    chassis.moveToPoint(10, -15, 2000, {.minSpeed = 120});
    */
}