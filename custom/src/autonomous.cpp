#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>
#include "../include/autonomous.h"
#include "motor-control.h"

void rtAuton() { //s1
  wait(50,msec);
  driveTo(31,1250,true,12);
  lower_intake.spin(fwd,100, pct);
  turnToAngle(226,1000,true,12);
  driveTo(31,1500,true,7);
  wait(50,msec);
  driveTo(19,750,true,8);
  turnToAngle(227,1000,true,12);
  lower_intake.spin(reverse,100, pct);
  wait(1500,msec);
  lower_intake.stop(coast);
  driveTo(-31,1250,true,8);
  wait(50,msec);
  match_loader.set(true);
  driveTo(-16,2000,true,9);
  turnToAngle(90,750,true,12);
  lower_intake.spin(fwd,100, pct);
  driveTo(22,1750,true,12);
  wait(750,msec);
  driveTo(-13,750,true,8);
  match_loader.set(false);
  driveTo(-25,1250,true,8);
  wait(150,msec);
  upper_intake.spin(fwd,100,pct);
}

void leftAuton() // case 2
{
  wait(50,msec);
  driveTo(31,1250,true,12);
  match_loader.set(true);
  turnToAngle(-90,750,true,11);
  lower_intake.spin(fwd,100, pct);
  driveTo(21,1500,true,11);
  wait(100,msec);
  driveTo(-13,750,true,8);
  match_loader.set(false);
  driveTo(-25,1250,true,8);
  upper_intake.spin(fwd,100,pct);
  wait(1250,msec);
  upper_intake.stop(coast);
  driveTo(21,1125,true,9);
  lower_intake.spin(fwd,75, pct);
  turnToAngle(-225,1250,true,12);
  driveTo(31,1750,true,8);
  wait(100,msec);
  lower_intake.stop(coast);
  turnToAngle(-45,1000,true,9);
  middle_piston.set(true);
  driveTo(-21,1250,true,8);
  lower_intake.spin(fwd,100, pct);
  upper_intake.spin(fwd,75, pct);
}

void SAWP() { 
  wait(50,msec);
  driveTo(31,900,true,12);
  match_loader.set(true);
  turnToAngle(90,500,true,11);
  lower_intake.spin(fwd,100, pct);
  driveTo(21.5,800,true,12);
  wait(325,msec);
  match_loader.set(false);
  driveTo(-38,1000,true,7);
  upper_intake.spin(fwd,100,pct);
  wait(1,sec);
  upper_intake.stop(brake);
  driveTo(19,875,true,11);
  turnToAngle(225,750,true,12);
  driveTo(31,900,true,12);
  turnToAngle(180,250,true,12);
  driveTo(29,900,true,12);
  wait(50,msec);
  driveTo(20,900,true,12);
  match_loader.set(true);
  turnToAngle(135,250,true,12);
  driveTo(-23,750,7);
  middle_piston.set(true);
  upper_intake.spin(fwd,75,pct);
  wait(825,msec);
  upper_intake.stop(brake);
  middle_piston.set(false);
  driveTo(49,1500,true,11);
  turnToAngle(90,250,true,12);
  driveTo(21,625,true,8);
  wait(625,msec);
  driveTo(-38,1000,true,7);
  upper_intake.spin(fwd,75,pct);
  lower_intake.spin(fwd,100,pct);
}

void right7long() {//left
  wait(50,msec);
  driveTo(31,1250,true,12);
  lower_intake.spin(fwd,100, pct);
  turnToAngle(226,1000,true,12);
  driveTo(31,1250,true,7);
  wait(50,msec);
  driveTo(-31,1250,true,8);
  wait(50,msec);
  match_loader.set(true);
  turnToAngle(90,1500,true,9);
  driveTo(21.5,1500,true,11);
  wait(300,msec);
  lower_intake.stop(coast);
  driveTo(-13,750,true,8);
  driveTo(-27,1250,true,7);
  match_loader.set(false);
  lower_intake.spin(fwd,100, pct);
  upper_intake.spin(fwd,100,pct);
}

void left7long()
{
  wait(50,msec);
  driveTo(31,900,true,12);
  lower_intake.spin(fwd,100, pct);
  turnToAngle(-226,900,true,12);
  driveTo(31,1250,true,7);
  // turnToAngle(-318,900,true,12);
  // driveTo(27,900,true,12);
  // wait(50,msec);
  // match_loader.set(true);
  // driveTo(-27,1000,true,10);
  // wait(50,msec);
  turnToAngle(-46,800,true,12);
  driveTo(33,1375,true,10);
  wait(50,msec);
  match_loader.set(true);
  turnToAngle(-90,1000,true,9);
  driveTo(-24,1250,12);
  upper_intake.spin(fwd,100,pct);
  wait(1,sec);
  upper_intake.stop(brake);
  driveTo(37,1250,true,12);
  driveTo(-3,150,true,12);
  driveTo(4,300,true,12);
  lower_intake.stop(coast);
  driveTo(-13,750,true,8);
  driveTo(-28,1000,true,10);
  match_loader.set(false);
  lower_intake.spin(fwd,100, pct);
  upper_intake.spin(fwd,100,pct);
  wait(1,sec);
  upper_intake.stop(coast);
}

void none()
{
  lower_intake.spin(fwd,50,pct);
}

void realskills()
{
  wait(50,msec);
  driveTo(31,1000,true,12);
  match_loader.set(true);
  turnToAngle(90,500,true,11);
  lower_intake.spin(fwd,100, pct);
  driveTo(22,1000,true,12);
  driveTo(-3,250,true,12);
  driveTo(6,750,true,12);
  driveTo(-3,250,true,12);
  driveTo(6,750,true,12);
  wait(500,msec);
  match_loader.set(false);
  driveTo(-16,700,true,12);
  turnToAngle(45,500,true,12);
  driveTo(30,1250,true,12);
  turnToAngle(90,750,true,11);
  driveTo(-100,2000,true,10);
  lower_intake.stop(coast);
  turnToAngle(315,500,true,12);
  driveTo(-20,600,true,12);
  turnToAngle(270,800,true,12);
  driveTo(-27,1750,true,12);
  lower_intake.spin(fwd,100,pct);
  upper_intake.spin(fwd,100,pct);
  match_loader.set(true);
  wait(2,sec);
  upper_intake.stop(coast);
  driveTo(18,1000,true,11);
  driveTo(18,1000,true,12);
  driveTo(-3,250,true,12);
  driveTo(6,750,true,12);
  driveTo(-3,250,true,12);
  driveTo(6,750,true,12);
  driveTo(-38,1000,true,10);
  match_loader.set(false);
  upper_intake.spin(fwd,100,pct);
  wait(3,sec);
  upper_intake.stop(coast);
  driveTo(11,500,false,12);
  turnToAngle(225,500,true,12);
  driveTo(16,1000,true,12);
  turnToAngle(180,500,true,12);
  driveTo(70,3000,true,12);


}

// --------------------------------------------------------------------------------------

/*
 * armPID
 * Runs a single PID update for the arm motor to reach the specified target position.
 * - arm_target: Desired arm position (degrees).
 */
void armPID(double arm_target) {
  PID pidarm = PID(0.1, 0, 0.5); // Initialize PID controller for arm
  pidarm.setTarget(arm_target);   // Set target position
  pidarm.setIntegralMax(0);  
  pidarm.setIntegralRange(1);
  pidarm.setSmallBigErrorTolerance(1, 1);
  pidarm.setSmallBigErrorDuration(0, 0);
  pidarm.setDerivativeTolerance(100);
  pidarm.setArrive(true);
  arm_motor.spin(fwd, pidarm.update(arm_motor.position(deg)), volt); // Apply PID output to arm motor
}

/*
 * armPIDLoop
 * Continuously runs the arm PID control in a separate thread, keeping the arm at the target position.
 */
void armPIDLoop() {
  while(true) {
    wait(10, msec);
  }
}

/*
 * rushClamp
 * Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
 * Used for quickly grabbing a mobile goal at the start of autonomous.
 */
void rushClamp() {
  while(clamp_distance.objectDistance(mm) > 85) { // Wait for object to be close enough
    wait(10, msec);
  }
  claw.set(true);        // Close the claw to grab the goal
  rush_arm.set(false);   // Lower the rush arm
}

/*
 * intakeThread
 * Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
 * Used for picking up rings or other objects during autonomous.
 */
void intakeThread(){
  optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
  optical_sensor.setLightPower(100);          // Set light power to max
  while(!optical_sensor.isNearObject() && intake_distance.objectDistance(mm) > 50){
    wait(10, msec);                           // Wait until object is detected
  }
  intake_motor.stop(hold);                    // Stop intake motor and hold
}

/*
 * redGoalRush
 * 2024-2025 World Championship runner-up(1698V) autonomous routine.
 * This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
 * It uses multiple threads for simultaneous arm, clamp, and intake control.
 */
void skills() {                  // Set arm PID target to store position

  thread al = thread(armPIDLoop);                      // Start arm PID loop in a thread
  thread rc = thread(rushClamp);                       // Start clamp routine in a thread
  intake_motor.spin(fwd, 12, volt);                    // Start intake motor at full speed
  thread it = thread(intakeThread);                    // Start intake sensor thread
  rush_arm.set(true);                                  // Lower rush arm

  driveTo(33, 1100, true);                             // Drive forward to first goal
  moveToPoint(-2, 10, -1, 15000, false);               // Pull the goal back
  stopChassis(hold);                                   // Stop chassis and hold position

  rc.interrupt();                                      // Stop clamp thread (goal should be clamped)
  rush_arm.set(true);                                  // Lower rush arm again (ensure down)
  claw.set(false);                                     // Open claw to release goal
  wait(100, msec);                                     // Brief pause

  correct_angle = normalizeTarget(-20);                // Adjust target heading for next maneuver
  driveTo(3, 800, true, 8);                            // Drive forward slightly
  driveTo(-5, 1000, true);                             // Back up

  rush_arm.set(false);                                 // Raise rush arm
  wait(200, msec);                                     // Wait for arm to raise

  turnToAngle(-90, 800, false);                        // Turn to face the goal backwards
  moveToPoint(0, 26, -1, 2000, false, 6);              // Move backwards into the goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  it.interrupt();                                      // Stop intake thread (ring should be collected)
  intake_motor.spin(fwd, 12, volt);                    // Restart intake

  moveToPoint(1, 7, 1, 2000, true);                    // Move near corner to drop goal
  turnToAngle(-90, 350, true);                         // Turn to drop goal
  mogo_mech.set(false);                                // Release mobile goal clamp
  driveChassis(-4, 4);                                 // Turn a bit to align with next target
  wait(300, msec);                                     // Wait for spin

  intake_motor.spin(fwd, -12, volt);                   // Reverse intake to push disc in front away
  moveToPoint(-13, -4, 1, 1500, false, 10);            // Move forward to push disc out of the way
  turnToAngle(180, 800, false);                        // Turn to clamp goal
  intake_motor.spin(fwd, 0, volt);                     // Stop intake

  moveToPoint(-31, 26, -1, 2000, false, 6);            // Move backwards into the next goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  turnToAngle(145, 300, true);                         // Turn to face corner
  moveToPoint(-4, -3, 1, 2000, false);                 // Move to corner
  intake_motor.spin(fwd, 12, volt);                    // Start intake

  correct_angle = normalizeTarget(135);                // Update heading for next maneuver
  driveTo(1000, 1500, false, 4);                       // Drive forward infinitely until timeout
  driveTo(-13, 2000, true, 6);                         // Back up
  driveTo(10, 2500, true, 3);                          // Drive forward to intake second corner ring

  wait(200, msec);                                     // Brief wait for intake

  moveToPoint(-11, 6, -1, 2000, false, 10);            // Move backward out of the corner
  turnToAngle(45, 400, true);                          // Turn to align for wallstake

  al.interrupt();                                      // Stop arm PID thread
  thread al2 = thread(armPIDLoop);                     // Start new arm PID thread

  moveToPoint(12, 34, 1, 1700, true, 8);               // Move forward to final wallstake scoring position

  al2.interrupt();                                     // Stop arm PID thread
  arm_motor.spin(fwd, 1, volt);                        // Spin arm forward slightly

  turnToAngle(40, 200);                                // Final turn for alignment
  driveChassis(1, 1);                                  // Slow drive forward
}