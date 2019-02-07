/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivebase;

public class TurnToAngle extends Command {

  private final double angle;
  boolean isFinished = false;
  boolean inErrorZone = false;
  int count;

  public TurnToAngle(double angle, double timeout) {
    requires(Robot.drivebase);
    setTimeout(timeout);
    this.angle = angle;
  }

  @Override
  protected void initialize() {
    Robot.drivebase.TurnToAngle(angle);
  }

  @Override
  protected void execute() {
    // check how close we are to the target angle, if we are within the tolerance
    // for 10 roboRio
    // ticks then end the command
    double error = Robot.drivebase.turnController.getError();
    inErrorZone = Math.abs(error) < 3 ? true : false;

    if (inErrorZone) {
      count++;
      if (count >= 6) {
        isFinished = true;
      } else {
        isFinished = false;
      }
    } else {
      count = 0;
    }
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut() || isFinished;
  }

  @Override
  protected void end() {
    Robot.drivebase.setMotors(0.0, 0.0);
  }

  @Override
  protected void interrupted() {
  }
}
