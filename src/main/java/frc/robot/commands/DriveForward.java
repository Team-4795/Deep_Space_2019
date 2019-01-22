/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveForward extends Command {
  public DriveForward(double timeout) {
    requires(Robot.drivebase);
    setTimeout(timeout);
  }

  @Override
  protected void initialize() {
  }


  @Override
  protected void execute() {
    Robot.drivebase.setMotors(0.2, 0.2);
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }


  @Override
  protected void end() {
    Robot.drivebase.setMotors(0.0, 0.0);
  }

  @Override
  protected void interrupted() {
  }
}
