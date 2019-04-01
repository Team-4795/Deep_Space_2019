/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ColorSensor;
import frc.robot.Robot;
import frc.robot.ColorSensor.ColorData;

public class DriveTillLine extends Command {

  ColorSensor csLeft = ColorSensor.getInstanceMXP();
  ColorSensor csRight = ColorSensor.getInstanceOnboard();

  double thresh = 10.0;

  boolean done = false;

  public DriveTillLine() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    ColorData dataLeft = csLeft.getColor();
    ColorData dataRight = csRight.getColor();
    double leftClear = dataLeft.clear * 10000;
    double rightClear = dataRight.clear * 10000; 

    if(leftClear > thresh || rightClear > thresh)
    {
      Robot.drivebase.setMotors(0.0, 0.0);
      done = true;
    }
    else
    {
      Robot.drivebase.setMotors(-0.3, -0.3);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
