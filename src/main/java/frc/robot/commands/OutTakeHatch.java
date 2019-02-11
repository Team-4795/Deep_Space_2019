/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot; 

public class OutTakeHatch extends Command {
  double pushTime = 0.0;
  public OutTakeHatch(double time, double timeOut) {
    requires(Robot.hatch);
    setTimeout(timeOut);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pushTime = 0.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (pushTime == 0.0 || pushTime == 0.75) {
      if (Robot.oi.getMainXButton())
        pushTime = timeSinceInitialized();
      else {
        Robot.hatch.set(0.0);
        pushTime = 0.0;
      }
    }
    if (pushTime > 0.0 && pushTime <= 0.37) {
      Robot.hatch.set(0.25);
    } else if (pushTime > 0.37 && pushTime <= 0.75) {
      Robot.hatch.set(-0.25);
    } 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished() || isTimedOut();
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
