/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

public class ArmPIDBalance extends Command {

  public ArmPIDBalance() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.arm);
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.arm.balance();
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

}
