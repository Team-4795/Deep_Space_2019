/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class Climb extends Command {

  private double throttle;

  public Climb() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    throttle = ( 0.6 - (0.3 * Robot.oi.getArmRightTrigger()) );
    Robot.arm.actuate(Robot.oi.getArmLeftJoyY() * throttle);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
