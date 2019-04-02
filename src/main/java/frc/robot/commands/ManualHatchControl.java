/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public class ManualHatchControl extends Command {

  public ManualHatchControl() {
    requires(Robot.hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.hatch.armUp = true;
    Robot.hatch.kickerUp = false;
  }

  public static void hatchDown() {
    Robot.hatch.kickerUp = false;
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean hatchActuallyUp = Robot.hatch.hatchKicker.getSensorCollection().isFwdLimitSwitchClosed();
    boolean hatchActuallyDown = Robot.hatch.hatchKicker.getSensorCollection().isRevLimitSwitchClosed();

    if (Robot.oi.getMainBButtonPressed()) {
      Robot.hatch.armUp = false;
    } else  if (Robot.oi.getMainYButtonPressed()) {
      Robot.hatch.armUp = true;
    }
    if (Robot.oi.getMainLeftBumperPressed() && !Robot.hatch.armUp) { 
      Robot.hatch.kickerUp = !Robot.hatch.kickerUp;
    }

    if (hatchActuallyUp) {
      Robot.hatch.kickerUp = false;
    }

    if (Robot.climber.getClimbTime()) {
      Robot.hatch.setRamp(0.3);
      Robot.hatch.setKicker(hatchActuallyUp ? 0.0 : 0.35);
    } else if (Robot.hatch.kickerUp) {
      //Robot.hatch.armUp = false;
      Robot.hatch.setRamp(0.0);
      Robot.hatch.setKicker(.85);
    } else if (hatchActuallyDown) {
      Robot.hatch.setKicker(0.0);
    } else {
      Robot.hatch.setRamp(0.6);
      Robot.hatch.setKicker(-0.225);
    }

    Robot.hatch.setArmUp(Robot.hatch.armUp);

    SmartDashboard.putBoolean("Hatch Kicker Reverse Limit", hatchActuallyUp);
    SmartDashboard.putBoolean("Hatch Kicker Forward Limit", hatchActuallyDown);
    SmartDashboard.putBoolean("armUp", Robot.hatch.armUp);
    SmartDashboard.putBoolean("kickerUp", Robot.hatch.kickerUp);
    SmartDashboard.putNumber("Hatch Arm Encoder", Robot.hatch.getArmEncoder());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; //change to getRevLimitSwitch if needed later
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
