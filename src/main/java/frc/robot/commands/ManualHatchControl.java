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
    Robot.hatch.servoUp = true;
    Robot.hatch.hatchUp = false;
  }

  public static void hatchDown() {
    Robot.hatch.hatchUp = false;
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean hatchActuallyUp = Robot.hatch.hatchMotor.getSensorCollection().isFwdLimitSwitchClosed();
    boolean hatchActuallyDown = Robot.hatch.hatchMotor.getSensorCollection().isRevLimitSwitchClosed();

    if (Robot.oi.getMainBButtonPressed()) {
      Robot.hatch.servoUp = !Robot.hatch.servoUp;
    }
    if (Robot.oi.getMainLeftBumperPressed()) { //&& !Robot.oi.servoUp
      Robot.hatch.hatchUp = !Robot.hatch.hatchUp;
    }

    if (hatchActuallyUp) Robot.hatch.hatchUp = false;
    if (Robot.climber.getClimbTime()) {
      Robot.hatch.setRamp(0.3);
      Robot.hatch.set(hatchActuallyUp ? 0.0 : 0.35);
    } else if (Robot.hatch.hatchUp) {
      //Robot.hatch.servoUp = false;
      Robot.hatch.setRamp(0.0);
      Robot.hatch.set(.85);
    } else if (hatchActuallyDown) {
      Robot.hatch.set(0.0);
    } else {
      Robot.hatch.setRamp(0.6);
      Robot.hatch.set(-0.225);
    }

    Robot.hatch.setServoUp(Robot.hatch.servoUp);

    SmartDashboard.putBoolean("Reverse Limit", hatchActuallyUp);
    SmartDashboard.putBoolean("Forward Limit", hatchActuallyDown);
    SmartDashboard.putBoolean("servoUp", Robot.hatch.servoUp);
    SmartDashboard.putBoolean("hatchUp", Robot.hatch.hatchUp);
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
